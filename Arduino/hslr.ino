/* GYBE,LLC HSLR code to run Hamamatsu C12880MA with Feather M0 (+BLE) datalogger
 * 
 * Initial implmentation based on PURE engineering Arduino Uno example
  https://github.com/groupgets/c12880ma/blob/master/arduino_c12880ma_example/arduino_c12880ma_example.ino
  and also C. Versek (Northeastern U) https://github.com/open-eio/arduino-microspec

//  NOTES on Direct port-register manipulation on samd21
//REG_PORT_DIRSET0 = PORT_PA17;  // set direction of port-A PA17 to output
//REG_PORT_OUTSET0 = PORT_PA17;  // set HIGH
//REG_PORT_OUTCLR0 = PORT_PA17;  // set LOW
//REG_PORT_OUTTGL0 = PORT_PA17;  // toggle state
//REG_PORT_OUTTLGL1 = PORT_PB08; // use for port-B, as in PB08 
*/
//#include <Arduino.h>          // library for M0 BLE
#include "Adafruit_BLE.h"     // library for Adafruit BLE
#include "Adafruit_BluefruitLE_SPI.h"     // Adafruit BLE
#include "Adafruit_BluefruitLE_UART.h"    // Adafruit BLE
#include "BluefruitConfig.h"  // library for Adafruit BLE
#include <SPI.h>              // SPI library
#include <SD.h>               // connections for SD card handling
#include <Wire.h>             // added for I2C (ADC chip, RTC, etc.)
#include <RTClib.h>           // RTC library

// DEFINES N/A SINCE DIRECT PORT MANIPULATION USED:
//#define SPEC_TRG         A0  // PA02 on M0; not used presently 
#define SPEC_ST          A1    // PB08 on M0 Feather SAMD21
#define TEMP_IN          A2    // PB98 on M0; TEMP sensor center pin
#define SPEC_VIDEO       A3    // PA04 on M0 Feather SAMD21
#define WHITE_LED        A4    // PA05 on M0 Feather SAMD21
#define LASER_404        A5    // PB02 on M0 Feather SAMD21
#define SPEC_CLK         10    // PA18 on M0 Feather SAMD21
#define VBATPIN          A7    // BAT pin tied to A7 through x2 V-divider
                               // (PA07 on samd21)
#define SPEC_CHANNELS    288   // Number of spectrometer channels
                         
//---Adafruit RTC & chipselect defines--------------------------------------------------------------------*/
// Define RTC version (comment out version that's N/A)
//#define PCF8523RTC    // standard RTC (+/- 1~2secs / day)    
#define DS3231RTC   // precision RTC (+/- 0.2secs / day)

#ifdef PCF8523RTC   // RTC choice PCF8523
  RTC_PCF8523 rtc;  // rtc object for rtc.now in Adalogger PCF8523 FeatherWing
  const int chipSelect = 10;   // Use this pin for Adalogger PCF8523 Featherwing
#endif
#ifdef DS3231RTC    // RTC choice DS3231
  RTC_DS3231 rtc;   // rtc object for rtc.now in DS3231 RTC FeatherWing
  const int chipSelect = 4;      // SD pin setting on Feather M0 datalogger
#endif
//---Adafruit BLE defines--------------------------------------------------------------------*/
#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"
// Create the bluefruit object
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
// A small BLE helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}
//-----------------------------------------------------------------------
// Vars and constants declaration
const uint32_t ADCbits= pow(2,12);// Number of ADC bits for dynamic int. setting
const int ovr_samples = 20;    // Number of ADC samples that get 'averaged'; min =1
const int greenLED = 8;        // Green LED pin number to indicate SD writing
const int redLED = 13;         // Red LED pin number to indicate sensor running
const byte interruptPin = 12;  // Interrupt pin for Start-Stop Toggle
volatile boolean SSTstate = 1; // Initial SST state
uint16_t dark_count = 10;      // Number of dark spectra to capture
uint16_t ii;                   // Dark spectra counter
uint16_t data[SPEC_CHANNELS];  // Video-out bucket
uint16_t data1[SPEC_CHANNELS]; // Video-out bucket temporary
uint16_t data2[SPEC_CHANNELS]; // Video-out bucket temp
uint16_t data3[SPEC_CHANNELS]; // Video-out bucket temp
uint16_t data4[SPEC_CHANNELS]; // Video-out bucket temp
uint16_t avgdata[SPEC_CHANNELS] = {0}; // Oversample avgdata array
uint32_t spec_counter = 0;  // Spectrum counter
uint32_t millis_count;      // Milisecond time-stamp (relative)
uint32_t sinceStart_micros; // Integration Time start timestamp
uint32_t timings[10];       // Timing = miros() array
uint32_t duration_micros;
uint16_t delaymicros = 1;   // Delay in microsec (defult=1)
uint16_t min_integ_micros=0;// This is correction which is platform dependent and 
                            // measured first...
uint16_t maxvalue;          // Peak spectral intensity for dynInteg
float int_time = 0.0006;    // Default integ. time in secs
                            //    do not set below ~0.0005
float max_int_time = 2;     // Max integration time for dynamic integ. adjustment (secs)
float measuredvbat = 0;     // measured bat voltage = 2 * 3.3V * analogRead(VBATPIN) / 2^12
float measuredtemp = 0;     // measured temp = 3.3V * analogRead(TEMP_IN) / 2^12
/*
uint32_t filtsum; // vars for leaky integrator filter
int shift=2;      // increase shift for more integration samples in filter                                    
*/
File dataFile;                // Declare global File 'dataFile'
                              // to write to SD card
char filename[] = "H000.dat"; // Base SD filename with extention
char datetime_buf[16];        // Datetime buffer size big enough to store yyyymmddhhmmss

  // Define inline function analogReadFast (10bit only SAMD21)
  // adapted from Albert-Arduino-library/Albert.h
  // use prescaler bit = 2 to increase ADC conversion rate
  int inline analogReadFast(byte ADCpin, byte prescalerBits=4) // inline library functions must be in header
  { ADC->CTRLA.bit.ENABLE = 0;                     // Disable ADC
    while( ADC->STATUS.bit.SYNCBUSY == 1 );        // Wait for synchronization
    ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV64 |   // Divide Clock by 64
                     ADC_CTRLB_RESSEL_12BIT;     // ivan's guess to take it to 12bit
                     //ADC_CTRLB_RESSEL_10BIT;   // this was original
    ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1 |   // 1 sample 
                      ADC_AVGCTRL_ADJRES(0x00ul);  // Adjusting result by 0
    ADC->SAMPCTRL.reg = 0x00;                      // Sampling Time Length = 0
    ADC->CTRLA.bit.ENABLE = 1;                     // Enable ADC
  while( ADC->STATUS.bit.SYNCBUSY == 1 );          // Wait for synchronization
  return analogRead(ADCpin);
  }

/******************************************************************************/
/******************************************************************************/
void setup(){ 
  Serial.begin(115200); // Initialize serial; baud rate to 115200
  Serial.println();
  //initializeBLE();
  initializeSD();
  initializeRTC();
  initialize_c12880ma();

  // Initialize interruptPIN
  pinMode(interruptPin, INPUT_PULLUP);
//  attachInterrupt(digitalPinToInterrupt(interruptPin), SST_ISR, FALLING);
}
/******************************************************************************/
/******************************************************************************/
void loop(){
    attachInterrupt(digitalPinToInterrupt(interruptPin), SST_ISR, FALLING);
 if (SSTstate) {     
    delay(200);         // Do nothing if SSTstate change High to Low not detected
    digitalWrite(redLED,HIGH);
  }
  else {
   digitalWrite(redLED,LOW);
   spec_counter++;      // Increment spectra counter
   readSpec();          // Read spectrometer output
 //  dynInteg();          // Sets dynamic integration
   readRTClock();       // Read real-time clock (PST)
   readVBatt();         // Read battery voltage out
   readTemp();          // Read analog TMP36 temp sensor out

    //    if (spec_counter%10 == 0) {
    printDataSerial();       // Print to serial
    //printDataSD();           // Print to SD card

///////////////////////////////////
        //writeBinaryDataSerial(); // Write binary data to serial
        //writeBinaryDataSD();     // Placeholder...
    
    //    avgdata[SPEC_CHANNELS] = {0};
    //    }

    // Read dark_count # of dark meausrements every 500 samples
///////////////////////////////////
    if (spec_counter%500 == 0) {
    readDark(dark_count);
    }
///////////////////////////////////
//       printTimings();// debugging only; print time deltas to serial
///////////////////////////////////
  /*  while (ble.isConnected()) {  // Start only after BLE connection
    spec_counter++;
    readSpec();              // Read spectrometer output
    printDataSerial();       // Print to serial
    //  if (spec_counter%20 == 0) {
        printDataBLE();          // Print to BLE via UART
        //avgdata[SPEC_CHANNELS] = {0};
    //  }
    }
  */
///////////////////////////////////
   readKeyboard();          // Read keyboard inputs
  }
}
/******************************************************************************/
/******************************************************************************/
void SST_ISR(){          // Interupt Service Request for Start-Stop Toggle
  detachInterrupt(digitalPinToInterrupt(interruptPin));
  SSTstate = !SSTstate;
}

/******************************************************************************/
/******************************************************************************/
void pulse_clock(int cycles){
  for(int i = 0; i < cycles; i++){
    //digitalWrite(SPEC_CLK, HIGH);
    REG_PORT_OUTSET0 = PORT_PA18;    //Set samd21 port PA18 to HIGH
    delayMicroseconds(delaymicros);
    //digitalWrite(SPEC_CLK, LOW);
    REG_PORT_OUTCLR0 = PORT_PA18;    //Set samd21 port PA18 to LOW
    delayMicroseconds(delaymicros);
  }
}
/******************************************************************************/
/******************************************************************************/
void pulse_clock_timed(int duration_micros){
  uint32_t Start_intmicros = micros();
  while (micros() - Start_intmicros < duration_micros){
  //  digitalWrite(SPEC_CLK, HIGH);
  REG_PORT_OUTSET0 = PORT_PA18;    //Set samd21 port PA18 to HIGH
    delayMicroseconds(delaymicros);
  //  digitalWrite(SPEC_CLK, LOW);
  REG_PORT_OUTCLR0 = PORT_PA18;    //Set samd21 port PA18 to LOW
    delayMicroseconds(delaymicros);
  }
}
/******************************************************************************/
/******************************************************************************/

void readDark(int dark_count){
    ii=0;
      while(ii < dark_count){
      ii++;
      spec_counter++;      // Increment spectra counter
      readDarkSpec();      // Read Dark measurement
      readRTClock();       // Read real-time clock (PST)
      readVBatt();         // Read battery voltage out
      readTemp();          // Read Analog Dev TMP36 temp sensor out
      printDataSerial();   // Print to serial
      //printDataSD();       // Print to SD card
      }    
}

/******************************************************************************/
/******************************************************************************/
void readSpec() {
  //compute integration time
  duration_micros = (float) (int_time * 1e6);
  //duration_micros -= min_integ_micros; //correction based on 48 pulses after ST goes low
  //duration_micros = max(duration_micros,0);

  // Start clock cycle and set start pulse ST to signal start
  //digitalWrite(SPEC_CLK, HIGH);
  REG_PORT_OUTSET0 = PORT_PA18;    //Set samd21 port PA18 SPEC_CLK to HIGH
  delayMicroseconds(delaymicros);
  //digitalWrite(SPEC_CLK, LOW);
  REG_PORT_OUTCLR0 = PORT_PA18;    //Set samd21 port PA18 SPEC_CLK to LOW
  //digitalWrite(SPEC_ST, HIGH);
  REG_PORT_OUTSET1 = PORT_PB08;    //Set samd21 port PB08 SPEC_ST to HIGH
  delayMicroseconds(delaymicros);

  //Pixel integration starts after three clock pulses
  pulse_clock(3);
  sinceStart_micros = micros();  // timer at start of integration
  timings[0] = micros() - sinceStart_micros;

  //Integration Time = 4 cycles +timed int (duration_micros) +48 cycles
  pulse_clock(4);
  pulse_clock_timed(duration_micros);
  timings[1] = micros() - sinceStart_micros;
  //Set SPEC_ST to low
    //digitalWrite(SPEC_ST, LOW);
    REG_PORT_OUTCLR1 = PORT_PB08;    //Set samd21 port PB08 SPEC_ST to LOW
  timings[2] = micros() - sinceStart_micros;

  //Sample for a period of time
  //Integration stops at pulse #48 after ST went low
  pulse_clock(48);
  timings[3] = micros() - sinceStart_micros;  // measure integration time

  //pixel output is ready after last pulse #88 after ST went low
  pulse_clock(40);
  timings[4] = micros() - sinceStart_micros;

  //Read from SPEC_VIDEO
    maxvalue = 0;
    for(int i = 0; i < SPEC_CHANNELS; i++){
      data[i] = analogReadFast(SPEC_VIDEO);      // onboard ADC ~20msec sample rate
      // Get peak intensity value (digial number 0 to ADCbits-1)
      if (data[i] > maxvalue) {
         maxvalue = data[i];
      }   
      //******************************
//      avgdata[i] = avgdata[i]+data[i]/10;
//      data[i] = avgdata[i];
      //******************************
//      data[i] = filter(data[i]);  // Apply leaky integrator filter to data
//      data2[i] = analogReadFast(SPEC_VIDEO,2);  // measured 14msec sample rate (9.4msec without serial print)
//      data[i] = data2[i] - data[i];
//    data[i] = analogRead(SPEC_VIDEO);          // onboard ADC
     //******************************
      pulse_clock(1); 
      }
  
  //Store relative time stamp of data read (from Arduino clock)
  millis_count = millis();
  timings[5] = micros() - sinceStart_micros;  
}
/******************************************************************************/
/******************************************************************************/
void readDarkSpec() {
  // Hardcode integration time to minimum
  duration_micros = 0;
  //  Hardcode zero for integration time output to ID dark current measurement 
  timings[3] = 0; 
  
  // Start clock cycle but keep start pulse ST low for dark measurement
  REG_PORT_OUTSET0 = PORT_PA18;    //Set samd21 port PA18 SPEC_CLK to HIGH
  delayMicroseconds(delaymicros);
  REG_PORT_OUTCLR0 = PORT_PA18;    //Set samd21 port PA18 SPEC_CLK to LOW
  REG_PORT_OUTCLR1 = PORT_PB08;    //Set samd21 port PB08 SPEC_ST to LOW
  delayMicroseconds(delaymicros);

  //Pixel integration starts after three clock pulses
  pulse_clock(3);
  sinceStart_micros = micros();  // timer at start of integration

  //Integrate for: 4 cycles + duration_micros + 48 cycles
  pulse_clock(4);
  pulse_clock_timed(duration_micros);
  //Set SPEC_ST to low
  REG_PORT_OUTCLR1 = PORT_PB08;    // Set samd21 port PB08 SPEC_ST to LOW
                                   // Should be low, but keeping timing the same
                                   // as regular spectrum readout / measurement
  //Sample for a period of time
  //Integration stops at pulse #48 after ST went low
  pulse_clock(48);
  //Pixel output is ready after last pulse #88 after ST went low
  pulse_clock(40);

  //Read from SPEC_VIDEO
    for(int i = 0; i < SPEC_CHANNELS; i++){
      data[i] = analogReadFast(SPEC_VIDEO);      // onboard ADC ~20msec sample rate
      pulse_clock(1);  
      }
}

/******************************************************************************/
/******************************************************************************/
void dynInteg() {
  //Dynamic int_time calc. for next readSpec
    // IF saturated (ADCbits - 10), scale int time by 1/8
    if ((maxvalue > ADCbits - 10) & (int_time > 0.000001)) {
      int_time = int_time/8;
    }
    // IF under threshold (ADCbits*0.8, hardcoded for now), scale up int time to new
    // target within 90% of maxvalue
    // max_int_time is set to 2 or 3sec to guardband against slow dynamic corrections
    // NEED TO TEST to see what is needed for dark water signal!!!
    else if ((maxvalue < ADCbits*0.8) & (int_time < max_int_time)) {
      int_time = int_time + (float) (int_time *
      (ADCbits * 0.9 - maxvalue ) / ADCbits * 0.9);
    }
}
/******************************************************************************/
/******************************************************************************/
void readRTClock() {
  DateTime now = rtc.now();
    sprintf(datetime_buf,"%04u%02u%02u%02u%02u%02u",
            now.year(),
            now.month(),
            now.day(),
            now.hour(),
            now.minute(),
            now.second());
}
/******************************************************************************/
/******************************************************************************/
void readVBatt() {
  // Read LiPoly Battery Voltage if connected to Feather M0
//  delay(10);
  analogReadFast(VBATPIN);
  analogReadFast(VBATPIN);
  measuredvbat = analogReadFast(VBATPIN);
  measuredvbat /= pow(2,12);  // Convert to voltage from 12bit ADC
  measuredvbat *= 2;          // Since voltage divider divided by 2, neeed to multiply back
  measuredvbat *= 3.3;        // Multiply by 3.3V, our reference voltage
}
/******************************************************************************/
void readTemp() {
  // Read Temperature from Analog Devices TMP36 (-40 to 150C range)
  analogReadFast(TEMP_IN);
//  analogRead(TEMP_IN);
//  delay(10);
  analogReadFast(TEMP_IN);
  analogReadFast(TEMP_IN);
  measuredtemp = analogReadFast(TEMP_IN);
  measuredtemp /= pow(2,12);  // Convert to voltage from 12bit ADC
  measuredtemp *= 3.3;        // Multiply by 3.3V, our reference voltage
  measuredtemp = (measuredtemp - 0.5) * 100; //voltage to temp deg-C conversion 
                                             //10 mv per degree wit 500 mV offset
                                             // t = (voltage - 500mV) x 100
}
/******************************************************************************/
// Leaky integrator filter - chosen because of low compuation overhead
// see http://www.bot-thoughts.com/2013/07/oversampling-decimation-filtering.html for more info on the leaky integrator filter
/*uint16_t filter(uint16_t value) {
        filtsum += (value - (filtsum >> shift));
        return (filtsum >> shift);
}
*/
/******************************************************************************/
/******************************************************************************/
void printDataSerial() {
      // Log data to serial port
      Serial.print(spec_counter); // print spectrum counter
        Serial.print(F(","));     // the F ensures that the string not loaded to RAM
      Serial.print(datetime_buf); // print RTC readout
        Serial.print(F(","));     // 
      Serial.print(millis_count); // print millis() data
        Serial.print(F(","));     // 
      Serial.print(timings[3]);   // print integration time data
        Serial.print(F(","));
      Serial.print(measuredvbat); // Measured voltage of connected LiPoly bat
        Serial.print(F(","));     
      Serial.print(measuredtemp); // Measured temperature TMP36 sensor, celsius
        Serial.print(F(","));     
/*      Serial.print(0.000000);       // placeholder Measured (gpsLAT) GPS LAT
        Serial.print(F(","));      
      Serial.print(0.000000);       // placeholder Measured (gpsLON) GPS LON
        Serial.print(F(","));      
      Serial.print(0.00);           // place-holder Measured depth, meters
        Serial.print(F(","));
      Serial.print(0.00);           // place-holder IMU1 (roll)
        Serial.print(F(","));
      Serial.print(0.00);           // place-holder IMU2 (pitch)
        Serial.print(F(","));
      Serial.print(0.00);           // place-holder IMU3 (yaw)
        Serial.print(F(","));
      Serial.print(0.00);           // place-holder IMU4 (heading)
        Serial.print(F(","));
      Serial.print(0.00);           // place-holder IMU5 (altitude)
        Serial.print(F(","));
*/
      Serial.print(digitalRead(LASER_404));  // print LSR state
        Serial.print(F(","));
      Serial.print(digitalRead(WHITE_LED));  // print LED state
    // Loop over all channels to store spectra to serial out   
    for (int i = 0; i < SPEC_CHANNELS; i++){
        Serial.print(F(","));
      Serial.print(data[i]);
    }
    Serial.print(F("\n"));  // separate spectra by newline  
}

/******************************************************************************/
/******************************************************************************/
void printDataSD() {
    // Log data to SD card dataFile
    // Open SD file for writing
    dataFile = SD.open(filename, FILE_WRITE);  
    if (dataFile) {
   digitalWrite(greenLED, HIGH);    // turn green LED ON to signal write
      dataFile.print(spec_counter); // print spectrum counter
        dataFile.print(F(","));
      dataFile.print(datetime_buf);   // print RTC readout, YYYYMMDDhhmmss
        dataFile.print(F(","));      
      dataFile.print(millis_count);   // print millis() data, miliseconds
        dataFile.print(F(","));
      dataFile.print(timings[3]);     // print integration time data, microseconds
        dataFile.print(F(","));
      dataFile.print(measuredvbat);   // Measured voltage of connected LiPoly bat, V
        dataFile.print(F(","));
      dataFile.print(measuredtemp);   // Measured temperature TMP36 sensor, celsius
        dataFile.print(F(","));
/*      dataFile.print(0.000000);       // placeholder Measured (gpsLAT) GPS LAT
        dataFile.print(F(","));      
      dataFile.print(0.000000);       // placeholder Measured (gpsLON) GPS LON
        dataFile.print(F(","));      
      dataFile.print(0.00);           // place-holder Measured depth, meters
        dataFile.print(F(","));
      dataFile.print(0.00);           // place-holder IMU1 (roll)
        dataFile.print(F(","));
      dataFile.print(0.00);           // place-holder IMU2 (pitch)
        dataFile.print(F(","));
      dataFile.print(0.00);           // place-holder IMU3 (yaw)
        dataFile.print(F(","));
      dataFile.print(0.00);           // place-holder IMU4 (heading)
        dataFile.print(F(","));
      dataFile.print(0.00);           // place-holder IM5 (altitude)
        dataFile.print(F(","));
 */
      dataFile.print(digitalRead(LASER_404));  // print LSR state
        dataFile.print(F(","));
      dataFile.print(digitalRead(WHITE_LED));  // print LED state
   digitalWrite(greenLED, LOW);     // green LED off after SD write
    // Loop over all channels to store spectra to SD card
    for (int i = 0; i < SPEC_CHANNELS; i++){
        dataFile.print(F(","));
      dataFile.print(data[i]);
     }
      dataFile.print(F("\n"));
   }
  // Close SD file after writing complete
  dataFile.close();
}
/******************************************************************************/
/******************************************************************************/
void printDataBLE() {
    // Log data to BLE via UART
    // Loop over all channels to store spectra to BLE via UART   
    for (int i = 0; i < SPEC_CHANNELS; i++){
        ble.print(F(","));
      ble.print(data[i]);
    }
//    ble.println();
    ble.print(F("\n"));  // separate spectra by newline
    ble.print(F("***")); // special character
    ble.print(F("\n"));  // separate spectra by newline
    Serial.println();    // throw anotehr newline to serial out as well
}

/******************************************************************************/
/******************************************************************************/
void writeBinaryDataSerial() {
      // Log BINARY data to serial port 
//      Serial.write(byte(spec_counter)); // print spectrum counter
//      Serial.write(byte(millis_count)); // print millis() data
//      Serial.write(byte(timings[3]));   // print integration time data
//      Serial.write(byte(measuredvbat)); // Measured voltage of connected LiPoly bat
//      Serial.write(byte(measuredtemp)); // place-holder temperature (TBD)
//      Serial.write(byte(digitalRead(LASER_404)));  // print LSR state
//      Serial.write(byte(digitalRead(WHITE_LED)));  // print LED state
    // Loop over all channels to store spectra to serial out   
//    for (int i = 0; i < SPEC_CHANNELS; i++){
//      Serial.write(byte(data[i]));
//    Serial.write(byte('\n'));  // separate spectra by newline  

  byte *b = (byte *) &spec_counter; //convert 32uint_t
Serial.write(b,2);                        // write 2byte
  b = (byte *) &millis_count;       //convert 32uint_t
Serial.write(b,4);                        // write 4bytes
  b = (byte *) &timings[2];         //convert 32uint_t
Serial.write(b,4);                        // write 4bytes
  b = (byte *) &measuredvbat;       //convert float
Serial.write(b,4);                        // write 4bytes
  b = (byte *) &measuredtemp;       //convert float
Serial.write(b,4);                        // write 2bytes
Serial.write(digitalRead(LASER_404));     // write 1byte
Serial.write(digitalRead(WHITE_LED));     // write 1bytes
  for (int i = 0; i < SPEC_CHANNELS; i++){
     b = (byte *) &data[i];         //convert 32uint_t
     Serial.write(b,2);                   // write 2bytes x 288 = 576bytes
  }
Serial.write('\n'); // write line-feed char
}

/******************************************************************************/
/******************************************************************************/
/* Prints timings[1-4] vars (for debugging only);  helpful to turn off  
 *  printDataSerial at same time for readability
*/
void printTimings(){
      // Alternative print data for debugging
        Serial.print(sinceStart_micros);
        Serial.print(",");
//        Serial.print("Time to conseq micros() calls");
        Serial.print(timings[0]);
        Serial.print(",");
//        Serial.print("4clocks + Timed integ: ");
        Serial.print(timings[1]);
        Serial.print(",");
//        Serial.print("ST Low Time: ");
        Serial.print(timings[2]);
        Serial.print(",");
//        Serial.print("Integration Time: ");
        Serial.print(timings[3]);
        Serial.print(",");
//        Serial.print("Output Ready time: ");
        Serial.print(timings[4]);
        Serial.print(",");
//        Serial.print("ADC+readout complete: ");
        Serial.print(timings[5]);
        Serial.print("\n");
//        Serial.print("###");
}
/******************************************************************************/
/******************************************************************************/
/* Reads keyboard serial inputs (e.g from Processing API code), 
 * and sets integration time multiplier, and toggles LSR and LED 
 * on-off state.  Also if incomingByte == 85, both LSR and LED 
 * are set to 'off' when exiting Processing API.
 */
void readKeyboard(){ 
  int incomingByte = 0;
  // check for incoming data
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();
    if (incomingByte==93) {
      int_time = min(int_time*2,1048576);
    } // Integration time multiply by 2x
    if (incomingByte==91) {
      int_time = max(int_time/2,0);
    } // Integration Time divide by 2x
    if (incomingByte==76) {
      digitalWrite(LASER_404, !digitalRead(LASER_404));
    }  // Toggle laser ON-OFF state
    if (incomingByte==87) {
      digitalWrite(WHITE_LED, !digitalRead(WHITE_LED));
    }  // Togle white LED ON-OFF state
    if (incomingByte==120) {
      digitalWrite(LASER_404, LOW);
      digitalWrite(WHITE_LED, LOW);
      dataFile.close();
    }  // Shut down both Laser and LED upon exiting
       // and close SD card file
  }
}

/******************************************************************************/
/******************************************************************************/
/* Initialize output pins / ports on Feather M0
 *  Determine clock-pulse based minimum integration time 
 *  min_integ_micros (platrorm dependent)  
//-----------------------------*/ 
void initialize_c12880ma() {
pinMode(redLED, OUTPUT);
digitalWrite(redLED,HIGH);

  REG_PORT_DIRSET0 = PORT_PA18;   //Set samd21 port for SPEC_CLK to OUTPUT
  REG_PORT_DIRSET1 = PORT_PB08;   //Set samd21 port for SPEC_ST to OUTPUT
  REG_PORT_DIRSET1 = PORT_PB02;   //Set samd21 port for LASER_404 to OUTPUT
  REG_PORT_DIRSET0 = PORT_PA05;   //Set samd21 port for WHITE_LED to OUTPUT

  // Start with CLK Low (doesn't seem to matter;  used to be High)
  //  digitalWrite(SPEC_CLK, LOW); 
  REG_PORT_OUTCLR0 = PORT_PA18;    //Set samd21 port PA18 to HIGH
  // Start with ST Low
  //  digitalWrite(SPEC_ST, LOW);
  REG_PORT_OUTCLR1 = PORT_PB08;    //Set samd21 port PB08 to LOW
  // MEASURE minimum integration time (platform dependent)
  // 48 clock cycles are required after ST goes low 
  sinceStart_micros = micros();
  pulse_clock(48);
  min_integ_micros = (int)(micros() - sinceStart_micros); 
}

/******************************************************************************/
/******************************************************************************/
/* Initialize SD card:  create open and name new SD card file, and 
 *  set-up LED pattern
//-----------------------------*/ 
// SD card setup on M0 Feather
  // Initialize built-in LED and set to low
  // use to indicate SD operation otherwise off
void initializeSD() {
  pinMode(greenLED, OUTPUT);
  digitalWrite(greenLED,HIGH);

  // Create, open and name the SD card file
  // Note that only one file can be open at a time.
  pinMode(chipSelect, OUTPUT);
  SD.begin(chipSelect);
  // Generate filename;  increment name by 1 (00 to 999)
  for (uint8_t i = 0; i < 1000; i++) {
   filename[1] = i/100 + '0'; // division by 100
   filename[2] = (i/10)%10 + '0';  // division by 10 and remainder
   filename[3] = i%10 + '0';  // modulo second digit
     if (!SD.exists(filename)) {
      // Open a new file only if it doesn't exist;  no overwrites
      break;  // leave the loop
      }
    }
}
/******************************************************************************/
/******************************************************************************/
/* Initialize RTC modeule
//-----------------------------*/ 
void initializeRTC(){
if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
#ifdef PCF8523RTC  // RTC choice PCF8523
  if (!rtc.initialized()) {
    Serial.println("RTC is NOT running!");
        // following line sets the RTC to the date & time this sketch was compiled
    // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));  Serial.println("RTC reset: setting time to host datetime!");
        // This line sets the RTC with an explicit date & time, for example to set
        // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }
#endif
#ifdef DS3231RTC  // RTC choice RTC_DS3231
  if (rtc.lostPower()) {
    Serial.println("RTC lost power: setting time to host datetime!");
        // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
        // This line sets the RTC with an explicit date & time, for example to set
        // January 21, 2014 at 3am you would call:
        // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }  
#endif
}
/******************************************************************************/
/******************************************************************************/
/* Initialize BLE modeule, set LED pattern and switch to 'DATA' mode
//-----------------------------*/ 
// Initialize the BLE module
void initializeBLE(){
  if ( !ble.begin(VERBOSE_MODE) )
  {
  // error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  // Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    // Perform a BLE factory reset to known state
    // Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
    // error(F("Couldn't factory reset"));
    }
  }
  ble.echo(false);    // Disable command echo from Bluefruit
  ble.verbose(false); // debug info is a little annoying after this point!
  // Wait for start only after connection
  //  while (!ble.isConnected()) {
  //    delay(500);
  //  }

  // Change Mode LED Activity
  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) ) {
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }
  // Switch module to DATA mode
  ble.setMode(BLUEFRUIT_MODE_DATA);
}

