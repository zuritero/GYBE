/* GYBE,LLC HSLR code to run Hamamatsu C12880MA with Feather M0 +BLE) datalogger
 * 
 * Initial implmentation based on work in PURE engineering Arduino Uno example
  https://github.com/groupgets/c12880ma/blob/master/arduino_c12880ma_example/arduino_c12880ma_example.ino
  and also C. Versek (Northeastern U) https://github.com/open-eio/arduino-microspec

//  Direct port-register manipulation on samd21
//REG_PORT_DIRSET0 = PORT_PA17;  // set direction of port-A PA17 to output
//REG_PORT_OUTSET0 = PORT_PA17;  // set HIGH
//REG_PORT_OUTCLR0 = PORT_PA17;  // set LOW
//REG_PORT_OUTTGL0 = PORT_PA17;  // toggle state
//REG_PORT_OUTTLGL1 = PORT_PB08; // use for port-B, as in PB08 
*/
#include <Arduino.h>          // library for M0 BLE
#include "Adafruit_BLE.h"     // library for Adafruit BLE
#include "Adafruit_BluefruitLE_SPI.h"     // Adafruit BLE
#include "Adafruit_BluefruitLE_UART.h"    // Adafruit BLE
#include "BluefruitConfig.h"  // library for Adafruit BLE
#include <SPI.h>              // SPI library
#include <SD.h>               // connections for SD card handling
#include <Wire.h>             // added for ADC
//#include "Adafruit_ADS1015.h" // added for ADC incl i2c
//Adafruit_ADS1115 ads1115(0x48); // construct an ads1115 at address 
                                // 0x48 (GND address pin)
//#define SPEC_TRG         A0  // PA02 on M0; not used presently 
#define SPEC_ST          A1    // PB08 on M0 Feather SAMD21
#define TEMPPIN          A2    // TEMP pin tied to A2 (PB09 on samd21)
#define SPEC_VIDEO       A3    // PA04 on M0 Feather SAMD21
#define WHITE_LED        A4    // PA05 on M0 Feather SAMD21
#define LASER_404        A5    // PB02 on M0 Feather SAMD21
#define SPEC_CLK         10    // PA18 on M0 Feather SAMD21
#define VBATPIN          A7    // BAT pin tied to A7 through x2 V-divider
                               // (PA07 on samd21)
#define SPEC_CHANNELS    288   // Number of spectrometer channels
   
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
const int chipSelect = 4;      // SD pin setting on Feather M0 datalogger
const int greenLED = 8;        // Green LED pin number
const int OVRsamples = 20;     //minumum value =1
                               // number of ADC samples that get 'averaged'
uint16_t data[SPEC_CHANNELS];  // Video-out bucket
uint16_t data1[SPEC_CHANNELS]; // Video-out bucket temporary
uint16_t data2[SPEC_CHANNELS]; // Video-out bucket temp
uint16_t data3[SPEC_CHANNELS]; // Video-out bucket temp
uint16_t data4[SPEC_CHANNELS]; // Video-out bucket temp
uint16_t avgdata[SPEC_CHANNELS] = {0}; // Oversample avgdata array
uint16_t spec_counter = 0;  // Spectrum counter
uint32_t timings[10];       // Timing = miros() array
uint32_t sinceStart_micros; // Integration Time start timestamp
uint16_t delaymicros = 1;   // Delay in microsec (defult=1)
uint16_t min_integ_micros=0;// This is correction which is platform dependent and 
                            // measured first...
unsigned long millis_count; // Milisecond time-stamp (relative)
uint32_t duration_micros;
float int_time = 0.001;     // Default integ. time in msecs
                            //    do not set below ~0.0005;
float measuredvbat = 0;     // measured bat voltage = 2 * 3.3V * analogRead(VBATPIN) / 2^12
float measuredtemp = 0;     // measured temp = 3.3V * analogRead(TEMPPIN) / 2^12
/*
uint32_t filtsum; // vars for leaky integrator filter
int shift=2;      // increase shift for more integration samples in filter                                    
*/
File dataFile;                // Declare global File 'dataFile'
                              // to write to SD card
char filename[] = "H000.dat"; // Base SD filename with extention

// Define inline function analogReadFast (10bit only SAMD21)
// adapted from Albert-Arduino-library/Albert.h
// use prescaler bit = 2 to increase ADC conversion rate
int inline analogReadFast(byte ADCpin, byte prescalerBits=4) // inline library functions must be in header
  { ADC->CTRLA.bit.ENABLE = 0;                     // Disable ADC
    while( ADC->STATUS.bit.SYNCBUSY == 1 );        // Wait for synchronization
    ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV64 |   // Divide Clock by 64
                     ADC_CTRLB_RESSEL_12BIT;     // ivan's guess to take it to 12bit
//                     ADC_CTRLB_RESSEL_10BIT;   // this was original
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

//  ads1115.begin();      // Initialize ads1115
//  //ads1115.setGain(GAIN_TWO);  // 2x gain amplifier
//  //if not set, GAIN_TWOTHIRDS is default; +/- 6.144V  1 bit = 3mV
//  // FOR ADS1115 I also also changed in Adafruit_ADS1015.h library  
//  // #define ADS1115_CONVERSIONDELAY (8) --> changed to (1)   // msec
//  // and also
//  // #define ADS1015_REG_CONFIG_DR_1600SPS (0x0080) to (0xE0) //  

//-----------------------------// 
// Initialize the BLE module
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
//-----------------------------//
// SD card setup on M0 Feather
  // Initialize built-in LED and set to low
  // use to indicate SD operation otherwise off
  pinMode(greenLED, OUTPUT);
  digitalWrite(greenLED,LOW);

  // Create, open and name the SD card file
  // Note that only one file can be open at a time.
  pinMode(chipSelect, OUTPUT);
  SD.begin(chipSelect);
  // Generate filename;  increment name by 1 (00 to 999)
  for (uint8_t i = 0; i < 1000; i++) {
    filename[1] = i/100 + '0'; // division by 100
    filename[2] = i/10 + '0';  // division by 10
    filename[3] = i%10 + '0';  // modulo second digit
     if (!SD.exists(filename)) {
      // Open a new file only if it doesn't exist;  no overwrites
      break;  // leave the loop
      }
    }
//-----------------------------//
  initialize_c12880ma();
  // readSpec();        // read spectrometer data and
  // printDataSerial(); // write to serial once to remove data in buffer
}

/******************************************************************************/
/******************************************************************************/
void loop(){
   spec_counter++;          // Increment spectra counter
   readSpec();              // Read spectrometer output
   readVBatt();             // Read battery voltage out
   readTemp();              // Read Analog Dev TMP36 temp sensor out

    //    if (spec_counter%10 == 0) {
      printDataSerial();       // Print to serial
    //    avgdata[SPEC_CHANNELS] = {0};
    //    }

//       printTimings();          // Print time deltas to Serial; for debugging only
    //   printDataSD();           // Print to SD card

  /*  while (ble.isConnected()) {  // Start only after BLE connection
    spec_counter++;
    readSpec();              // Read spectrometer output
    printDataSerial();       // Print to serial
      if (spec_counter%20 == 0) {
        printDataBLE();          // Print to BLE via UART
        //avgdata[SPEC_CHANNELS] = {0};
      }
    }
  */
   readKeyboard();          // Read keyboard inputs
}
/******************************************************************************/
/******************************************************************************/
void initialize_c12880ma() {
//  analogReadResolution(12);    //(SAMD21) 12-bit ADC capability
  //  pinMode(SPEC_CLK, OUTPUT);   //Set SPEC_CLK pin to OUTPUT
  REG_PORT_DIRSET0 = PORT_PA18;    //Set desired samd21 port to OUTPUT
  //  pinMode(SPEC_ST, OUTPUT);    //Set SPEC_ST pin to OUTPUT
  REG_PORT_DIRSET1 = PORT_PB08;    //Set desired samd21 port to OUTPUT
  //  digitalWrite(SPEC_CLK, LOW); // Start with CLK High
  REG_PORT_OUTCLR0 = PORT_PA18;    //Set samd21 port PA18 to HIGH
  //  digitalWrite(SPEC_ST, LOW);  // Start with ST Low
  REG_PORT_OUTCLR1 = PORT_PB08;    //Set samd21 port PB08 to LOW
  // MEASURE minimum integration time (platform dependent)
  // 48 clock cycles are required after ST goes low 
  sinceStart_micros = micros();
  pulse_clock(48);
  min_integ_micros = (int)(micros() - sinceStart_micros);  
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
void readSpec() {
  //compute integration time
  duration_micros = (float) (int_time * 1e6);
  //duration_micros -= min_integ_micros; //correction based on 48 pulses after ST goes low
  //duration_micros = max(duration_micros,0);

  // Start clock cycle and set start pulse ST to signal start
  //digitalWrite(SPEC_CLK, HIGH);
  REG_PORT_OUTSET0 = PORT_PA18;    //Set samd21 port PA18 to HIGH
  delayMicroseconds(delaymicros);
  //digitalWrite(SPEC_CLK, LOW);
  REG_PORT_OUTCLR0 = PORT_PA18;    //Set samd21 port PA18 to LOW
  //digitalWrite(SPEC_ST, HIGH);
  REG_PORT_OUTSET1 = PORT_PB08;    //Set samd21 port PB08 to HIGH
  delayMicroseconds(delaymicros);

  //Pixel integration starts after three clock pulses
  pulse_clock(3);
  sinceStart_micros = micros();  // timer at start of integration
//  timings[0] = micros() - sinceStart_micros;

  //Integrate for: 4 cycles + duration_micros + 48 cycles
  pulse_clock(4);
  pulse_clock_timed(duration_micros);
  timings[1] = micros() - sinceStart_micros;
  //Set SPEC_ST to low
    //digitalWrite(SPEC_ST, LOW);
    REG_PORT_OUTCLR1 = PORT_PB08;    //Set samd21 port PB08 to LOW
  timings[2] = micros() - sinceStart_micros;

  //Sample for a period of time
  //Integration stops at pulse #48 after ST went low
  pulse_clock(48);
//  timings[3] = micros() - sinceStart_micros;

  //pixel output is ready after last pulse #88 after ST went low
  pulse_clock(40);
//  timings[4] = micros() - sinceStart_micros;

  //Read from SPEC_VIDEO
    for(int i = 0; i < SPEC_CHANNELS; i++){
      data[i] = analogReadFast(SPEC_VIDEO);      // onboard ADC ~20msec sample rate
      //******************************
//      avgdata[i] = avgdata[i]+data[i]/10;
//      data[i] = avgdata[i];
      //******************************
//      data[i] = filter(data[i]);  // Apply leaky integrator filter to data
//      data2[i] = analogReadFast(SPEC_VIDEO,2);  // measured 14msec sample rate (9.4msec without serial print)
//      data[i] = data2[i] - data[i];
//    data[i] = analogRead(SPEC_VIDEO);          // onboard ADC
//    data4[i] = ads1115.readADC_SingleEnded(0);  // ADS1115 read
      pulse_clock(1);
    }

  //Store relative time stamp of data read (from Arduino clock)
  millis_count = millis();
  timings[5] = micros() - sinceStart_micros;  
}
/******************************************************************************/
void readVBatt() {
  // Read LiPoly Battery Voltage if connected to Feather M0
//  delay(10);
  analogReadFast(VBATPIN);
  measuredvbat = analogReadFast(VBATPIN);
  measuredvbat /= pow(2,12);  // Convert to voltage from 12bit ADC
  measuredvbat *= 2;          // Since voltage divider divided by 2, neeed to multiply back
  measuredvbat *= 3.3;        // Multiply by 3.3V, our reference voltage
}
/******************************************************************************/
void readTemp() {
  // Read Temperature from Analog Devices TMP36 (-40 to 150C range)
  analogReadFast(TEMPPIN);
//  analogRead(TEMPPIN);
//  delay(10);
  analogReadFast(TEMPPIN);
  analogReadFast(TEMPPIN);
  measuredtemp = analogReadFast(TEMPPIN);
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
      Serial.print(millis_count); // print millis() data
        Serial.print(F(","));     // the F ensures that the string not loaded to RAM
      Serial.print(timings[2]);   // print integration time data
        Serial.print(F(","));
      Serial.print(measuredvbat); // Measured voltage of connected LiPoly bat
        Serial.print(F(","));     
      Serial.print(measuredtemp); // place-holder temperature (TBD)
        Serial.print(F(","));
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
/* Prints timings[1-4] vars (for debugging only);  helpful to turn off  
 *  printDataSerial at same time for readability
*/
void printTimings(){
      // Alternative print data for debugging
        Serial.print(sinceStart_micros);
        Serial.print(",");
//        Serial.print("4clocks + Timed integ: ");
        Serial.print(timings[0]);
        Serial.print(",");
//        Serial.print("ST Low Time: ");
        Serial.print(timings[1]);
        Serial.print(",");
//        Serial.print("Integration Time: ");
        Serial.print(timings[2]);
        Serial.print(",");
//        Serial.print("Output Ready time: ");
        Serial.print(timings[3]);
        Serial.print(",");
//        Serial.print("ADC+readout time: ");
        Serial.print(timings[4]);
        Serial.print("\n");
//        Serial.print("###");
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
      dataFile.print(millis_count); // print millis() data
        dataFile.print(F(","));
      dataFile.print(timings[2]);   // print integration time data
        dataFile.print(F(","));
      dataFile.print(measuredvbat);      // Measured voltage of connected LiPoly bat
        dataFile.print(F(","));
      dataFile.print(F("0.0"));     // place-holder for temperature (TBD)
        dataFile.print(F(","));
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
   digitalWrite(LED_BUILTIN, LOW);   // SD-LED off after SD write
   }
  // Close SD file after writing complete
  dataFile.close();
}
/******************************************************************************/
/******************************************************************************/
void printDataBLE() {
      // Log data to BLE via UART
/*      ble.print(spec_counter);  // print spectrum counter
        ble.print(F(","));
      ble.print(millis_count);  // print millis() data
        ble.print(F(","));
      ble.print(timings[2]);  // print integration time data
        ble.print(F(","));
      ble.print(measuredvbat);      // Measured voltage of connected LiPoly bat
        ble.print(F(","));          // the F ensures that the string not loaded to RAM
      ble.print(F("0.0"));          // place-holder temperature (TBD)
        ble.print(F(",")); 
      ble.print(digitalRead(LASER_404));  // print LSR state
        ble.print(F(","));
      ble.print(digitalRead(WHITE_LED));  // print LED state
*/    // Loop over all channels to store spectra to BLE via UART   
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

