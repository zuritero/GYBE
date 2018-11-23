

/* This code is to test the full transmission of spectrometer data with other types like temp, battery. 
 * The variable types are different.* 
 * 
 */

uint16_t spec_counter;
unsigned long millis_count;
uint32_t timings;
float measuredvbat;
float measuredTemp;
uint16_t Laser404;
uint16_t White_LED;
uint16_t data[5];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
//  assign values
  spec_counter = 15;
  millis_count = 99000;
  timings = 200;
  measuredvbat = 12.3;
  measuredTemp = 23.4;
  Laser404 = 30;
  White_LED = 40;
  data[0] = 500;
  data[1] = 600;
  data[2] = 700;
  data[3] = 800;
  data[4] = 900;
//  data = {500, 600, 700, 800, 900};

  // Write buffer data
  // Need to provide pointer to the ADDRESS of the variable
  Serial.write( (uint8_t*)&spec_counter, sizeof(spec_counter) );
  Serial.write( (uint8_t*)&millis_count, sizeof(millis_count) );
  Serial.write( (uint8_t*)&timings, sizeof(timings) );
  Serial.write( (uint8_t*)&measuredvbat, sizeof(measuredvbat) );
  Serial.write( (uint8_t*)&measuredTemp, sizeof(measuredTemp) );
  Serial.write( (uint8_t*)&Laser404, sizeof(Laser404) );
  Serial.write( (uint8_t*)&White_LED, sizeof(White_LED) );
//  data is an array, which is itself a pointer to an address, reason why I don't need to reference it
  Serial.write( (uint8_t*)data, sizeof(data) );

}

void loop() {
  delay(1000);
}
