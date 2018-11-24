/* Funcion to calculate how long it takes to read and set the clock*/

#include <RTClib.h>
#include <Wire.h>
#include <Time.h>
#define PCF8523RTC

#ifdef PCF8523RTC
  RTC_PCF8523 rtc;
#endif

uint32_t millis_ref;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(152000);

  for(int i = 0; i < 50; i++){
    Serial.println(i);
    delay(1000);
  }

  if (!rtc.begin()){
    Serial.println("Couldn't find the RTC");
    while(1);
  }
  
  uint32_t startm = millis();
  DateTime enow = rtc.now();
  rtc.adjust(enow);
  setTime(enow.unixtime());   // Set system clock
  uint32_t endm = millis();
  millis_ref = millis() - (endm - startm);
  Serial.print("Millis ref set to: ");
  Serial.println(millis_ref);
  Serial.print("RTC read->set duration (ms): ");
  Serial.println(endm - startm);
  delay(500);

//  millis_ref = millis();
}

void loop() {
//  Function to compute the duration of reading and setting the clock
DateTime enow = rtc.now();
time_t sdatetime = now();
uint32_t cmillis = millis();
uint32_t datetime = enow.unixtime();
Serial.print("RTC epoch, System Epoch, millis: ");
Serial.print(datetime);
Serial.print(F(","));
Serial.print(sdatetime);
Serial.print(F(","));
//Serial.print("millis counter: ");
//Serial.println(cmillis);
Serial.println((cmillis - millis_ref) % 1000);
delay(950);
}
