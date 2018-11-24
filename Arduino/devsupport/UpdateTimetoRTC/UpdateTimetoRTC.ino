/*
   Function is used to update sync the system time to RTC, it will also update the RTC and system time when instructed by the user
   To update time, send serial commands: T<epoch time>
   epoch time => number of seconds since Jan 01, 1970 (10 digits long)
   Unplug for the hslr board, otherwise any rtc functions hang because rtc is unable to acknowledge due to pin routing

*/

//#include <Time.h>   // Need to include for time functionality
//#include <Arduino.h>
#include <Wire.h>
//#include <DS1307RTC.h>  // basic library for DS1307, returns time_t
#include <RTClib.h>

#define TIME_MSG_LEN 11 // time sync to PC is header followed by epoch as 10 ascii digits
#define TIME_HEADER 'T' // Header tag for serial message
#define PCF8523RTC   // precision RTC (+/- 0.2secs / day)

#ifdef PCF8523RTC   // RTC choice PCF8523
RTC_PCF8523 rtc;  // rtc object for rtc.now in Adalogger PCF8523 FeatherWing
const int chipSelect = 10;   // Use this pin for Adalogger PCF8523 Featherwing
#endif

//time_t datetime;   // epoch format, hold current date time
uint32_t datetime;

void setup() {
  //  Initiate serial communication
  Serial.begin(115200);

  // Wait until serial port is available (Feather M0 board port comm delay)
  for (int i = 0; i < 55; i++) {
    Serial.println(i);
    delay(1000);
  }

  // Check to see RTC exists
  if (!rtc.begin()) {
    Serial.println("Couldn't find the RTC");
    while (1);  // Stay here forever
  }

  Serial.println("System Time before sync: ");
  displayTime();


  #ifdef PCF8523RTC  // RTC choice PCF8523
    if (!rtc.initialized()) {
      Serial.println("RTC is NOT running!");
      // following line sets the RTC to the date & time this sketch was compiled
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
      Serial.println("RTC reset: setting time to host datetime!");
      // This line sets the RTC with an explicit date & time, for example to set
      // January 21, 2014 at 3am you would call:
      // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
    }
  #endif

}

void loop() {
  // Monitor for message from PC
  if (Serial.available())
    processSyncMessage();
  //  else
  //    Serial.println("Waiting for Sync Message");
  //  delay(1000);
}

void processSyncMessage() {
  //  Function to identify type of sync message, and execute accordingly
  char c = Serial.read();
  //  Time sync command from PC
  if (c == TIME_HEADER) {
    //    call to update time
    updateTime();
  }
  else
    Serial.println("Serial input not identified");
}

void updateTime() {
  // Function to update the RTC and system time to serial input
  // Verify input message has correct number of characters
  if (Serial.available() == 0) {
    Serial.read();  // Remove data on serial input buffer
    displayTime();
    return;
  }

  if (Serial.available() != TIME_MSG_LEN - 1) {
    Serial.println("Request to update time, but incorrect format given");
    //    Flush the Serial input buffer
    while (Serial.available())
      Serial.read();
    return;
  }

  //  time_t newtime = 0;
  uint32_t newtime = 0;
  //  Read time from PC, and covert to epoch
  for (int i = 0; i < TIME_MSG_LEN - 1; i++) {
    char c = Serial.read();
    newtime = (10 * newtime) + (c - '0');  // Convert char digit to number
  }

  //  rtc.set(newtime); // Update RTC time
  rtc.adjust(newtime);
  //  setTime(newtime); // Update system time

  displayTime();
  // Verify time was updated???

}

void displayTime() {
  // Function to display current time
  DateTime now = rtc.now();
  Serial.println("Current date/time is: ");
  Serial.print(now.year());
  printDigits(now.month());
  printDigits(now.day());
  Serial.print("_");
  printDigits(now.hour());
  Serial.print(":");
  printDigits(now.minute());
  Serial.print(":");
  printDigits(now.second());
  Serial.println();
  Serial.print("Epoch time: ");
  //  Serial.println(now());
  Serial.println(now.unixtime());
}


void printDigits(int digits) {
  // Function to print 2 digits per value
  if (digits < 10)
    Serial.print("0");

  Serial.print(digits);
}
