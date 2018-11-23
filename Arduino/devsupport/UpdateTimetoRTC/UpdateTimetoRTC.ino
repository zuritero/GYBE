/*
 * Function is used to update sync the system time to RTC, it will also update the RTC and system time when instructed by the user
 * To update time, send serial commands: T<epoch time>
 * epoch time => number of seconds since Jan 01, 1970 (10 digits long)

*/

#include <Time.h>   // Need to include for time functionality
#include <Wire.h> 
#include <DS1307RTC.h>  // basic library for DS1307, returns time_t
//#include <RTClib.h>
/
#define TIME_MSG_LEN 11 // time sync to PC is header followed by epoch as 10 ascii digits
#define TIME_HEADER 'T' // Header tag for serial message


void setup() {
//  Initiate serial communication
  Serial.begin(9600);

  Serial.println("System Time before sync: ");
  displayTime();

  Serial.println("RTC Time before sync: ");
  Serial.println(RTC.get());

  if (RTC.chipPresent())
    Serial.println("RTC Chip Present");
  else
    Serial.println("RTC Chip No detected");

//  Sync system time to RTC (time keeper)
  setSyncProvider(RTC.get); // function to set system time to RTC
  if (timeStatus() != timeSet)
    Serial.println("Unable to sync with RTC");
  else {
    Serial.println("System time udpated to RTC");
    displayTime();
  }
    
}

void loop() {
// Monitor for message from PC
  if (Serial.available())
    processSyncMessage();

  delay(1000);
}

void processSyncMessage(){
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
  if (Serial.available() == 1) {
    Serial.read();  // Remove data on serial input buffer
    displayTime();
    return;
  }
  
  if (Serial.available() != TIME_MSG_LEN-1){
    Serial.println("Request to update time, but incorrect format given");
    return;
  }
  
  time_t newtime = 0;
//  Read time from PC, and covert to epoch
  for (int i=0; i<TIME_MSG_LEN-1; i++){
    char c = Serial.read();
    newtime = (10 * newtime) + (c - '0');  // Convert char digit to number
  }
  
  RTC.set(newtime); // Update RTC time
  setTime(newtime); // Update system time

  displayTime();
// Verify time was updated???

}

void displayTime(){
// Function to display current time
  Serial.println("Current date/time is: ");
  Serial.print(year());
  printDigits(month());
  printDigits(day());
  Serial.print("_");
  printDigits(hour());
  Serial.print(":");
  printDigits(minute());
  Serial.print(":");
  printDigits(second());
  Serial.println();
  Serial.print("Epoch time: ");
  Serial.println(now());
}

void printDigits(int digits) {
// Function to print 2 digits per value
  if (digits < 10)
    Serial.print("0");
    
  Serial.print(digits);
}
