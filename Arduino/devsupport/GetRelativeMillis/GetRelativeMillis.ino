/*
 * Function is used to test out getting absolute time down to milliseconds, which would be used to link with spectrometer acquisitions

*/

#include <Time.h>   // Need to include for time functionality

// Use for simulating variable roll over (millis() counter)
uint32_t Maxmillis = 1000*20;  // 15 minutes
uint32_t pushtorollover = 4294960000; // help simulate rollover millis counter
uint32_t  refmillis = millis(); // Get reference millis (0 ms marker)
time_t  reftime;  // epoch: seconds since Jan 1, 1970
//uint32_t reftime = now();
uint32_t  old_refmillis = refmillis;  // to be sued for debugging


void setup() {
  delay(1568);  
//  set the reference millis (done here to simulate start of delay)
  refmillis = millis();
  old_refmillis = refmillis;
  
  Serial.begin(115200);
  //  Verify size of variable time_t
  Serial.print("Size of time_t variable in bytes: ");
  Serial.println(sizeof(reftime));
//  sprintf(buf, "Size of time_t variable: %02d", sizeof(reftime));
//  Serial.println(buf);
  Serial.print("Initial clock in seconds: ");;
  Serial.println(reftime);
//  sprintf(buf, "Initial clock in seconds: %02d", reftime);
//  Serial.println(buf);
  Serial.print("Ref Millis initalized to: ");
  Serial.println(refmillis);
  
}

void loop() {
  // put your main code here, to run repeatedly:

// Curent time in seconds (starts counter if not updated from computer
  Serial.print("Current time in seconds: ");
  Serial.println(now());
//  Get current millis, after the sec marker
  uint32_t curmillis = millis();
  Serial.print("Absolute milliseconds :");
  Serial.println(curmillis);
//  if curmillis < refmillis; 
  Serial.print("Milliseconds after sec marker: ");
  Serial.println((curmillis - refmillis) % 1000);

//// Check to see if variable has rolledover (sim: exceeding Max value )
//  if (curmillis > Maxmillis) {
//    SetNewMillisRef(curmillis);
//  }

// Simulate millis counter rolliver over
  curmillis += pushtorollover;
  Serial.print("current millis plus push to rollover: ");
  Serial.println(curmillis);

  if (curmillis < refmillis) {
    SetNewMillisRef(curmillis);
  }

  delay(2351);
  
}

/***********************************************************/

void SetNewMillisRef(uint32_t millisin){
  // update the old millisecond ref; debugging purposes
  old_refmillis = refmillis;
  
  Serial.println("Inside SetNewMillisRef function");
// compute Gap to reach Max value (32 bit = 4,294,967,295)
  uint8_t  msGap2MaxValue =  (4294967295 - refmillis) % 1000;  // ms gap to reach max value
  Serial.print("millisecond gap to Max value: ");
  Serial.println(msGap2MaxValue);
//  Compute new millisecond reference, have to use 999 because the next milli is a second
  uint32_t NewMillisRef = 999 - msGap2MaxValue;
  Serial.print("New Millisecond Reference: ");
  Serial.println(NewMillisRef);


// compute milliseconds after sec marker
  uint16_t postmillis = (millisin - NewMillisRef) % 1000;

// Not Common - Current millis less than the new sec marker reference
  if (postmillis < 0) {
    Serial.println("current millis was less than New Millis Ref");
    postmillis = postmillis + msGap2MaxValue;  // Gap to Max should be added, it shouldn't add up to more than 1000 ms
  }
  Serial.print("Milliseconds since the turn of the second marker clock: ");
  Serial.println(postmillis);

// Need to update the global millisecond reference
  refmillis = NewMillisRef;
}
