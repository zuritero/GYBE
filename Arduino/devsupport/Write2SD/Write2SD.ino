//Write to SD card

#include <SD.h>

//We always need to set the CS (chip select) pin
const int CS_PIN = 10;

// Provide power to SD shield, set it high
const int POW_PIN = 8;

void setup() {
  // put your setup code here, to run once:
  // Initialize serial port, and inform
  Serial.begin(115200);
  Serial.println("Initializing card");
  //CS pin is an output, from board to SD feather
//  digitalWrite(CS_PIN, OUTPUT);   // It works w/ this commented

  //Card will draw power from pin 8, set it high
  pinMode(POW_PIN, OUTPUT);
  digitalWrite(POW_PIN, HIGH);

  //Make sure SD card is initialized
  if (!SD.begin(CS_PIN))
  {
    Serial.println("Card Failure");
    return;
  
  }
  //Success
  Serial.println("Card Ready");

}

void loop() {
  // put your main code here, to run repeatedly:
  long timeStamp = millis();
  String dataString = "Hello World";
  
  //Open a file on SD, and write to it
  File dataFile = SD.open("log21.csv", FILE_WRITE);

  // write to file if created successfully
  if (dataFile)
  {
    dataFile.print(timeStamp);
    dataFile.print(",");
    dataFile.println(dataString);
    dataFile.close();   // Data isn't written until connection closes

    // Print to screen to compare
    Serial.print(timeStamp);
    Serial.print(",");
    Serial.println(dataString);
  }
  else
  {
    Serial.println("Unable to open file on SD");
  }
  delay(1000);
}
