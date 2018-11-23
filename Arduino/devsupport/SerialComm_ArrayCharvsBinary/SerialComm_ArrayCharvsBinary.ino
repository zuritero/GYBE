// Script to send binary and char through serial

const int POT=0; // Potentiometer on analog pin 0
unsigned int intArray[10] = {1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000};
//unsigned int intArray[] = {1, 2, 3, 4};
//char charArray[10] = {""};

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);  //Start serial port, baud rate 115200

}

void loop()
{
  // Need to send int array, broken up in to bytes, int = 2 bytes
  Serial.write(254);
  Serial.write( (uint8_t*)intArray, sizeof(intArray) );
//  Send a terminating value
  Serial.write(255);
  
//  // Loop to send binArray (2 byte elements)
//  for (int i = 0; i < sizeof(intArray)/sizeof(int); i++)
//  {
//    Serial.write(intArray[i], sizeof(int));
//  }
//  Serial.write(65535);  //indicator for last number
  
//  // Loop to send char version of each element of binArray
//  for (int i = 0; i < sizeof(intArray)/sizeof(int); i++)
//  {
//    Serial.println(String(intArray[i]));
//  }
//  Serial.println('End of intArray');

  delay(1000);   
}
