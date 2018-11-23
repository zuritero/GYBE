//Simple Serial Printing Test

const int POT=0; // Potentiometer on analog pin 0

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);  //Start serial port, baud rate 9600

}

void loop()
{
  // put your main code here, to run repeatedly:
  int val = analogRead(POT);  // Read potentiometer
  int per = map(val, 0, 1023, 0, 100);  //Convert to percentage
  Serial.print("ASCII value: ");
  Serial.println("5");
  Serial.print(" Binary rep : ");
  Serial.write(53);
  Serial.println("");
  Serial.println("Both values should read 5 on terminal");
  //  Serial.println("%");
  delay(1000);   
}
