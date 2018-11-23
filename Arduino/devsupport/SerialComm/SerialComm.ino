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
  Serial.print("Analog Reading: ");
  Serial.println(val);
  Serial.print(" Percentage : ");
  Serial.print(per);
  Serial.println("%");
  delay(1000);   
}
