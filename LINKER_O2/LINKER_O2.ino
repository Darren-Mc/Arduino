#include <math.h>

//const int buzzerPin=3;                 //Connect the Buzzer module to Pin3, Digital 3
float thresholdVoltage= 1.84;         //The treshold for which the Buzzer should sound. 
int sensorValue;
float sensorVoltage; 
long int time;

void setup()
{
   Serial.begin(9600);                //Start the Serial connection
  //pinMode(buzzerPin,OUTPUT);            //Set the LED on Digital 12 as an OUTPUT
 //start = millis();
}

void loop()
{
sensorValue = analogRead(A0);
//sensorVoltage =(sensorValue/1024)*5.0;
time=millis();
Serial.print((float)time/1000.0);
Serial.print("\t");
Serial.println(sensorValue);
/*if(sensorVoltage<thresholdVoltage)
  {
    digitalWrite(buzzerPin,HIGH);
  }
  else
  {
  digitalWrite(buzzerPin,LOW);
  }*/
 //sensorVoltage = sensorVoltage/101*1000;
//Serial.println("the output voltage is:");

//Serial.print(sensorVoltage);
//Serial.println("mV");
while(millis()-time<100){}
//delay(100);
}
