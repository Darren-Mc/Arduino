#include <math.h>

const int buzzerPin=3;                 //Connect the Buzzer module to Pin3, Digital 3
float thresholdVoltage= 1.84;         //The treshold for which the Buzzer should sound. 

void setup()
{
   Serial.begin(9600);                //Start the Serial connection
  pinMode(buzzerPin,OUTPUT);            //Set the LED on Digital 12 as an OUTPUT
 
}

void loop()
{

float sensorValue;
float sensorVoltage; 
sensorValue = analogRead(A2);
sensorVoltage =(sensorValue/1024)*5.0;

Serial.println(sensorVoltage);
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
delay(100);
}
