/* Sweep
 by BARRAGAN <http://barraganstudio.com> 
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://arduino.cc/en/Tutorial/Sweep
*/ 

#include <Servo.h> 
#include <Filter.h>
 
Servo myservo;  // create servo object to control a servo 
                // twelve servo objects can be created on most boards
 
int pos = 0;    // variable to store the servo position 
int dpos = 1;
long int count = 0;
unsigned long prev = 0;

Filter<uint16_t> w(100);
 
void setup() 
{ 
  Serial.begin(9600);
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object 
} 
 
void loop() 
{ 
  unsigned long ms = millis();
  w.push(analogRead(A0));
  count++;
  if ( ms - prev > 15 )
  {
    Serial.print(pos);
    Serial.print('\t');
    Serial.println(w.mean());
    if(pos == 0) dpos = 1;
    else if (pos == 180) dpos = -1;
    pos = pos+dpos;
    prev = millis();
    myservo.write(pos);
    count = 0;
  }
} 

