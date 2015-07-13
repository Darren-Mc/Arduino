#include <arduino2.h>

#define c_EncoderPinA A0
#define c_EncoderPinB A1
#define c_EncoderPinZ A2

#define d_EncoderPinZ 2
#define d_EncoderPinA 3
#define d_EncoderPinB 4

#define d_EncoderInterruptZ 0
#define d_EncoderInterruptA 1

volatile bool _EncoderBSet;
volatile long _EncoderTicks = 0;
long _last = 0;
double _speed = 0;
long _CentreCount = 0;
bool positionKnown = false;

#define	GPIO2_PREFER_SPEED	1

void setup()
{
  Serial.begin(115200);
 
  // Quadrature encoders
  // Left encoder
  pinMode2(c_EncoderPinA, INPUT_PULLUP);      // sets pin A as input
  pinMode2(c_EncoderPinB, INPUT_PULLUP);      // sets pin B as input
  pinMode2(c_EncoderPinZ, INPUT_PULLUP);      // sets pin B as input
  digitalWrite(A0, HIGH);
  digitalWrite(A1, HIGH);
  digitalWrite(A2, HIGH);
  
  attachInterrupt(d_EncoderInterruptA, HandleInterruptA, RISING);
  attachInterrupt(d_EncoderInterruptZ, HandleInterruptZ, RISING);
}
 
void loop()
{
  while ( Serial.available() == 0 ) {}
  while ( Serial.available() > 0 ) Serial.read();
  uint32_t n = 0;
  uint32_t start = micros();
  while ( Serial.available() == 0 )
  {
    Serial.write(analogRead(A0)/4);
    Serial.write(analogRead(A1)/4);
    Serial.write(analogRead(A2)/4);
    n++;
  }
  Serial.flush();
  Serial.println();
  Serial.println((double)(micros()-start)/n);
  while ( Serial.available() > 0 ) Serial.read();
}

// Interrupt service routines for the left motor's quadrature encoder
void HandleInterruptA()
{
  // Test transition; since the interrupt will only fire on 'rising' we don't need to read pin A
  _EncoderBSet = digitalRead2(d_EncoderPinB);   // read the input pin
 
  // and adjust counter + if A leads B
    _EncoderTicks += _EncoderBSet ? -1 : +1;
    
    long us = micros();
    if(_last != 0) _speed = 5000.0f/(double)(us-_last);
    _last = us;
}
