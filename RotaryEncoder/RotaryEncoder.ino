#include <arduino2.h>

#define c_EncoderInterruptZ 0
#define c_EncoderInterruptA 1
#define c_EncoderPinZ 2
#define c_EncoderPinA 3
#define c_EncoderPinB 4
volatile bool _EncoderBSet;
volatile long _EncoderTicks = 0;
long _last = 0;
double _speed = 0;
long _CentreCount = 0;
bool positionKnown = false;

#define	GPIO2_PREFER_SPEED	1

void test_inputs_pullup(void);

void setup()
{
  Serial.begin(115200);
 
  // Quadrature encoders
  // Left encoder
  pinMode2(c_EncoderPinA, INPUT_PULLUP);      // sets pin A as input
  pinMode2(c_EncoderPinB, INPUT_PULLUP);      // sets pin B as input
  pinMode2(c_EncoderPinZ, INPUT_PULLUP);      // sets pin B as input
  /*
  attachInterrupt(c_EncoderInterruptA, HandleInterruptA, RISING);
  attachInterrupt(c_EncoderInterruptZ, HandleInterruptZ, RISING);*/
}
 
void loop()
{
  while ( Serial.available() == 0 ) {}
  while ( Serial.available() > 0 ) Serial.read();
  while ( Serial.available() == 0 )
  {
    Serial.print(digitalRead2(c_EncoderPinA));
    Serial.print(digitalRead2(c_EncoderPinB));
    Serial.print(digitalRead2(c_EncoderPinZ));
    Serial.print('\n');
  }
  Serial.flush();
  while ( Serial.available() > 0 ) Serial.read();
}
 
// Interrupt service routines for the left motor's quadrature encoder
void HandleInterruptA()
{
  // Test transition; since the interrupt will only fire on 'rising' we don't need to read pin A
  _EncoderBSet = digitalRead(c_EncoderPinB);   // read the input pin
 
  // and adjust counter + if A leads B
    _EncoderTicks += _EncoderBSet ? -1 : +1;
    
    long us = micros();
    if(_last != 0) _speed = 5000.0f/(double)(us-_last);
    _last = us;
}

void HandleInterruptZ()
{
  // Test transition; since the interrupt will only fire on 'rising' we don't need to read pin A
  _CentreCount = _EncoderTicks;
  //long ms = millis();
  //_speed = 1000.0f/(double)(ms-_last);
  //_last = ms;
  
  positionKnown = true;
}

double Orientation()
{
  if (_EncoderTicks - _CentreCount == 0) return 0.0;
  if (_EncoderTicks - _CentreCount < 0 ) return 360.0f + 1.8f*(_EncoderTicks - _CentreCount);
  return 1.8f*(_EncoderTicks - _CentreCount);
}

void test_inputs_pullup(void)
{
  uint8_t pin;
  for ( pin=2; pin<20; pin++ )
  {
    pinMode2(pin, INPUT_PULLUP);
 
  }

  // LED pin cannot be used as input
  pinMode2(13, OUTPUT);	

  // scan all pins and print the one which is low
  // excluding pins for serial line (0 and 1)!
  // and pin for LED
  for ( pin=2; pin<20; pin++ )
  {
    if ( pin != 13 && digitalRead2(pin) == LOW )
    {
	   Serial.print("pin low: ");
	   Serial.println(pin);
	   delay(500);
    }
  }

}
  
