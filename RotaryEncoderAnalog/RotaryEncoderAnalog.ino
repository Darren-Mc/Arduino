#include <arduino2.h>

#define c_EncoderPinA A1
#define c_EncoderPinB A2
#define c_EncoderPinZ A0

#define d_EncoderPinZ 2
#define d_EncoderPinA 3
#define d_EncoderPinB 4

#define d_EncoderInterruptZ 0
#define d_EncoderInterruptA 1

volatile bool _EncoderBSet;
volatile int _EncoderTicks = 0;
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
}
 
void loop()
{
  while ( Serial.available() == 0 ) {}
  while ( Serial.available() > 0 ) Serial.read();
  uint32_t n = 0;
  uint32_t start = micros();
  while ( Serial.available() == 0 )
  {
    Serial.write(analogRead(c_EncoderPinA)/4);
    Serial.write(analogRead(c_EncoderPinB)/4);
    //Serial.write(analogRead(c_EncoderPinZ)/4);
    send_ticks();
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
}

char *int2str( int num ) {
    static char retnum[11];       // Enough for 20 digits plus NUL from a 64-bit uint.
    sprintf( retnum, "%l", num );
    return retnum;
}

void send_ticks ()
{
  // get access to the float as a byte-array:
  byte * data = (byte *) &_EncoderTicks; 
  // write the data to the serial
  Serial.write (data, 2);
  //_EncoderTicks++;
}

