#define c_EncoderPinA A0
#define c_EncoderPinB A1
#define c_EncoderPinZ A2

void setup()
{
  Serial.begin(115200);
 
  // Quadrature encoders
  // Left encoder
  digitalWrite(A0, HIGH);
  digitalWrite(A1, HIGH);
  digitalWrite(A2, HIGH);
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
  
