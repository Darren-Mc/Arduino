/************/
/* INCLUDES */
/************/
#include <Servo.h> // Use of the Servo library disables analogWrite() (PWM) functionality on pins 9 and 10

/***************************/
/* PIN CONNECTIONS & NAMES */
/***************************/
#define SERVO_PIN 9

/*************/
/* CONSTANTS */
/*************/
#define MIN 740
#define MAX 2430
#define BUFLEN 100
#define MAXF 5
#define MAXA 45
/*****************/
/* SERVO OBJECTS */
/*****************/
Servo servo;

/********************/
/* GLOBAL VARIABLES */
/********************/
char buf[BUFLEN+1];
float freq, amp;
int pos;

/*********/
/* SETUP */
/*********/
void setup() {
  Serial.begin(38400);
  servo.attach(SERVO_PIN, MIN, MAX);
  servo.writeMicroseconds(pos = (MAX+MIN)/2);
}

/********/
/* LOOP */
/********/
void loop() {
  Serial.println("Enter frequency, amplitude");
  while ( Serial.available() == 0 ) {}
   int n = Serial.readBytesUntil(',',buf,BUFLEN);
   buf[n] = '\0';
   freq = atof(buf);
   n = Serial.readBytes(buf,BUFLEN);
   buf[n] = '\0';
   amp = atof(buf);
   if( !(freq > 0 && freq <= MAXF) )
   {
     Serial.print("Frequency must be between 0 and ");
     Serial.println(MAXF);
   }
   else if( !(amp > 0 && amp <= MAXA) )
   {
     Serial.print("Amplitude must be between 0 and ");
     Serial.println(MAXA);
   }
   else
   {
     Serial.print("Platform oscillating at ");
     Serial.print(freq);
     Serial.print("Hz, ");
     Serial.print(amp);
     Serial.println("Deg");
     
     float ampus = amp*(float)(MAX-MIN)/180.0;
     float mid = (float)(MAX+MIN)/2.0;
     int start = micros();
     while ( Serial.available() == 0 ) {
       pos = round(ampus*sin(2*PI*freq*(micros()-start)/1000000.0) + mid);
       servo.writeMicroseconds(pos);
     }
     while( pos != round(mid))
     {
       pos = round(ampus*sin(2*PI*freq*(micros()-start)/1000000.0) + mid);
       servo.writeMicroseconds(pos);
     }   
   }
}
