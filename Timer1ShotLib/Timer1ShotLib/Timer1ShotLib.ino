/************/
/* INCLUDES */
/************/
#include <digitalWriteFast.h>
//#include <TimerOne.h>
#define	GPIO2_PREFER_SPEED	1
#include <arduino2.h>

/*************/
/* CONSTANTS */
/*************/
#define RESOLUTION 65536
#define BUFLEN 100
#define MAXF 5
#define MAXA 45

/********************/
/* GLOBAL VARIABLES */
/********************/
int maxus = 2390; // 20 ms
int mid = 1545;
double deg2us = 845 / 90.0;
long start = 0;
unsigned int pwmPeriod;
unsigned char clockSelectBits;
char oldSREG;
char buf[BUFLEN+1];
float freq, amp;
int pos;

/*************/
/* FUNCTIONS */
/*************/
void timerSetup(int duty) {
  TCCR1A = 0;                 // clear control register A 
  TCCR1B = 0; //_BV(WGM13);        // set mode 8: phase and frequency correct pwm, stop the timer
  TCNT1 = 0x00;
  long cycles = (F_CPU / 1000000) * maxus;                                      // the counter runs backwards after TOP, interrupt is at BOTTOM so divide microseconds by 2
  if(cycles < RESOLUTION)              clockSelectBits = _BV(CS10);              // no prescale, full xtal
  else if((cycles >>= 3) < RESOLUTION) clockSelectBits = _BV(CS11);              // prescale by /8
  else if((cycles >>= 3) < RESOLUTION) clockSelectBits = _BV(CS11) | _BV(CS10);  // prescale by /64
  else if((cycles >>= 2) < RESOLUTION) clockSelectBits = _BV(CS12);              // prescale by /256
  else if((cycles >>= 2) < RESOLUTION) clockSelectBits = _BV(CS12) | _BV(CS10);  // prescale by /1024
  else        cycles = RESOLUTION - 1, clockSelectBits = _BV(CS12) | _BV(CS10);  // request was out of bounds, set as maximum
  
  oldSREG = SREG;				
  cli();							// Disable interrupts for 16 bit register access
  ICR1 = 0;//pwmPeriod = cycles;                                          // ICR1 is TOP in p & f correct pwm mode
  //OCR1A = 0;  // Set TOP to 0. This effectively keeps us from counting becuase the counter just keeps reseting back to 0.
          // We break out of this by manually setting the TCNT higher than 0, in which case it will count all the way up to MAX and then overflow back to 0 and get locked up again.
  //OCR1B = 512;
  SREG = oldSREG;
  
  TCCR1A |= _BV(COM1A0) | _BV(COM1A1);// | _BV(WGM10) | _BV(WGM11);                    // activates the output pin
  // OC2B=Set on Match, clear on BOTTOM. Mode 7 Fast PWM.
  
  TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));         // Ensure clock mode bits are cleared
  TCCR1B |= _BV(WGM13) | clockSelectBits;                              // Set clock mode / prescaler
  
  
  DDRB |= _BV(PORTB1);                                   // sets data direction register for pwm output pin
  
  //DDRD |= _BV(3);     // Set pin to output (Note that OC2B = GPIO port PD3 = Arduino Digital Pin 3)
    
  //unsigned long dutyCycle = pwmPeriod;
  
  //dutyCycle *= duty;
  //dutyCycle >>= 10;
  
  //Serial.println(dutyCycle);
  
  oldSREG = SREG;
  cli();
  OCR1B = duty;//dutyCycle;
  SREG = oldSREG;
  
  TCCR1B |= clockSelectBits;		// Lex - make sure the clock is running.  We don't want to restart the count, in case we are starting the second WGM
					// and the first one is in the middle of a cycle
}

/*********/
/* SETUP */
/*********/
void setup() {
  // put your setup code here, to run once:
  Serial.begin(38400);
  pinMode2(8, OUTPUT);
  timerSetup(512);
  //start = micros();
}

/********/
/* LOOP */
/********/
void loop() {
  // put your main code here, to run repeatedly:
  //long t1 = micros();
  //while(TCNT1>0){}
  //long t2 = micros();
  //long t1 = micros();
  //cli();
  //TCNT1=16*1500;
  Serial.println("Enter frequency, amplitude");
  pos = 16*mid;
  
  while ( Serial.available() == 0 ) {
       TCNT1 = pos;
       digitalWrite2(8, HIGH);
       while(TCNT1>0);
       digitalWrite2(8, LOW);
       delay(20);
  }
  
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
     
     float ampus = amp*deg2us;
     long start = micros();
     
     while ( Serial.available() == 0 ) {
       long time = micros();
       pos = round(16*(ampus*sin(2*PI*freq*(time-start)/1000000.0) + mid));
       TCNT1 = pos;
       digitalWrite2(8, HIGH);
       while(TCNT1>0);
       digitalWrite2(8, LOW);
       delay(20);
     }
     
     bool above = pos > 16*mid;    
   
     while (pos != 16*mid) { //( ( above && pos > 16*mid ) || ( !above && pos < 16*mid ) ) {
       long time = micros();
       pos = round(16*(ampus*sin(2*PI*freq*(time-start)/1000000.0) + mid));
       TCNT1 = pos;
       digitalWrite2(8, HIGH);
       while(TCNT1>0);
       digitalWrite2(8, LOW);
       delay(20);
     }
   }
  //long time = micros();
  //TCNT1 = round(16*(200*sin(2*PI*0.5*(time-start)/1000000.0) + 1500));
  //digitalWrite2(8, HIGH);
  //while(TCNT1>0);
  //digitalWrite2(8, LOW);
  //long t2 = micros();
  //Serial.println(t2-time);
  //delay(20);
  //long t2 = micros();
  //Serial.println(t2-t1);
  //delay(20);
}
