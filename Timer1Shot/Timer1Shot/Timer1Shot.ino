// More info about this program is here...
// http://wp.josh.com/2015/03/05/the-perfect-pulse-some-tricks-for-generating-precise-one-shots-on-avr8/

// Demo of a technique to generate narrow and precise one shot pulses using a 
// timer module on an AVR. This demo code is writen for an Arduino and uses
// the Timer2 moudle, but this techniquie should would on other AVRs and timers. 

// The demo generates one pulse per second. 
// The 1st pulse is 0 cycles long (no pulse), 
// The 2nd pulse is 1 cycle long (~63ns),
// The 3rd pulse is 2 cycles long (~126ns), 
// ...up to a total of 20 pulses, and then starts over.

// The one shot pulses are output on Digial pin 3

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
#define OFFSET -40


#define OSP_SET_WIDTH(cycles) (OCR1B = 0xffff-(cycles-1))

int pos;
int maxt = MAX + OFFSET;
int mint = MIN + OFFSET;
double mid = 191.62;
double amp = 23.30;
long start;

// Setup the one-shot pulse generator and initialize with a pulse width that is (cycles) clock counts long

void osp_setup(uint16_t cycles) {


  TCCR1B =  0;      // Halt counter by setting clock select bits to 0 (No clock source).
              // This keeps anyhting from happeneing while we get set up

  TCNT1 = 0x0000;     // Start counting at bottom. 
  OCR1A = 0;      // Set TOP to 0. This effectively keeps us from counting because the counter just keeps reseting back to 0.
          // We break out of this by manually setting the TCNT higher than 0, in which case it will count all the way up to MAX and then overflow back to 0 and get locked up again.
  OSP_SET_WIDTH(cycles);    // This also makes new OCR values get loaded frm the buffer on every clock cycle. 

  TCCR1A = _BV(COM2B0) | _BV(COM2B1) | _BV(WGM20) | _BV(WGM21); // OC2B=Set on Match, clear on BOTTOM. Mode 7 Fast PWM.
  TCCR1B = _BV(WGM22) | _BV(CS20);         // Start counting now. WGM22=1 to select Fast PWM mode 7

  DDRD |= _BV(3);     // Set pin to output (Note that OC2B = GPIO port PD3 = Arduino Digital Pin 3)
}

void timer1_setup (byte mode, int prescale, byte outmode_A, byte outmode_B, byte capture_mode)
{
  // enforce field widths for sanity
  mode &= 15 ;
  outmode_A &= 3 ;
  outmode_B &= 3 ;
  capture_mode &= 3 ;

  byte clock_mode = 0 ; // 0 means no clocking - the counter is frozen.
  switch (prescale)
  {
    case 1: clock_mode = 1 ; break ;
    case 8: clock_mode = 2 ; break ;
    case 64: clock_mode = 3 ; break ;
    case 256: clock_mode = 4 ; break ;
    case 1024: clock_mode = 5 ; break ;
    default:
      if (prescale < 0)
        clock_mode = 7 ; // external clock
  }
  TCCR1A = (outmode_A << 6) | (outmode_B << 4) | (mode & 3) ;
  TCCR1B = (capture_mode << 6) | ((mode & 0xC) << 1) | clock_mode ;
}

// Setup the one-shot pulse generator

void osp_setup() {

  timer1_setup(0x0, 1, 0x0, 0x0, 0x0);

}

// Fire a one-shot pulse. Use the most recently set width. 

#define OSP_FIRE() (TCNT1 = OCR1B - 1)

// Test there is currently a pulse still in progress

#define OSP_INPROGRESS() (TCNT1>0)

// Fire a one-shot pusle with the specififed width. 
// Order of operations in calculating m must avoid overflow of the unint8_t.
// TCNT1 starts one count lower than the match value becuase the chip will block any compare on the cycle after setting a TCNT. 

#define OSP_SET_AND_FIRE(cycles) {uint16_t m=0xffff-(cycles-1); OCR1B=m;TCNT1 =m-1;}


void setup()
{
  Serial.begin(38400);
  osp_setup();
  //OSP_SET_WIDTH(254);
  start = micros();
}

void loop()
{
  long time = micros();
  //pos = round(amp*sin(2*PI*0.5*(time-start)/1000000.0) + mid);
  noInterrupts();
  OSP_SET_AND_FIRE(250);
  while (OSP_INPROGRESS()){};
  interrupts();
  long time2 = micros();
  Serial.println(time2-time);
  _delay_ms(20);
  
  // Step though 0-19 cycle long pulses for demo purposes 
  /*long start = micros();
  //for(int i=0; i < 150; i++)
  //{
    OSP_FIRE(); //160 = 10us
    while (OSP_INPROGRESS()){}; 
  //}
  long now = micros();
  Serial.println(now-start);
  _delay_ms(20);*/
  
  /*
  for (pos = 168; pos < 192; pos++) {

    //long beginning = micros();
    OSP_SET_AND_FIRE(pos);

    //while (OSP_INPROGRESS());         // This just shows how you would wait if nessisary - not nessisary in this application. 

    //long now = micros();
    //Serial.println(now-beginning);
    delay(20);//_delay_ms(20);      // Wait a sec to let the audience clap

  }
  
  for (pos = 192; pos > 168; pos--) {
    //long beginning = micros();
    OSP_SET_AND_FIRE(pos);

    //while (OSP_INPROGRESS());         // This just shows how you would wait if nessisary - not nessisary in this application. 

    //long now = micros();
    //Serial.println(now-beginning);
    
    delay(20);//_delay_ms(20);      // Wait a sec to let the audience clap

  }*/

}
