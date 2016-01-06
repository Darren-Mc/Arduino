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


#define OSP_SET_WIDTH(cycles) (OCR2B = 0xff-(cycles-1))

int pos;
int maxt = MAX + OFFSET;
int mint = MIN + OFFSET;
int mid = round((float)(maxt+mint)/2.0);

// Setup the one-shot pulse generator and initialize with a pulse width that is (cycles) clock counts long

void osp_setup(uint8_t cycles) {


  TCCR2B =  0;      // Halt counter by setting clock select bits to 0 (No clock source).
              // This keeps anyhting from happeneing while we get set up

  TCNT2 = 0x00;     // Start counting at bottom. 
  OCR2A = 0;      // Set TOP to 0. This effectively keeps us from counting because the counter just keeps reseting back to 0.
          // We break out of this by manually setting the TCNT higher than 0, in which case it will count all the way up to MAX and then overflow back to 0 and get locked up again.
  OSP_SET_WIDTH(cycles);    // This also makes new OCR values get loaded frm the buffer on every clock cycle. 

  TCCR2A = _BV(COM2B0) | _BV(COM2B1) | _BV(WGM20) | _BV(WGM21); // OC2B=Set on Match, clear on BOTTOM. Mode 7 Fast PWM.
  TCCR2B = _BV(WGM22) | _BV(CS22) | _BV(CS20);         // Start counting now. WGM22=1 to select Fast PWM mode 7

  DDRD |= _BV(3);     // Set pin to output (Note that OC2B = GPIO port PD3 = Arduino Digital Pin 3)
}

// Setup the one-shot pulse generator

void osp_setup() {

  osp_setup(1);

}

// Fire a one-shot pulse. Use the most recently set width. 

#define OSP_FIRE() (TCNT2 = OCR2B - 1)

// Test there is currently a pulse still in progress

#define OSP_INPROGRESS() (TCNT2>0)

// Fire a one-shot pusle with the specififed width. 
// Order of operations in calculating m must avoid overflow of the unint8_t.
// TCNT2 starts one count lower than the match value becuase the chip will block any compare on the cycle after setting a TCNT. 

#define OSP_SET_AND_FIRE(cycles) {uint8_t m=0xff-(cycles-1); OCR2B=m;TCNT2 =m-1;}


void setup()
{
  Serial.begin(38400);
  osp_setup();
  OSP_SET_WIDTH(254);
}

void loop()
{
  // Step though 0-19 cycle long pulses for demo purposes 
  long start = micros();
  //for(int i=0; i < 150; i++)
  //{
    OSP_FIRE(); //160 = 10us
    while (OSP_INPROGRESS()){}; 
  //}
  long now = micros();
  Serial.println(now-start);
  _delay_ms(20);
  /*for (pos = mid-100; pos < mid+100; pos++) {

    OSP_SET_AND_FIRE(pos*F_CPU / 1000000);

    //while (OSP_INPROGRESS());         // This just shows how you would wait if nessisary - not nessisary in this application. 

    _delay_ms(20);      // Wait a sec to let the audience clap

  }
  
  for (pos = mid+100; pos > mid-100; pos--) {

    OSP_SET_AND_FIRE(pos*F_CPU / 1000000);

    //while (OSP_INPROGRESS());         // This just shows how you would wait if nessisary - not nessisary in this application. 

    _delay_ms(20);      // Wait a sec to let the audience clap

  }*/

}