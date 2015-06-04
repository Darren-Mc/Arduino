/*********************************************************************************************************/
/* File: al5d.ino                                                                                        */
/* Author: Darren McMorran                                                                               */
/* Date Started: 28/05/2015                                                                              */
/*                                                                                                       */
/* Control program for the AL5D robot arm with BotBoarduino                                              */
/* BotBoarduino Manual: http://www.lynxmotion.com/images/html/build185.htm                               */
/*                                                                                                       */
/* To upload code remove Arduino reset connection (required to prevent Arduino restarting when running)  */
/*********************************************************************************************************/

/************/
/* INCLUDES */
/************/
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

/*********************************/
/* General system debugging code */
/*********************************/
//#define DEBUG // Uncomment line to add debugging code

#ifdef DEBUG      
#  define DEBUG(x) (Serial.print(x))
#  define DEBUGln(x) (Serial.println(x))
#else
#  define DEBUG(x)
#  define DEBUGln(x)
#endif // DEBUG

/***************************/
/* PIN CONNECTIONS & NAMES */
/***************************/
// Servo models taken from http://www.lynxmotion.com/images/html/build143b.htm
//
// Base Servo =        HS-485HB <-- Unsure if this is correct
// Shoulder Servo =    HS-805BB
// Elbow Servo =       HS-755HB
// Wrist Servo =       HS-422
// Gripper Servo =     HS-422 <-- Unsure if this is correct

// Analog pins used for accelerometers
#define BAS_R_PIN 0
#define BAS_H_PIN 1
#define SHL_R_PIN 0
#define SHL_H_PIN 1
#define ELB_R_PIN 2 
#define ELB_H_PIN 3
#define WRI_R_PIN 4
#define WRI_H_PIN 5

// Slider pins
#define STEP_PIN       8
#define DIR_PIN        9
#define LSWITCH_PIN    11
#define RSWITCH_PIN    12

#define LIGHT_PIN      6

/*************/
/* CONSTANTS */
/*************/
#define MAX_BUFFER     100
#define MAX_STEPS      2354 // Maximum number of steps slider can move
#define MAX_COUNT 20   // Used for stall detection in levelGripper function
#define THRESHOLD1 3   // Threshold for levelling gripper
#define THRESHOLD2 6   // Maximum range of fluctuations from accelerometer while stationary
#define THRESHOLD3 20  // Threshold used for error detection in shoulder and elbow movements

#define BASE_HEIGHT 169.0   // base height (all mm)
#define HUMERUS 146.05      // shoulder to elbow
#define ULNA 187.325        // elbow to wrist 
#define HAND 90.0           // wrist to gripper tip 

// Gradients and offsets used to convert desired angle to pulse length required for that angle
// See AL5D Servo Calibration spreadsheet for derivation
#define BAS_GRADIENT -2.478
#define SHL_GRADIENT 2.001
#define ELB_GRADIENT -2.312
#define WRI_GRADIENT -2.672

#define BAS_OFFSET 606
#define SHL_OFFSET 186.836
#define ELB_OFFSET 579.134
#define WRI_OFFSET 615.516

// Analog references for the centre voltage from accelerometers
#define BAS_R_REF 0
#define SHL_R_REF 320
#define ELB_R_REF 316
#define WRI_R_REF 0

#define BAS_H_REF 0
#define SHL_H_REF 362
#define ELB_H_REF 359
#define WRI_H_REF 362

// Gripper positions
#define GRI_OPEN 320
#define GRI_MICROPLATE 500 //500 PulseLength
#define GRI_MICROSLIDE 500 //500 PulseLength
#define GRI_CLOSED 590

// Directional definitions for controlling slider
#define LEFT           LOW
#define RIGHT          HIGH

// Servo IDs used for addressing servos in arrays
enum ServoID
{
  BAS = 0,
  SHL,
  ELB,
  WRI,
  GRI
};

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define BAS_MIN    160 // this is the pulse length count (out of 4096) that corresponds to BAS_MIN_D
#define BAS_MAX    383 // this is the pulse length count (out of 4096) that corresponds to BAS_MAX_D
#define SHL_MIN    200 // this is the pulse length count (out of 4096) that corresponds to SHL_MIN_D
#define SHL_MAX    540 // this is the pulse length count (out of 4096) that corresponds to SHL_MAX_D //HS805BB 186-612
#define ELB_MIN    160 // this is the pulse length count (out of 4096) that corresponds to ELB_MIN_D
#define ELB_MAX    540 // this is the pulse length count (out of 4096) that corresponds to ELB_MAX_D
#define WRI_MIN    130 // this is the pulse length count (out of 4096) that corresponds to WRI_MIN_D
#define WRI_MAX    500 // this is the pulse length count (out of 4096) that corresponds to WRI_MAX_D
#define GRI_MIN    320 // this is the pulse length count (out of 4096) that corresponds to GRI_MIN_D
#define GRI_MAX    590 // this is the pulse length count (out of 4096) that corresponds to GRI_MAX_D
//WRI 375 = 90
//ELB 370 = 90
//SHL 368 = 90  
//BAS 383 = 90

#define BAS_PARK 90
#define SHL_PARK 90
#define ELB_PARK 180
#define WRI_PARK 90


/*****************/
/* SERVO OBJECTS */
/*****************/
// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

/********************/
/* GLOBAL VARIABLES */
/********************/
// 'current' variables store the intended angles and 'actual' variables store the real angles used by servos
// e.g. for an elbow current angle of 180 (the elbow is in line with the shoulder) the real angle is around 0
int     i,j,count=0;
int     ret_val, g_moveDelay = 50; // g_moveDelay is defined at global scope and is used to control speed of arm
char    buf[MAX_BUFFER+1];
double   number[4], current[4], actual[4], g_angle_d[4];
int      g_pulse[5];
double   g_radius_mm, g_height_mm, g_hand_d, g_sliderPos;
double   g_prevRadius_mm, g_prevHeight_mm, g_prevBase_d, g_prevHand_d;

const double GRADIENT[] = {BAS_GRADIENT, SHL_GRADIENT, ELB_GRADIENT, WRI_GRADIENT};
const double OFFSET[] = {BAS_OFFSET, SHL_OFFSET, ELB_OFFSET, WRI_OFFSET};
const double R_PIN[] = {BAS_R_PIN, SHL_R_PIN, ELB_R_PIN, WRI_R_PIN};
const double H_PIN[] = {BAS_H_PIN, SHL_H_PIN, ELB_H_PIN, WRI_H_PIN};
const double R_REF[] = {BAS_R_REF, SHL_R_REF, ELB_R_REF, WRI_R_REF};
const double H_REF[] = {BAS_H_REF, SHL_H_REF, ELB_H_REF, WRI_H_REF};

const double SERVO_MIN[] = {BAS_MIN, SHL_MIN, ELB_MIN, WRI_MIN, GRI_MIN};
const double SERVO_MAX[] = {BAS_MAX, SHL_MAX, ELB_MAX, WRI_MAX, GRI_MAX};

/********************/
/* PRE CALCULATIONS */
/********************/
double HUM_SQ = HUMERUS*HUMERUS;
double ULN_SQ = ULNA*ULNA;

/*****************/
/* RETURN VALUES */
/*****************/
//  0: Success
// -1: Input angle below servo range
// -2: Input angle above servo range
// -3: Destination is physically out of reach
// -4: One or more servos have stalled

/*************/
/* FUNCTIONS */
/*************/
// Moves arm to specified servo angles
int moveArm( int basPulse, int shlPulse, int elbPulse, int wriPulse )
{  
   int ret_val;
   
   DEBUG("moveArm("); DEBUG(basPulse); DEBUG(","); DEBUG(shlPulse); DEBUG(","); DEBUG(elbPulse); DEBUG(","); DEBUG(wriPulse); DEBUGln(")");
   
   //Drive servos
   if ( ( ret_val = servoWritePulse( BAS, basPulse ) ) != 0 ) return ret_val;
   if ( ( ret_val = servoWritePulse( SHL, shlPulse ) ) != 0 ) return ret_val;
   if ( ( ret_val = servoWritePulse( ELB, elbPulse ) ) != 0 ) return ret_val;
   if ( ( ret_val = servoWritePulse( WRI, wriPulse ) ) != 0 ) return ret_val;
   
   //delay(g_moveDelay);
   
   return ret_val;
}

// Moves the arm slowly to a new position, depending on the specified delay
int slowMoveArm( double basAngle_d, double shlAngle_d, double elbAngle_d, double wriAngle_d )
{ 
   DEBUG("slowMoveArm("); DEBUG(basAngle_d); DEBUG(","); DEBUG(shlAngle_d); DEBUG(","); DEBUG(elbAngle_d); DEBUG(","); DEBUG(wriAngle_d); DEBUGln(")");
   
   int ret_val;
   
   int basPulse = degToPulse( BAS, basAngle_d );
   int shlPulse = degToPulse( SHL, shlAngle_d );
   int elbPulse = degToPulse( ELB, elbAngle_d );
   int wriPulse = degToPulse( WRI, wriAngle_d );
   
   int bDiff = abs( basPulse - g_pulse[BAS] );
   int sDiff = abs( shlPulse - g_pulse[SHL] );
   int eDiff = abs( elbPulse - g_pulse[ELB] );
   int wDiff = abs( wriPulse - g_pulse[WRI] );
   
   int steps = max( bDiff, sDiff );
   steps = max( steps, eDiff );
   steps = max( steps, wDiff );
   DEBUG("Steps: "); DEBUGln(steps);
   
   if ( steps == 0 ) return 0;
   
   long int delayus = round(((double)1000*g_moveDelay)/steps);
   
   DEBUG("Delayus: "); DEBUGln(delayus);
   
   if( steps < 2 )
   {
      ret_val = moveArm( basPulse, shlPulse, elbPulse, wriPulse );
      if (delayus > 1000) delay(delayus/1000);
      else delayMicroseconds(delayus);
   }
   else
   {
      double bStep = (double)(basPulse - g_pulse[BAS])/steps;
      double sStep = (double)(shlPulse - g_pulse[SHL])/steps;
      double eStep = (double)(elbPulse - g_pulse[ELB])/steps;
      double wStep = (double)(wriPulse - g_pulse[WRI])/steps;
      
      double bMove = g_pulse[BAS] + bStep;
      double sMove = g_pulse[SHL] + sStep;
      double eMove = g_pulse[ELB] + eStep;
      double wMove = g_pulse[WRI] + wStep;
      
      for(int i=0; i<steps; i++)
      {
         if( ( ret_val = moveArm( round(bMove), round(sMove), round(eMove), round(wMove) ) ) != 0 ) break;
         bMove += bStep; sMove += sStep; eMove += eStep; wMove += wStep;
         if (delayus > 1000) delay(delayus/1000);
         else delayMicroseconds(delayus);
      }
   }

   return ret_val;
}

// Arm positioning routine utilizing inverse kinematics in cylindrical coordinates
// (r = radius, h = height, b = azimuth = base servo angle, handAngle = angle of hand from horizontal)
int moveArmCoord( double radius_mm, double height_mm, double basAngle_d, double handAngle_d )
{
   if ( radius_mm < 0 ) return -4;
   DEBUG("moveArmCoord("); DEBUG(radius_mm); DEBUG(","); DEBUG(height_mm); DEBUG(","); DEBUG(basAngle_d); DEBUG(","); DEBUG(handAngle_d); DEBUGln(")");
   /* Lengths from shoulder to wrist in cylindrical coordinates */
   double s2w_h = height_mm - BASE_HEIGHT - HAND*sin( radians( handAngle_d) );
   double s2w_r = radius_mm - HAND*cos( radians( handAngle_d) );
   /* Shoulder to wrist distance ( AKA s2w ) */
   double s2w = ( s2w_h * s2w_h ) + ( s2w_r * s2w_r ); 
   double s2w_sqrt = sqrt( s2w ); // Length of line from shoulder to wrist (S-W)
   /* s2w angle to ground */
   double a1 = atan2( s2w_h, s2w_r ); // Angle between S-W line and ground
   /* s2w angle to humerus */
   double a2 = acos((( HUM_SQ - ULN_SQ ) + s2w ) / ( 2 * HUMERUS * s2w_sqrt )); // Angle between S-W line and humerus
   /* shoulder angle */
   double shlAngle_r = a1 + a2;
   /* elbow angle */
   double elbAngle_r = acos(( HUM_SQ + ULN_SQ - s2w ) / ( 2 * HUMERUS * ULNA )); // Angle between humerus and ulna
   /* wrist angle */
   double wriAngle_d = 360 + handAngle_d - degrees(shlAngle_r + elbAngle_r);
   
   return ( s2w_sqrt > HUMERUS + ULNA ) ? -3 : slowMoveArm( basAngle_d, degrees(shlAngle_r), degrees(elbAngle_r), wriAngle_d ); // Return -3 if no solution exists (out of range)
}

// Moves the arm slowly to a new position, depending on the specified delay using cylindrical coordinates
// (r = radius, h = height, b = azimuth = base servo angle)
int slowMoveArmCoord( double radius_mm, double height_mm, double basAngle_d, double handAngle_d )
{
   DEBUG("slowMoveArmCoord("); DEBUG(radius_mm); DEBUG(","); DEBUG(height_mm); DEBUG(","); DEBUG(basAngle_d); DEBUG(","); DEBUG(handAngle_d); DEBUGln(")");
   
   calcPosition();
   
   g_prevRadius_mm = g_radius_mm; g_prevHeight_mm = g_height_mm; g_prevBase_d = g_angle_d[BAS]; g_prevHand_d = g_hand_d;// Use with feedback to allow reversing after a collision
   
   int ret_val;
   double rDiff = abs( radius_mm - g_radius_mm );
   double hDiff = abs( height_mm - g_height_mm );
   double bDiff = abs( basAngle_d - g_angle_d[BAS] );
   double dDiff = abs( handAngle_d - g_hand_d );
   
   int steps = max( rDiff, hDiff );
   steps = max( steps, bDiff );
   steps = max( steps, dDiff );
   DEBUG("Steps: "); DEBUGln(steps);
   
   double rStep = ( radius_mm - g_radius_mm )/steps;
   double hStep = ( height_mm - g_height_mm )/steps;
   double bStep = ( basAngle_d - g_angle_d[BAS] )/steps;
   double dStep = ( handAngle_d - g_hand_d )/steps;
   
   double rMove = g_radius_mm + rStep;
   double hMove = g_height_mm + hStep;
   double bMove = g_angle_d[BAS] + bStep;
   double dMove = g_hand_d + dStep;
   
   for(int i=0; i<steps; i++)
   {
     if ( ( ret_val = moveArmCoord( rMove, hMove, bMove, dMove ) )  != 0 ) return ret_val;
     rMove += rStep; hMove += hStep; bMove += bStep; dMove += dStep;
   }
   
   //delay(100); // Wait for arm to stop moving
   
   //levelGripper();
   
   //delay(100); // Wait for arm to stop moving
   
   //if ( checkPosition() != 0 ) return -4; // Stall detection

   return 0;
}

// Sends signal to stepper motor to move slider in a given direction
// Returns -1 if left limit is reached, -2 for right
void slide(int dir, int steps)
{
   int i=0;
   digitalWrite(DIR_PIN, dir);
   while( i++<steps && !limitSwitch(dir) )
   { 
      digitalWrite(STEP_PIN, HIGH);
      delay(1);
      digitalWrite(STEP_PIN, LOW);
      delay(1);
      dir ? g_sliderPos++ : g_sliderPos--;
   }
   DEBUGln();
}

// Checks the limit switch in the direction slider is moving
// Returns -1 if left limit switch is hit, -2 for right
int limitSwitch(int dir)
{
   if(!dir)
   {
      if(digitalRead(LSWITCH_PIN))
      {
         g_sliderPos = 0;
         return -1;
      }
   }
   else
   {
      if(digitalRead(RSWITCH_PIN))
      {
         g_sliderPos = MAX_STEPS;
         return -2;
      }
   }
   DEBUG(g_sliderPos); DEBUG(" ");
   return 0;
}

// Read from servo internal potentiometer
int getFeedback(int analogPin)
{
   int reading[20];
   for (int j=0; j<20; j++){
      reading[j] = analogRead(analogPin);  // get raw data from servo potentiometer
      delay(1);                            // delay necessary to improve accuracy of readings
    }                                      // sort the readings low to high in array                                
    boolean done = false;                  // clear sorting flag             
    while(done != true){                   // simple swap sorts numbers from lowest to highest
    done = true;
    for (int j=0; j<20; j++){
      if (reading[j] > reading[j + 1]){    // sorting numbers here
        int test = reading[j + 1];
        reading [j+1] = reading[j] ;
        reading[j] = test;
        done = false;
       }
     }
   }
//  for (int j=0; j<20; j++){        //un-comment this for-loop to see the raw ordered data
//      Serial.print(j);
//      Serial.print("  ");
//      Serial.println(reading[j]);
//  }
    int mean = 0;
    for (int k=6; k<14; k++){      //discard the 6 highest and 6 lowest readings
      mean += reading[k];
    }
    return round((double)mean/8);   //average useful readings
}

// Changes the wrist angle until the gripper is level
double levelGripper()
{
   int ret_val, stall_count = 0;
   double h, h_old, servo_angle = actual[WRI], servo_angle_old = servo_angle;
   h = getFeedback(H_PIN[WRI]) - H_REF[WRI];
   h_old = h;
   DEBUG("servo_angle: "); DEBUG(servo_angle); DEBUG(", h: "); DEBUGln(h);
   if ( abs(h) <= THRESHOLD1 ) return 0;
   boolean condition = h > 0;
   while ( ( abs(h) > THRESHOLD1 ) && ( condition ? h > 0 : h < 0 ) )
   {
      if ( ( ret_val = servoWrite( WRI, condition ? --servo_angle : ++servo_angle ) ) != 0 ) return ret_val;
      delay(20);
      h = getFeedback(H_PIN[WRI]) - H_REF[WRI];
      DEBUG("servo_angle: "); DEBUG(servo_angle); DEBUG(", h: "); DEBUGln(h);
      if ( abs( h - h_old ) < THRESHOLD2 ) stall_count++;
      else
      {
         servo_angle_old = servo_angle;
         h_old = h;
         stall_count = 0;
      }
      if ( stall_count == MAX_COUNT )
      {
         servoWrite( WRI, servo_angle_old);
         return -4;
      }
   }
   return 0;
}

// Use accelerometer feedback to ensure the arm has reached the destination
int checkPosition()
{
   double angle_d = getAngle(SHL);
   if ( abs( angle_d - g_angle_d[SHL] ) > THRESHOLD3 ) return 1;
   DEBUG("Check1: "); DEBUGln(angle_d - g_angle_d[SHL]);
   angle_d = getAngle(ELB) - angle_d;
   DEBUG("Check2: "); DEBUGln(angle_d - g_angle_d[ELB] + 180);
   return ( abs( angle_d - g_angle_d[ELB] + 180 ) > THRESHOLD3 );
}

// Read from accelerometers and determine angle from horizontal
double getAngle(int id)
{
   double r = getFeedback(R_PIN[id]) - R_REF[id];
   double h = getFeedback(H_PIN[id]) - H_REF[id];
   double angle_d = degrees( id == ELB ? atan2(-h,-r) : atan2(-h,r) );
   DEBUG("r: "); DEBUG(r); DEBUG(", h: "); DEBUG(h); DEBUG(", angle_d: "); DEBUGln(angle_d);
   return angle_d;
}

// Write angle to servo using microseconds for added precision
int servoWrite( int id, double angle_d )
{
   //DEBUG("servoWrite("); DEBUG(id); DEBUG(", "); DEBUG(angle_deg); DEBUGln(")");
   //if (angle_d < SERVO_MIN_D[id]) { DEBUG(angle_d); DEBUG("<"); DEBUGln(SERVO_MIN_D[id]); return -1; }
   //if (angle_d > SERVO_MAX_D[id]) { DEBUG(angle_d); DEBUG(">"); DEBUGln(SERVO_MAX_D[id]); return -2; }
   
   int ret_val;
   
   if ( id == 4 ) ret_val = servoWritePulse( id, (int) round(angle_d) );
   else
   {
     ret_val = servoWritePulse( id, degToPulse( id, angle_d ) );
   }
   
   if (ret_val != 0 ) return ret_val;
   
   g_angle_d[id] = angle_d;
   
   return 0;
}

// Write to servo using pulse length
int servoWritePulse( int id, int pulseLength)
{
  if ( pulseLength == g_pulse[id] ) return 0;
  if ( pulseLength < SERVO_MIN[id] ) { DEBUG("Pulse too small ("); DEBUG(pulseLength); DEBUG("<"); DEBUG(SERVO_MIN[id]); DEBUGln(")"); return -1; }
  if ( pulseLength > SERVO_MAX[id] ) { DEBUG("Pulse too large ("); DEBUG(pulseLength); DEBUG(">"); DEBUG(SERVO_MAX[id]); DEBUGln(")"); return -2; }
  
  pwm.setPWM(id, 0, pulseLength);
  
  g_pulse[id] = pulseLength;
  
  DEBUG("pwm.setPWM("); DEBUG(id); DEBUG(", 0, "); DEBUG(pulseLength); DEBUGln(")");
  
  return 0;  
}

// Calculates the cylindrical coordinates of the wrist from the servo angles
void calcPosition()
{   
   double s = pulseToDeg( SHL, g_pulse[SHL] ), e = pulseToDeg( ELB, g_pulse[ELB] ) - 180, w = pulseToDeg( WRI, g_pulse[WRI] ) - 180;
   
   g_hand_d = s + e + w;
   
   g_radius_mm = HUMERUS*cos( radians(s) ) + ULNA*cos( radians(s + e) ) + HAND*cos( radians(g_hand_d) );
   
   g_height_mm = BASE_HEIGHT + HUMERUS*sin( radians(s) ) + ULNA*sin( radians(s + e) ) + HAND*sin( radians(g_hand_d) );
   
   DEBUG("g_radius_mm: "); DEBUG(g_radius_mm); DEBUG(", g_height_mm: "); DEBUGln(g_height_mm);
}

int degToPulse ( int id, double angle_d )
{
  return (int) round(GRADIENT[id]*angle_d + OFFSET[id]);
}

double pulseToDeg ( int id, int pulseLength )
{
  return ((double)pulseLength - OFFSET[id])/GRADIENT[id];
}

/*********/
/* SETUP */
/*********/
void setup()
{
   Serial.begin(57600);
   DEBUGln("Serial connection started\n");  
   
   pinMode(STEP_PIN, OUTPUT);
   pinMode(DIR_PIN, OUTPUT);
   pinMode(LSWITCH_PIN, INPUT);
   pinMode(RSWITCH_PIN, INPUT);
   pinMode(LIGHT_PIN, OUTPUT);
   
   pwm.begin();
  
   pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
   
   // Move arm to rest position and calculate position
   servoWrite( BAS, BAS_PARK );
   servoWrite( SHL, SHL_PARK );
   servoWrite( ELB, ELB_PARK );
   servoWrite( WRI, WRI_PARK );
   servoWrite( GRI, GRI_OPEN );
   //DEBUG("Robot parked: ("); DEBUG(g_pulse[BAS]); DEBUG(","); DEBUG(g_pulse[SHL]); DEBUG(","); DEBUG(g_pulse[ELB]); DEBUG(","); DEBUG(g_pulse[WRI]); DEBUG(","); DEBUG(g_pulse[GRI]); DEBUGln(")");
   DEBUG("Robot parked: {"); DEBUG(pulseToDeg( BAS, g_pulse[BAS] )); DEBUG(","); DEBUG(pulseToDeg( SHL, g_pulse[SHL] )); DEBUG(","); DEBUG(pulseToDeg( ELB, g_pulse[ELB] )); DEBUG(","); DEBUG(pulseToDeg( WRI, g_pulse[WRI] )); DEBUGln(",delay}");
   calcPosition();
   DEBUG("Robot parked: <"); DEBUG(g_radius_mm); DEBUG(","); DEBUG(g_height_mm); DEBUG(","); DEBUG(pulseToDeg( BAS, g_pulse[BAS] )); DEBUG(","); DEBUG(g_hand_d); DEBUGln(",delay>");
   DEBUGln("Enter r to return robot to park position\n");
   //slide(LEFT,MAX_STEPS);
    
   DEBUGln("Base Servo = 0");
   DEBUGln("Shoulder Servo = 1");
   DEBUGln("Elbow Servo = 2");
   DEBUGln("Wrist Servo = 3");
   DEBUGln("Gripper Servo = 4\n");
}

/********/
/* LOOP */
/********/
void loop() 
{
   if(Serial.available() > 0)
   {
      i=0;
      buf[i] = Serial.read(); delay(1);
      switch( buf[i] )
      {
         case 'o': 
            if( ( ret_val = servoWrite(GRI,GRI_OPEN) ) == 0 ) Serial.println("Gripper Opened"); 
            else Serial.println(ret_val);
            break;
         case 'c':
            servoWrite(GRI,GRI_CLOSED); Serial.println("Gripper Closed"); break;
         case 's':
            servoWrite(GRI,GRI_MICROSLIDE); Serial.println("Gripped Microscope Slide"); break;
         case 'p':
            servoWrite(GRI,GRI_MICROPLATE); Serial.println("Gripped Microplate"); break;
         case 'r':
            g_moveDelay = 1000;
            if( ( ret_val = slowMoveArm(BAS_PARK,SHL_PARK,ELB_PARK,WRI_PARK) ) == 0 ) Serial.println("Arm returned to park position");
            else Serial.println(ret_val);
            break;
         case 'R':
            moveArm(degToPulse(BAS, BAS_PARK), degToPulse(SHL, SHL_PARK), degToPulse(ELB, ELB_PARK), degToPulse(WRI, WRI_PARK)); Serial.println("ARM PARKED!"); break;
            
         case 'l': //start of setting light PWM value
            i=0; buf[i] = Serial.read(); delay(1);
            while( buf[i] != 'l' ) // end of light PWM value
            {
               i++;
               if( i > MAX_BUFFER ) break;
               buf[i] = Serial.read(); delay(1);
            }
            buf[i] = '\0';
            number[0] = atof(buf);
            analogWrite(LIGHT_PIN, number[0]);
            break;
            
         case '(': // start of slider move left
            i=0; buf[i] = Serial.read(); delay(1);
            while( buf[i] != ')' ) // end of slider move left
            {
               i++;
               if( i > MAX_BUFFER ) break;
               buf[i] = Serial.read(); delay(1);
            }
            buf[i] = '\0';
            number[0] = atof(buf);
            slide(LEFT, number[0]);
            break;
      
         case '[': //start of slider move right
            i=0; buf[i] = Serial.read(); delay(1);
            while( buf[i] != ']' ) // end of slider move right
            {
               i++;
               if( i > MAX_BUFFER ) break;
               buf[i] = Serial.read(); delay(1);
            }
            buf[i] = '\0';
            number[0] = atof(buf);
            slide(RIGHT, number[0]);
            break;
         
         case '|': // start of individual servo angle move packet
            i=0; buf[i] = Serial.read(); delay(1);
            while( buf[i] != '|' ) // end of individual servo angle move packet
            {
               i++;
               if( i > MAX_BUFFER ) break;
               buf[i] = Serial.read(); delay(1);
            }
            buf[i] = '\0';
            number[0] = atof(buf);
            g_moveDelay = 50;
            Serial.println(servoWrite(ELB, number[0]));
            //Serial.println(slowMoveArm(number[0],g_angle_d[SHL],g_angle_d[ELB],g_angle_d[WRI]));
            break;
            
         case '<': // start of coordinate move packet
            for(j=0; j<4; j++) // capture first 4 arguments: depth, height, base angle, hand angle
            {
               i=0;
               buf[i] = Serial.read(); delay(1);
               while( buf[i] != ',' )
               {
                  i++;
                  if( i > MAX_BUFFER ) break;
                  buf[i] = Serial.read(); delay(1);
               }
               buf[i] = '\0';
               number[j] = atof( buf );
            }
            i=0;
            buf[i] = Serial.read(); delay(1);
            while( buf[i] != '>' ) // end of packet
            {
               i++;
               if( i > MAX_BUFFER ) break;
               buf[i] = Serial.read(); delay(1);
            }
            buf[i] = '\0';
            g_moveDelay = atof( buf ); // g_moveDelay
            ret_val = slowMoveArmCoord( number[0], number[1], number[2], number[3] );
            if( ret_val == 0 )
            {
               Serial.print(number[0]); Serial.print(","); Serial.print(number[1]); Serial.print(","); Serial.print(number[2]); Serial.print(","); Serial.println(number[3]);
            }
            else
            {
               Serial.println(ret_val);
               if ( ret_val == -4 ) // If the arm did not reach the destination, return to previous position
               {
                  slowMoveArmCoord( g_prevRadius_mm, g_prevHeight_mm, g_prevBase_d, g_prevHand_d );
               }
            }
            break;
            
         case '{': // start of servo angle move packet
            for(j=0; j<4; j++) // capture first 4 arguments: basAngle_d, shlAngle_d, elbAngle_d & wriAngle_d
            {
               i=0;
               buf[i] = Serial.read(); delay(1);
               while( buf[i] != ',' )
               {
                  i++;
                  if( i > MAX_BUFFER ) break;
                  buf[i] = Serial.read(); delay(1);
               }
               buf[i] = '\0';
               number[j] = atof( buf );
            }
            i=0;
            buf[i] = Serial.read(); delay(1);
            while( buf[i] != '}' ) // end of packet
            {
               i++;
               if( i > MAX_BUFFER ) break;
               buf[i] = Serial.read(); delay(1);
            }
            buf[i] = '\0';
            g_moveDelay = atof( buf ); // g_moveDelay
            ret_val = slowMoveArm( number[0], number[1], number[2], number[3] );
            if( ret_val == 0 )
            {
               Serial.print(number[0]); Serial.print(","); Serial.print(number[1]); Serial.print(","); Serial.print(number[2]); Serial.print(","); Serial.println(number[3]);
               calcPosition();
               Serial.print("<"); Serial.print(g_radius_mm); Serial.print(","); Serial.print(g_height_mm); Serial.print(","); Serial.print(pulseToDeg( BAS, g_pulse[BAS] )); Serial.print(","); Serial.print(g_hand_d); Serial.println(",delay>");
            }
            else
            {
               Serial.println(ret_val);
            }
            break;
      }
   }
}
