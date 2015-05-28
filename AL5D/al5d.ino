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
#include <Servo.h> // Use of the Servo library disables analogWrite() (PWM) functionality on pins 9 and 10

/*********************************/
/* General system debugging code */
/*********************************/
#define DEBUG // Uncomment line to add debugging code

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
// Base Servo =        HS-485HB <-- Unsure if this is correct    -Matt: seems correct
// Shoulder Servo =    HS-805BB
// Elbow Servo =       HS-755HB
// Wrist Servo =       HS-645MG                                  -Matt: Actually servo says HS-422
// Gripper Servo =     HS-485HB <-- Unsure if this is correct    -Matt: pretty sure it's the HS-422

// Servo pins
#define BAS_PIN  2
#define SHL_PIN  3
#define ELB_PIN  4
#define WRI_PIN  10
#define GRI_PIN  5

// Analog pins used for accelerometers
#define BAS_R_PIN 0                    // There is currently no accelerometer on the base, and I see no need for them but I will leave this redudancy here as it causes no issues
#define BAS_H_PIN 1                   
#define SHL_R_PIN 0                   
#define SHL_H_PIN 1
#define ELB_R_PIN 2 
#define ELB_H_PIN 3
#define WRI_R_PIN 4                    // Currently not used
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
#define MAX_COUNT 20   // Used for stall detection in levelGripper function  Was 20
#define THRESHOLD1 50   // Threshold for levelling gripper  Was 3
#define THRESHOLD2 55   // Maximum range of fluctuations from accelerometer while stationary
#define THRESHOLD3 65 // Threshold used for error detection in shoulder and elbow movements Was 11

#define BASE_HEIGHT 169.1   // base height (all mm)
#define HUMERUS 146.05      // shoulder to elbow
#define ULNA 157.9        // elbow to wrist 
#define HAND 108.0           // wrist to gripper tip 

// Gradients and offsets used to adjust servo angles so that measured angles are closer to input
// See C:\Users\loam\Documents\Darren\Servo Angles v2.ods for derivation
#define BAS_GRADIENT 1.011
#define SHL_GRADIENT 0.815
#define ELB_GRADIENT -0.880 // Elbow servo moves opposite direction to others
#define WRI_GRADIENT 1.052
#define GRI_GRADIENT 1

#define BAS_OFFSET 5
#define SHL_OFFSET 13.968
#define ELB_OFFSET 165.267 // Elbow servo moves opposite direction to others
#define WRI_OFFSET -3.758
#define GRI_OFFSET 0

// Analog references for the centre voltage from accelerometers
#define BAS_R_REF 0
#define SHL_R_REF 484.848 
#define ELB_R_REF 460  // Was 478.788
#define WRI_R_REF 0

#define BAS_H_REF 0
#define SHL_H_REF 520 // Was 548.485
#define ELB_H_REF 530  // Was 543.939
#define WRI_H_REF 511.5

// Gripper positions
#define GRI_OPEN 100
#define GRI_MICROPLATE 160
#define GRI_MICROSLIDE 110
#define GRI_CLOSED 180
#define GRI_TUBE 150

// Directional definitions for controlling slider
#define LEFT           LOW
#define RIGHT          HIGH

// Servo IDs used for addressing servos in arrays
#define BAS 0
#define SHL 1
#define ELB 2
#define WRI 3
#define GRI 4

/*****************/
/* SERVO OBJECTS */
/*****************/
Servo servo[5]; // Array of servos, use ID to address each individual servo

/********************/
/* GLOBAL VARIABLES */
/********************/
// 'current' variables store the intended angles and 'actual' variables store the real angles used by servos
// e.g. for an elbow current angle of 180 (the elbow is in line with the shoulder) the real angle is around 0
int     i,j,count=0;
int     ret_val, move_delay = 50; // move_delay is defined at global scope and is used to control speed of arm
char    buf[MAX_BUFFER+1];
float   number[4], current[4], actual[4];
float   current_g, current_r, current_h, current_l;
float   old_r, old_h, old_b;

const float gradient[] = {BAS_GRADIENT, SHL_GRADIENT, ELB_GRADIENT, WRI_GRADIENT, GRI_GRADIENT};
const float offset[] = {BAS_OFFSET, SHL_OFFSET, ELB_OFFSET, WRI_OFFSET, GRI_OFFSET};
const float R_PIN[] = {BAS_R_PIN, SHL_R_PIN, ELB_R_PIN, WRI_R_PIN};
const float H_PIN[] = {BAS_H_PIN, SHL_H_PIN, ELB_H_PIN, WRI_H_PIN};
const float R_REF[] = {BAS_R_REF, SHL_R_REF, ELB_R_REF, WRI_R_REF};
const float H_REF[] = {BAS_H_REF, SHL_H_REF, ELB_H_REF, WRI_H_REF};

/********************/
/* PRE CALCULATIONS */
/********************/
float hum_sq = HUMERUS*HUMERUS;
float uln_sq = ULNA*ULNA;

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
int moveArm( float bas_angle_d, float shl_angle_d, float elb_angle_d, float wri_angle_d )
{  
   int ret_val;
   
   DEBUG("moveArm("); DEBUG(bas_angle_d); DEBUG(","); DEBUG(shl_angle_d); DEBUG(","); DEBUG(elb_angle_d); DEBUG(","); DEBUG(wri_angle_d); DEBUG(")-->(");
   
   DEBUG(adjust(BAS, bas_angle_d)); DEBUG(","); DEBUG(adjust(SHL, shl_angle_d)); DEBUG(","); DEBUG(adjust(ELB, elb_angle_d)); DEBUG(","); DEBUG(adjust(WRI, wri_angle_d)); DEBUGln(")");
   //Drive servos
   if ( ( ret_val = servoWrite( BAS, adjust(BAS, bas_angle_d) ) ) != 0 ) return ret_val;
   if ( ( ret_val = servoWrite( SHL, adjust(SHL, shl_angle_d) ) ) != 0 ) return ret_val;
   if ( ( ret_val = servoWrite( ELB, adjust(ELB, elb_angle_d) ) ) != 0 ) return ret_val;
   if ( ( ret_val = servoWrite( WRI, adjust(WRI, wri_angle_d) ) ) != 0 ) return ret_val;
   
   
   delay(move_delay);
   //Serial.print(getAngle(SHL));  Serial.print(","); Serial.print(getAngle(ELB)); Serial.print(",");  Serial.print(getAngle(WRI)); Serial.print(","); Serial.println();
   Serial.print(analogRead(R_PIN[SHL])); Serial.print(","); Serial.print(analogRead(H_PIN[SHL])); Serial.print(","); Serial.print(analogRead(R_PIN[ELB])); Serial.print(","); Serial.print(analogRead(H_PIN[ELB]));  Serial.print(","); Serial.println(analogRead(H_PIN[WRI]));
   
   return ret_val;
}

// Moves the arm slowly to a new position, depending on the specified delay
// If the last input angle is negative, the function will determine the wrist angle required to achieve horizontal position
int slowMoveArm( float bas_angle_d, float shl_angle_d, float elb_angle_d, float wri_angle_d )
{ 
   if ( wri_angle_d < 0 ) wri_angle_d = current[WRI] - current[SHL] + shl_angle_d - current[ELB] + elb_angle_d;             //Matt: I believe this calculates the required angle of the wrist in order for it to be horizontal. This relies on the sum of the current angles being a constant and so is equal to the sum of the destination angles. I think this is geometrically correct if the wrist is always to be pointed in the same direction.
   
   if ( wri_angle_d < 0 ) wri_angle_d = 0;                           // If the desired wrist angle is out of bounds, take it as far as it can go
   else if ( wri_angle_d > 180 ) wri_angle_d = 180;
   
   DEBUG("slowMoveArm("); DEBUG(bas_angle_d); DEBUG(","); DEBUG(shl_angle_d); DEBUG(","); DEBUG(elb_angle_d); DEBUG(","); DEBUG(wri_angle_d); DEBUGln(")");
   
   int ret_val;
   float diff_b = abs( bas_angle_d - current[BAS] );
   float diff_s = abs( shl_angle_d - current[SHL] );
   float diff_e = abs( elb_angle_d - current[ELB] );
   float diff_w = abs( wri_angle_d - current[WRI] );
   
   float steps = max( diff_b, diff_s );
   steps = max( steps, diff_e );
   steps = max( steps, diff_w );
   steps = round(steps);
   DEBUG("Steps: "); DEBUGln(steps);
   
   if( steps < 2 )
   {
      ret_val = moveArm( bas_angle_d, shl_angle_d, elb_angle_d, wri_angle_d );
   }
   else
   {
      float b_step = (bas_angle_d - current[BAS])/steps;
      float s_step = (shl_angle_d - current[SHL])/steps;
      float e_step = (elb_angle_d - current[ELB])/steps;
      float w_step = (wri_angle_d - current[WRI])/steps;
      
      float b_move = current[BAS] + b_step;
      float s_move = current[SHL] + s_step;
      float e_move = current[ELB] + e_step;
      float w_move = current[WRI] + w_step;
      
      for(int i=0; i<steps; i++)
      {
         if( ( ret_val = moveArm( b_move, s_move, e_move, w_move ) ) != 0 ) break;
         b_move += b_step; s_move += s_step; e_move += e_step; w_move += w_step;
      }
   }

   return ret_val;
}

// Arm positioning routine utilizing inverse kinematics in cylindrical coordinates
// (r = radius, h = height, b = azimuth = base servo angle)
int moveArmCoord( float radius_mm, float height_mm, float bas_angle_d )
{
   DEBUG("moveArmCoord("); DEBUG(radius_mm); DEBUG(","); DEBUG(height_mm); DEBUG(","); DEBUG(bas_angle_d); DEBUGln(")");
   /* Wrist position */
   float wri_h = height_mm - BASE_HEIGHT;
   float wri_r = radius_mm;
   /* Shoulder to wrist distance ( AKA sw ) */
   float s_w = ( wri_h * wri_h ) + ( wri_r * wri_r ); 
   float s_w_sqrt = sqrt( s_w ); // Length of line from shoulder to wrist (S-W)
   /* s_w angle to ground */
   float a1 = atan2( wri_h, wri_r ); // Angle between S-W line and ground
   /* s_w angle to humerus */
   float a2 = acos((( hum_sq - uln_sq ) + s_w ) / ( 2 * HUMERUS * s_w_sqrt )); // Angle between S-W line and humerus - found using the cosine law
   /* shoulder angle */
   float shl_angle_r = a1 + a2;
   float shl_angle_d = degrees( shl_angle_r );
   /* elbow angle */
   float elb_angle_r = acos(( hum_sq + uln_sq - s_w ) / ( 2 * HUMERUS * ULNA )); // Angle between humerus and ulna
   float elb_angle_d = degrees( elb_angle_r ); // Shifted so that neutral position is 180 degrees
   
   return ( s_w_sqrt > HUMERUS + ULNA ) ? -3 : slowMoveArm( bas_angle_d, shl_angle_d, elb_angle_d, -1 ); // Return -3 if no solution exists (out of range)
}

// Moves the arm slowly to a new position, depending on the specified delay using cylindrical coordinates
// (r = radius, h = height, b = azimuth = base servo angle)
int slowMoveArmCoord( float radius_mm, float height_mm, float bas_angle_d )
{
   DEBUG("slowMoveArmCoord("); DEBUG(radius_mm); DEBUG(","); DEBUG(height_mm); DEBUG(","); DEBUG(bas_angle_d); DEBUGln(")");
   
  calcPosition();
   
   old_r = current_r; old_h = current_h; old_b = current[BAS]; // Use with feedback to allow reversing after a collision
   
   int ret_val;
   float diff_r = abs( radius_mm - current_r );
   float diff_h = abs( height_mm - current_h );
   float diff_b = abs( bas_angle_d - current[BAS] );
   
   int steps = max( diff_r, diff_h );
   steps = max( steps, diff_b );
   DEBUG("Steps: "); DEBUGln(steps);
   
   float r_step = ( radius_mm - current_r )/steps;
   float h_step = ( height_mm - current_h )/steps;
   float b_step = ( bas_angle_d - current[BAS] )/steps;
   
   float r_move = current_r + r_step;
   float h_move = current_h + h_step;
   float b_move = current[BAS] + b_step;
   
   for(int i=0; i<steps; i++)
   {
     if ( ( ret_val = moveArmCoord( r_move, h_move, b_move ) )  != 0 ) return ret_val;
     r_move += r_step; h_move += h_step; b_move += b_step;
   }
   
  
   delay(100); // Wait for arm to stop moving
   
   levelGripper();

   delay(100); // Wait for arm to stop moving
   
   //check position
   if ( checkPosition() != 0 ) return -4; // Stall detection
   return 0;
}

// Adjust the servo angle to compensate for servo error
float adjust( int id, float angle_d )
{
   return gradient[id]*angle_d + offset[id];
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
      dir ? current_l++ : current_l--;
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
         current_l = 0;
         return -1;
      }
   }
   else
   {
      if(digitalRead(RSWITCH_PIN))
      {
         current_l = MAX_STEPS;
         return -2;
      }
   }
   DEBUG(current_l); DEBUG(" ");
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
    return round((float)mean/8);   //average useful readings
}

// Changes the wrist angle until the gripper is level
float levelGripper()
{
   int ret_val, stall_count = 0;
   float h, h_old, servo_angle = actual[WRI], servo_angle_old = servo_angle;
   h = H_REF[WRI]- getFeedback(H_PIN[WRI]) ;
   h_old = h;
   DEBUG("servo_angle: "); DEBUG(servo_angle); DEBUG(", h: "); DEBUGln(h);
   if ( abs(h) <= THRESHOLD1 ) return 0;
   boolean condition = h > 0;
   while ( ( abs(h) > THRESHOLD1 ) && ( condition ? h > 0 : h < 0 ) )
   {
      //Serial.println(h-h_old);
      if ( ( ret_val = servoWrite( WRI, condition ? --servo_angle : ++servo_angle ) ) != 0 ) return ret_val;
      delay(20);
      h = H_REF[WRI]- getFeedback(H_PIN[WRI]);
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
   float angle_d = getAngle(SHL);
   //Serial.print( angle_d - current[SHL]); 
   if ( abs( angle_d - current[SHL] ) > THRESHOLD3 ) return 1;
   DEBUG("Check1: "); DEBUGln(angle_d - current[SHL]);
   angle_d = getAngle(ELB) - angle_d;
   //Serial.print(","); Serial.println(angle_d - current[ELB] + 180);
   DEBUG("Check2: "); DEBUGln(angle_d - current[ELB] + 180);
   return ( abs( angle_d - current[ELB] + 180 ) > THRESHOLD3 );
  
}

// Read from accelerometers and determine angle from horizontal
float getAngle(int id)
{
   float r = getFeedback(R_PIN[id]) - R_REF[id];
   float h = getFeedback(H_PIN[id]) - H_REF[id];
   float angle_d = degrees( id == ELB ? atan2(-h,-r) : atan2(-h,r) );
   DEBUG("r: "); DEBUG(r); DEBUG(", h: "); DEBUG(h); DEBUG(", angle_d: "); DEBUGln(angle_d);
   return angle_d;
}

// Write angle to servo using microseconds for added precision
int servoWrite( int id, float angle_deg )
{
//   DEBUG("servoWrite("); DEBUG(id); DEBUG(", "); DEBUG(angle_deg); DEBUGln(")");
   int angle_us = round( 600 + 10*angle_deg );
   if ( angle_us < 600 )
   {
      if ( angle_deg < round( gradient[id] * (gradient[id] > 0 ? 0 : 180) + offset[id] ) ) return -1; // Allows inputs from 0-180 before adjustments are made
      angle_us = 600; DEBUGln("Clipped at 0");
   }
   if ( angle_us > 2400 )
   {
      if ( angle_deg > round( gradient[id] * (gradient[id] > 0 ? 180 : 0) + offset[id] ) ) return -2; // Allows inputs from 0-180 before adjustments are made
      angle_us = 2400; DEBUGln("Clipped at 180");
   }
   
   servo[id].writeMicroseconds( angle_us );
   actual[id] = angle_deg;
   current[id] = (angle_deg - offset[id])/gradient[id];
   
   return 0;
}

// Calculates the cylindrical coordinates of the wrist from the servo angles
void calcPosition()
{   
   float s = current[SHL]; float e = current[ELB] - 180;
   
   current_r = HUMERUS*cos( radians(s) ) + ULNA*cos( radians(s + e) );
   
   current_h = BASE_HEIGHT + HUMERUS*sin( radians(s) ) + ULNA*sin( radians(s + e) );
   
   DEBUG("current_r: "); DEBUG(current_r); DEBUG(", current_h: "); DEBUGln(current_h);
}

/*********/
/* SETUP */
/*********/
void setup()
{
   Serial.begin(57600);
   DEBUGln("Serial connection started\n");  
   
   analogReference(EXTERNAL);      // Sets the analog reference voltage to the voltage supplied to the AREF pin on the board - here this is recieving the 3.3V output from one of the accelerometers.  This sets the upper limit of values read by analogRead(). Since the accelerometers output 0-3.3V, the default upper limit of 5V wastes the top output of analogRead.
   
   for(int pinTest = 0; pinTest < 6; pinTest++){
      for(int testNum = 0; testNum < 3; testNum++){
          analogRead(pinTest);                          // Upon changing the reference voltage the first few readings may be incorrect so they are done here.
      }
   }
   
   pinMode(STEP_PIN, OUTPUT);
   pinMode(DIR_PIN, OUTPUT);
   pinMode(LSWITCH_PIN, INPUT);
   pinMode(RSWITCH_PIN, INPUT);
   pinMode(LIGHT_PIN, OUTPUT);
   
   servo[BAS].attach(BAS_PIN, 600, 2400);
   servo[GRI].attach(GRI_PIN, 600, 2400);
   servo[WRI].attach(WRI_PIN, 600, 2400);
   servo[SHL].attach(SHL_PIN, 600, 2400);
   servo[ELB].attach(ELB_PIN, 600, 2400);

   // Move arm to rest position and calculate position
   servoWrite(BAS,adjust(BAS,90));
   servoWrite(SHL,adjust(SHL,135));
   servoWrite(ELB,adjust(ELB,45));
   servoWrite(WRI,adjust(WRI,90));
   servoWrite(GRI,GRI_OPEN);
   calcPosition();
   slide(LEFT,MAX_STEPS);
    
   DEBUGln("Base Servo Attached - Pin 2");
   DEBUGln("Shoulder Servo Attached - Pin 3");
   DEBUGln("Elbow Servo Attached - Pin 4");
   DEBUGln("Wrist Servo Attached - Pin 10");
   DEBUGln("Gripper Servo Attached - Pin 11\n");
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
            servoWrite(GRI,GRI_OPEN); Serial.println("Gripper Opened"); break;
         case 'c':
            servoWrite(GRI,GRI_CLOSED); Serial.println("Gripper Closed"); break;
         case 's':
            servoWrite(GRI,GRI_MICROSLIDE); Serial.println("Gripped Microscope Slide"); break;
         case 'p':
            servoWrite(GRI,GRI_MICROPLATE); Serial.println("Gripped Microplate"); break;
         case 't':   
            servoWrite(GRI,GRI_TUBE); Serial.println("Gripped Tube"); break;
         case 'r':
            move_delay = 20;
            if( ( ret_val = slowMoveArm(90,135,45,90) ) == 0 ) Serial.println("Arm moved to rest position");
            else Serial.println(ret_val);
            break;
         case 'R':
            moveArm(90,90,180,0); Serial.println("Arm moved to rest position"); break;
            
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
            move_delay = 50;
            Serial.println(slowMoveArm(current[BAS],current[SHL],current[ELB],number[0]));
            break;
            
         case '<': // start of coordinate move packet
            for(j=0; j<3; j++) // capture first 3 arguments: depth, height, base angle          
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
            move_delay = atof( buf ); // move_delay
            ret_val = slowMoveArmCoord( number[0], number[1], number[2] );
            if( ret_val == 0 )
            {
               Serial.print(number[0]); Serial.print(","); Serial.print(number[1]); Serial.print(","); Serial.println(number[2]);
            }
            else
            {
               Serial.println(ret_val);
               if ( ret_val == -4 ) // If the arm did not reach the destination, return to previous position
               {
                  slowMoveArmCoord( old_r, old_h, old_b );
               }
            }
            break;
            
         case '{': // start of servo angle move packet
            for(j=0; j<4; j++) // capture first 4 arguments: bas_angle_d, shl_angle_d, elb_angle_d & wri_angle_d
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
            move_delay = atof( buf ); // move_delay
            ret_val = slowMoveArm( number[0], number[1], number[2], number[3] );
            if( ret_val == 0 )
            {
               Serial.print(number[0]); Serial.print(","); Serial.print(number[1]); Serial.print(","); Serial.println(number[2]);
            }
            else
            {
               Serial.println(ret_val);
            }
            break;
      }
   }
}

