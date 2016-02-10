/*
	Keepon controller: v 0.4 Second Version With Speed Control.
	-----------------------------------------------------------
	Listens for serial input in the following forms:
                PA160E // one command for one servo to arrive at 160 degrees with default speed
		PA090S020E //one command for one servo with angle and a target speed (20)
                RA090TA000E //several commands for multiple servos with only angles and default speed
                RA090S090TA000S200BA000E //several commands for multiple servos with speed and angles (some migh have only angles)
                S100E //adjust the default speed
                
	where R, B, T, P means (R)oll, (B)op, (T)ilt and (P)an. S means (S)peed.
        You can also add a U in the middle (don't use between numbers) of any command to make that command be (U)nstoppable.
        Unstoppable means that it will block the servo until it reaches the target position.
        The code needs refactoring and the message parsing could be improved with some error detection.  
        
	Ahsan Nawroj
	Henny Admoni
	Brad Hayes
        André Pereira
	Updated: 02/04/2015
*/

#include <Arduino.h>
#include <string.h>
#include <VarSpeedServo.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <SoftwareSerial.h>

// --------------------------------------------------------------------------------
//
//  PRE-PROCESSOR DIRECTIVES, DEFINITIONS, CONSTANTS
//
// --------------------------------------------------------------------------------
#define SERVO_PIN(x) (x+2)	// all servo pins are connected from pin 2 onwards
#define ROLL 0
#define BOP 1
#define TILT 2
#define PAN 3		
#define INITIAL_SERVO_SPEED 80		
#define DEFAULT_ANGLE 90


// ------------------------------------------------------------------ --------------
//
//  G L O B A L S
//
// --------------------------------------------------------------------------------
VarSpeedServo keeponServos[4];	                // 0 = roll, 1 = bop, 2 = tilt, 3 = pan
unsigned int baud_rate = 9600;
String commandString= String(); // string containing motor commands
unsigned int currentSpeed=INITIAL_SERVO_SPEED; //the variable we are using to set speed commands

// --------------------------------------------------------------------------------
//
//  M A I N   D R I V E R   F U N C T I O N S
//
// --------------------------------------------------------------------------------
void setup () {
	// Open serial connection over main hardware serial port (USB)
	Serial.begin(baud_rate);

	// Initialize servos
	resetMotorPositions();
}

void loop () {
       
       if (Serial.available() <= 0) {
	  return; // leave right away
       }
       
       char newByte;       
       while (Serial.available() > 0) {	                   // if there is something on the buffer,
	    newByte = (char)Serial.read();	           // read in the message one byte at a time
	    commandString += newByte;

            if (newByte == 'E') break;  // and break on the termination char
	}

      if(newByte=='E') //If it was a valid message
      {
        bool currentUnstoppable=false; // it is false by default it will be used to sendMotorCommand
        unsigned int currentAngle=DEFAULT_ANGLE; //the variable we are using to set angles
        char currentMotor='N'; //None
        int i=0; //setup a counter for the while cycle
        if (commandString[0]=='H') {
            readMotorValues();
            commandString= String();
         } else {
            while(1){ //Until it reaches the end of the command 
              if(commandString[i]=='P' || commandString[i]=='T' || commandString[i]=='R' || commandString[i]=='B' ||commandString[i]=='E') //If there is a motor command or the end of the command
              {
               if(currentMotor!='N') //If there is a command to execute
               {   
                 sendMotorCommand(currentMotor, currentSpeed, currentUnstoppable, currentAngle); //Execute the motor command
                 currentMotor=commandString[i]; //Set the target motor to the new one, executed in all commands with the exception of the first
               }
               else if(commandString[i]!='E'){ //Only executed in the first command and does not do any motor commands
                 currentMotor=commandString[i]; //Set the target motor to the new one
               }
               if(commandString[i]=='E')
               {
                 commandString=String(); //empties the commandString
                 currentMotor=='N'; //empties the current motor char with a char symbolozing NONE
                 break;
               }
             }  
             else if (commandString[i]=='A') //If we are trying to set the angle of a servo
                       currentAngle=(commandString.substring(i+1,i+4)).toInt(); //set our variable to that angle
             else if (commandString[i]=='S') //If we are trying to adjust the speed of a servo
                       currentSpeed=(commandString.substring(i+1,i+4)).toInt(); //set our variable to that speed
             else if (commandString[i]=='U') //If we want our current servo command to be Unstoppable
                       currentUnstoppable=true; //We set our variable for true
             i++; //increase the counter
            }
        }
      }
}

// --------------------------------------------------------------------------------
//
//  U T I L I T Y   F U N C T I O N S
//
// --------------------------------------------------------------------------------

/**
 * Sets a motor command using absolute positioning
 *
 * @param motor The motor number (use #defines like PAN)
 * @param absoluteAngle Angle for motor to achieve
 * @param delay Set with the amount of movement delay to expect
 * @return bool True if sent a motor command, false if no movement
 */
void sendMotorCommand(char motorChar, unsigned int servoSpeed, bool blockServo, unsigned int absoluteAngle) {
        //Convert from the char we got in the message to the ints specified in the constants
        unsigned int motor;
        if(motorChar == 'R') motor = ROLL; else if(motorChar == 'T') motor = TILT; else if(motorChar == 'P') motor = PAN; else if(motorChar == 'B') motor = BOP;
        
	if (absoluteAngle == keeponServos[motor].read()) return; // If this is the same as the last command, exit function
	unsigned int movement_target = constrain(absoluteAngle, 20, 160); // Constrain the movement to the range of the motor, minus a buffer for safety
	unsigned int movement_distance = abs(keeponServos[motor].read() - movement_target); // Find how far the motor needs to move
	keeponServos[motor].write(movement_target, servoSpeed, blockServo); // Send the motor command to the servo
}


void readMotorValues () {
        String state = String();
	for(int nServo = 0; nServo < 4; nServo++)  {
          state+=String(keeponServos[nServo].read()) + ((nServo <3)?':':';'); // default starting location
	}
        int str_len = state.length() + 1; 
        // Prepare the character array (the buffer) 
        char char_array[str_len];
        // Copy it over 
        state.toCharArray(char_array, str_len);
        Serial.write(char_array);
}

/**
 * Reset motor positions to home positions (90 degrees). 
 * This function will abort any current motor action.
 */
void resetMotorPositions () {
	for(int nServo = 0; nServo < 4; nServo++)  {
		// Attach servos to specified pins
		keeponServos[nServo].attach(SERVO_PIN(nServo));

		// Send command to each servo to return home
		keeponServos[nServo].write(90); // default starting location
	}
}
