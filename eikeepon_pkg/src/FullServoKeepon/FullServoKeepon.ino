/*
	MuseumKeepon controller: v 2.0. Uses servos for all motors.
	-----------------------------------------------------------
	Slave module to the ArduinoController python module

	Notes:
	------
	This module has an optional secondary slave arduino on a software serial channel
	that prints debug messages to the screen. The museumKeepon arduino's main hardware
	serial channel is busy with the ArduinoRemoteController.


	Ahsan Nawroj
	Updated: 05/09/2014
*/

#include <Arduino.h>
#include <string.h>
#include <SoftwareServo.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <SoftwareSerial.h>

//#include "museumKeepon.h"

// -------------------------------------------------------------------------------
//
//  PRE-PROCESSOR DIRECTIVES, DEFINITIONS
//
// --------------------------------------------------------------------------------
#define SERVO_PIN(x) (x+2)	// all servo pins are connected from 2 onwards
#define ROLL 0
#define BOP 1
#define TILT 2
#define PAN 3
#define SINGLE_BOP_DELAY 300 // --CALIBRATE--  changed previous 500
#define BOP_OFF_SPEED 90 // --CALIBRATE-- speed value for the servo
#define DEFAULT_BOP_SPEED 45 // --CALIBRATE-- 45

#define SERVO_CENTER 90
#define TILT_MIN  20
#define TILT_MAX  160
#define ROLL_MIN  30
#define ROLL_MAX  160

//#define ANIM_RATE 1 //see if it is being used

#define IDLE_FRONT 0
#define IDLE_LEFT 1
#define IDLE_RIGHT 2
#define IDLE_UP 3
#define IDLE_DOWN 4
#define IDLE_BOP 5
#define IDLE_MAX_STATES 6
#define IDLE_RATE 4000 // nr. of milis that idle behaviors are called

#define ANIM_NONE 0
#define ANIM_IDLE 1
#define ANIM_SPEAK 2
#define ANIM_BOUNCE 3
#define ANIM_SAD 4
#define ANIM_HAPPY 5


// --------------------------------------------------------------------------------
//
//  G L O B A L S
//
// --------------------------------------------------------------------------------
SoftwareServo keeponServos[4];		// 0 = roll, 1 = bop, 2 = tilt, 3 = pan
String commandString;				// input string from Python master
bool commandsAreRelative = true;	// OPTIONAL: relative vs. absolute commands
bool newCommandReceived = false;	// flag for command received

int panCommand = 90, rollCommand = 90,
	bopCommand = 0, tiltCommand = 90;		// global containers for inputs
int servoAngles[4];					// OPTIONAL: grouped container for inputs

unsigned long bopRunTime = 0;
unsigned long bopStartTime = millis();

SoftwareSerial debugSerial(10, 11); // RX, TX

// for animation and idle behaviors
//bool idleModeON = false;
//bool speakON = false;
//bool animationON = false;
int currentAnimation = ANIM_NONE;

int targetTilt = 90;
int targetRoll = 90;

int lastAnimTime = 0;
int lastIdleTime = 0;
int rollDirection = 1;

// --------------------------------------------------------------------------------
//
//  M A I N   D R I V E R   F U N C T I O N S
//
// --------------------------------------------------------------------------------
void setup () {

	Serial.begin(9600); // main hardware serial for ArduinoRemoteController
	debugSerial.begin(4800); // software serial for debug messages

	// INITIALIZE SERVOS
	// -----------------
	for(int nServo = 0;nServo < 4;nServo++)  {
		keeponServos[nServo].attach(SERVO_PIN(nServo));
		keeponServos[nServo].write(90); // default starting location
		delay_sft(10);
	}
	debugSerial.println("Initialized servos");
	debugSerial.println("Completed setup");

	lastAnimTime = millis();
	lastIdleTime = millis();
	//idleModeON = true;
	//testAllSweeps();
	//speakON = true;
	//targetRoll = ROLL_MAX;
}


void loop () {

	//check if bop is over 
	unsigned long currentTime = millis();
	if (bopCommand > 0 && (currentTime-bopStartTime) > bopRunTime) {
		//debugSerial.print("keep boping?\n");
		keeponServos[BOP].write(BOP_OFF_SPEED);
		delay_sft(1);
		bopRunTime = 0;
		bopCommand = 0;
	}

	receiveCommands();
	if (newCommandReceived) {
		
		debugSerial.print("Current position: ");
		debugSerial.print("P: "); debugSerial.print(keeponServos[PAN].read());
		debugSerial.print(", T: "); debugSerial.print(keeponServos[TILT].read());
		debugSerial.print(", R: "); debugSerial.print(keeponServos[ROLL].read());
		debugSerial.print(", B: "); debugSerial.println(keeponServos[BOP].read());
	
		debugSerial.print("New command: ");
		debugSerial.print("P: "); debugSerial.print(panCommand);
		debugSerial.print(", T: "); debugSerial.print(tiltCommand);
		debugSerial.print(", R: "); debugSerial.print(rollCommand);
		debugSerial.print(", B: "); debugSerial.println(bopCommand);
		
		panTo(panCommand); 
		rollTo(rollCommand); delay_sft(1);
		tiltTo(tiltCommand); delay_sft(1);
		//bopStartTime = millis();
		bopCount(bopCommand); delay_sft(1);	// this is a count of bops to perform	
	}

	switch (currentAnimation) {
		case ANIM_IDLE:
			performIdleStep();
			break;
		case ANIM_SPEAK:
			//targetRoll = ROLL_MAX; #don't forget to set targetRoll in command parsing!!
			performSpeakStep();
			break;
		case ANIM_BOUNCE:
			performBounceStep();
			break;
	}

	if (keeponServos[ROLL].read() != rollCommand) { //only send pan if there is a new value
		rollTo(rollCommand);
		delay_sft(1);
	}
	if (keeponServos[TILT].read() != tiltCommand) {
		tiltTo(tiltCommand); 
		delay_sft(1);
	}
	bopCount(bopCommand);	// this is a count of bops to perform	
	lastAnimTime = currentTime;

	//if (currentTime - lastAnimTime > ANIM_RATE) {
		//debugSerial.print("running anim \n");
		// if (idleModeON) {
		// 	performIdleStep();

		// } else if (speakON) {
		// 	performSpeakStep();
		// 	performBounceStep();

		// } else if (animationON) {
		// 	performBounceStep();
		// 	//bouncy bounce or other animations
		// }

		//panTo(panCommand); 

	//}	

}

// --------------------------------------------------------------------------------
//
//  I D L E   B E H A V I O R   F U N C T I O N S
//
// --------------------------------------------------------------------------------

void performIdleStep () {
	int currentTime = millis();
	int currentTilt = keeponServos[TILT].read();
	int currentRoll = keeponServos[ROLL].read();

	if (currentTime - lastIdleTime > IDLE_RATE) {
		//select new idle behavior
		selectNewIdleBehavior();
		lastIdleTime = currentTime;
	} else {
		//get to the desired position or do nothing
		if (currentTilt != targetTilt) {
			//gradually go to target tilt position
			if (targetRoll > currentRoll)
				rollCommand++;
			else
				rollCommand--;
		}
		if (currentRoll != targetRoll) {
			//gradually go to target roll position
			if (targetRoll > currentTilt)
				rollCommand++;
			else
				rollCommand--;
		}
	}
}

//randomly selects new idle position and sets target tilt, roll andd bop values
void selectNewIdleBehavior() {
	//debugSerial.print("** new idle behavior\n");
	int newIdleBehavior = random(IDLE_MAX_STATES);

	switch (newIdleBehavior){
		case IDLE_FRONT:
			debugSerial.print("IDLE_FRONT\n");
			keeponServos[BOP].write(BOP_OFF_SPEED);
			rollCommand = random (SERVO_CENTER-10, SERVO_CENTER+10);
			tiltCommand = random (SERVO_CENTER-10, SERVO_CENTER+10);
			break;
		case IDLE_DOWN:
			keeponServos[BOP].write(BOP_OFF_SPEED);
			debugSerial.print("IDLE_DOWN\n");
			rollCommand = SERVO_CENTER;
			tiltCommand = TILT_MIN;
			break;
		case IDLE_UP:
			keeponServos[BOP].write(BOP_OFF_SPEED);
			debugSerial.print("IDLE_UP\n");
			tiltCommand = TILT_MAX;
			rollCommand = SERVO_CENTER;
			break;
		case IDLE_RIGHT:
			keeponServos[BOP].write(BOP_OFF_SPEED);
			debugSerial.print("IDLE_RIGHT\n");
			rollCommand = ROLL_MAX;
			tiltCommand = SERVO_CENTER;
			break;
		case IDLE_LEFT:
			keeponServos[BOP].write(BOP_OFF_SPEED);
			debugSerial.print("IDLE_LEFT\n");
			rollCommand = ROLL_MIN;
			tiltCommand = SERVO_CENTER;
			break;
		case IDLE_BOP:
			bopCommand = random(1);   //(1,3);
			//debugSerial.print("IDLE_BOP\n");
			//debugSerial.print(bopCommand);
			bopStartTime = millis();
			newCommandReceived = true; 
			break;
		}
}

// --------------------------------------------------------------------------------
//
//  A N I M A T I O N   F U N C T I O N S
//
// --------------------------------------------------------------------------------

void performBounceStep () {

	int currentServoPos = keeponServos[ROLL].read();

	int randNumber = random(4);
	if (randNumber < 2) {
		bopCommand = random(2); //a few bops
		bopStartTime = millis();
	} else {

		//switch position once it reaches a limit
		if (targetRoll == currentServoPos) {
			rollDirection = rollDirection * -1;
			if (rollDirection > 0)
				targetRoll = random (ROLL_MAX-50, ROLL_MAX);
			else
				targetRoll = random (ROLL_MIN, ROLL_MIN+50);
		debugSerial.print(targetRoll);
		} else {
			//gradually go to target position
			if (targetRoll > currentServoPos)
				rollCommand = rollCommand + 1;
			else
				rollCommand = rollCommand - 1;
		}

		// if (targetRoll == currentServoPos) {
		// 	if (currentServoPos >= ROLL_MAX)
		// 		targetRoll = ROLL_MIN;
		// 	else
		// 		targetRoll = ROLL_MAX;
		// } else {
		// 	//gradually go to target position
		// 	if (targetRoll > currentServoPos)
		// 		rollCommand = rollCommand + 1;
		// 	else
		// 		rollCommand = rollCommand - 1;
		// }
	}	

}

void performSpeakStep () {

	keeponServos[BOP].write(BOP_OFF_SPEED);
	delay_sft(0.5);
	int currentServoPos = keeponServos[ROLL].read();

	int randNumber = random(20);
	if (randNumber > 18) {
		bopCommand = random(1); //a few bops
		bopStartTime = millis();
	} else {

		//switch direction once it reaches a limit
		if (targetRoll == currentServoPos) {
			rollDirection = rollDirection * -1;
			if (rollDirection > 0)
				targetRoll = random (ROLL_MAX-50, ROLL_MAX);
			else
				targetRoll = random (ROLL_MIN, ROLL_MIN+50);
			debugSerial.print(targetRoll);
		} else {
			//gradually go to target position
			if (targetRoll > currentServoPos)
				rollCommand = rollCommand + 1;
			else
				rollCommand = rollCommand - 1;
		}
	}	
}

// --------------------------------------------------------------------------------
//
//  U T I L I T Y   F U N C T I O N S
//
// --------------------------------------------------------------------------------

// Wrapper function for delays in this sketch
// Delays need to be chopped up to accomodate regular calls to SoftwareServo::refresh()
void delay_sft (long t) {
	if (t < 20) {delay(t); SoftwareServo::refresh(); }
	long i = t / 15;
	for (int j=0; j < i; j++) { delay(15); 	SoftwareServo::refresh(); }
	delay(t % 15); SoftwareServo::refresh();
}

// ------------------------------------------------------------------------
// Command parsing from serial stream
// ------------------------------------------------------------------------
// 	Expected string input:
// 		GGpXrYtZbT
// 		GG = "ab" (absolute), "re" (relative)
// 		pX = pan value X
// 		rY = roll value Y
// 		tZ = tilt value Z
// 		bT = bop value T (count of how many times to bop)
//
// 	Absolute positions 0-180
// 	Relative pan positions 90-180 for positive 0 to 90, 90-0 for negative 0 to 90
// 		sending 90 does no position change

void receiveCommands() {
  newCommandReceived = false;
  if (Serial.available() <= 0) {
	  //debugSerial.print("waiting for serial messages\n");
	  return; // leave right away
  }

	  char newByte;
	  commandString = "";	// forget last command string
	 while (Serial.available() > 0) {	// read new command in
	    newByte = (char)Serial.read();
	    commandString += newByte;
		delay_sft(2);  //changed May 26 - era 5
		if (newByte == '\0') break;
	}
	
	debugSerial.println ("received: " + commandString); // return read command 

	//new code for parsing animation commands 
	if (commandString[0] == 'a') {
		if (commandString[1] == 'i') {
			debugSerial.println("**** RECEIVED COMMAND IDLE ***");
			currentAnimation = ANIM_IDLE;
		} else if (commandString[1] == 's') {
			debugSerial.println("**** RECEIVED COMMAND SPEAK ***");
			currentAnimation = ANIM_SPEAK;
			bopCommand = 0;
			//bopCommand = 0;
			//rollTo(90);
			//rollTo(90);
			//delay_sft(5);
		} else if (commandString[1] == 'b'){
			debugSerial.println("**** RECEIVED COMMAND BOUNCY BOUNCE ***");
			currentAnimation = ANIM_BOUNCE;
		}

	}

	//end of new code for parsing animation commands 

	if (commandString == "" || commandString.length() < 8 )
		return; // bad command

	// update request or new instruction
	if (commandString[0] == 'u' && commandString[1] == 'u') {
		debugSerial.println("Update request received");
		debugSerial.print ("Sending: ");
		debugSerial.print("p");			debugSerial.print(keeponServos[PAN].read());
		debugSerial.print(",r");		debugSerial.print(keeponServos[ROLL].read());
		debugSerial.print(",t");		debugSerial.print(keeponServos[TILT].read());
		debugSerial.print(",b");		debugSerial.println(keeponServos[BOP].read());


		Serial.print("p");		Serial.print(keeponServos[PAN].read());
		Serial.print(",r");		Serial.print(keeponServos[ROLL].read());
		Serial.print(",t");		Serial.print(keeponServos[TILT].read());
		Serial.print(",b");		Serial.println(keeponServos[BOP].read());
		return;
	} 

	// relative vs. absolute commands
	//if (commandString[0] == 'a' && commandString[1] == 'b') commandsAreRelative = false;
	//else if (commandString[0] == 'r' && commandString[1] == 'e') commandsAreRelative = true;

        Serial.println ("received: " + commandString); // return read command	

        int i = 0;
        while (commandString[i] != '\0') {
			  //debugSerial.print("waiting for command parsing");
                if ( commandString[i] == 'p') {
                    //Serial.println(1);
                    int j = i + 1;
                    while ( (commandString[j] != 't') || (commandString[j] > '0' && commandString[j] < '9')) {
						debugSerial.print("reading pan");
                        j++;
                    }
                    panCommand = (commandString.substring(i+1,j)).toInt();
                    //Serial.println(panCommand);
                }
                else if ( commandString[i] == 't' ) {
                    int j = i + 1;
                    while ( commandString[j] != 'r' || (commandString[j] > '0' && commandString[j] < '9')) {
						debugSerial.print("reading tilt");
                        j++;
                    }
                    tiltCommand = (commandString.substring(i+1,j)).toInt();
                } 
                else if ( commandString[i] == 'r' ) {
                    int j = i + 1;
                    while ( commandString[j] != 'b' || (commandString[j] > '0' && commandString[j] < '9')) {
                        j++;
                    }
                    rollCommand = (commandString.substring(i+1,j)).toInt();
                } 
                else if ( commandString[i] == 'b' ) {
                    int j = i + 1;
                    while ( commandString[j] != '\0' || (commandString[j] > '0' && commandString[j] < '9')) {
                        j++;
                    }
                    bopCommand = (commandString.substring(i+1,j)).toInt();
                    //Serial.println(tmp);
                }
            i++;
        } newCommandReceived = true; 
}

// ------------------------------------------------------
// Pan command angle between -90 (all the way left) and 90 (all the way right)
// ------------------------------------------------------
// relative pan
void panBy (int relativeAngle) {
	keeponServos[PAN].write(constrain(keeponServos[PAN].read()+ relativeAngle, 20, 160));
}
// absolute pan
void panTo (int absoluteAngle) {
	keeponServos[PAN].write(constrain(absoluteAngle, 20, 160));
}

// ------------------------------------------------------
// Tilt command angle between -90 (all the way back) and 90 (all the way forward)
// ------------------------------------------------------
// relative tilt
void tiltBy (int relativeAngle) {
	keeponServos[TILT].write(constrain(keeponServos[TILT].read() + relativeAngle, 20, 160));
}
// absolute tilt
void tiltTo (int absoluteAngle) {
	keeponServos[TILT].write(constrain(absoluteAngle, 20, 160));
}

// ------------------------------------------------------
// Roll command angle is relative position from current.
// ------------------------------------------------------
// relative roll
void rollBy (int relativeAngle) {
	keeponServos[ROLL].write(constrain(keeponServos[ROLL].read() + relativeAngle, 20, 160));
}
// absolute roll
void rollTo (int absoluteAngle) {
	keeponServos[ROLL].write(constrain(absoluteAngle, 20, 160));
}


// ------------------------------------------------------
// Bop command gives number of bops to make.
// ------------------------------------------------------
void bopCount (int count) {
	if (count == 0) return;

	keeponServos[BOP].write(DEFAULT_BOP_SPEED);
	delay_sft(1);
	long necessaryDelay = SINGLE_BOP_DELAY*count;	// in milliseconds
	bopRunTime = necessaryDelay;
}



// ------------------------------------------------------
// Freeze all motors
// ------------------------------------------------------
void resetMotorPositions () {
	for(int nServo = 0;nServo < 4;nServo++)  {
		keeponServos[nServo].attach(SERVO_PIN(nServo));
		servoAngles[nServo] = 90; // default starting location
		delay_sft(1);
	}
}



// ------------------------------------------------------
//	Update motor locations
// ------------------------------------------------------
void motorStatusUp () {
	for(int nServo = 0;nServo < 4;nServo++)  {
		keeponServos[nServo].write(servoAngles[nServo]);
	}
}

void printServoAngles() {
	for (int i=0;i<3;i++) {
		debugSerial.print(servoAngles[i]);
		debugSerial.print(", "); }
	debugSerial.print(servoAngles[3]); debugSerial.println("");
}


// --------------------------------------------------------------------------------
//
//  T E S T I N G   F U N C T I O N S
//
// --------------------------------------------------------------------------------
// read external pots, set servos to pot inputs
void demo1_potToServos () {
	pinMode(A0, INPUT);	pinMode(A1, INPUT); pinMode(A2, INPUT); pinMode(A3, INPUT);
	int potPins[4] = {A0, A1, A2, A3};
	int sensitivity[4] = {10, 10, 20, 20};
	while (1) {
		for (int i=0; i < 4; i++) {
			int j = analogRead(potPins[i]);
			j=(int) constrain(j*sensitivity[i] - 512*(sensitivity[i]-1), 0, 1023);
			servoAngles[i] = map(j, 0, 1023, 0, 180);
		}
		printServoAngles(); motorStatusUp(); delay_sft(10);
	}
}

// sweep tilt and roll back and forth, bop once every two seconds
void demo2_bopWithTiltRoll(){
	int testDelay = 1;
	for (int i = 20; i < 160; i++) { tiltTo(i); delay_sft(testDelay); }
	bopCount(3);
	for (int i = 160; i > 20; i--) { tiltTo(i); delay_sft(testDelay); }
	delay_sft(3000);

	for (int i = 20; i < 160; i++) { rollTo(i); delay_sft(testDelay); }
	bopCount(3);
	for (int i = 160; i > 20; i--) { rollTo(i); delay_sft(testDelay); }
	delay_sft(3000);
}

// simple bop test
// useful to calibrate bop timings
void demo3_pureBop () {
	bopCount(3); delay_sft(5000); debugSerial.println("Bopped!");
}

// sweep through 0 to 180 for each servo
void testAllSweeps() {
	debugSerial.println("in test all sweeps!");
	while(1) {
		int testDelay = 1;
		for (int i = 20; i < 160; i=i+2) {
			//keeponServos[ROLL].write(i); delay_sft(testDelay);
			rollTo(i); delay_sft(testDelay);
		}	
		delay_sft(5000);
		for (int i = 160; i > 20; i--) {
			//keeponServos[ROLL].write(i); delay_sft(testDelay);
			rollTo(i); delay_sft(testDelay);
		}
		delay_sft(5000);
		
	}
}



