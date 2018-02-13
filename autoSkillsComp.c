#pragma config(Sensor, in1,    lineLeft,       sensorLineFollower)
#pragma config(Sensor, in2,    lineCenter,     sensorLineFollower)
#pragma config(Sensor, in3,    lineRight,      sensorLineFollower)
#pragma config(Sensor, in4,    gyro,           sensorGyro)
#pragma config(Sensor, dgtl1,  encoderMogo,    sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  encoderRight,   sensorQuadEncoder)
#pragma config(Sensor, dgtl5,  encoderLeft,    sensorQuadEncoder)
#pragma config(Motor,  port2,           backR,         tmotorVex393TurboSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port3,           backL,         tmotorVex393TurboSpeed_MC29, openLoop)
#pragma config(Motor,  port4,           tipR,          tmotorVex393HighSpeed_MC29, openLoop)
#pragma config(Motor,  port5,           mogoR,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           mogoL,         tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port7,           tipL,          tmotorVex393HighSpeed_MC29, openLoop)
#pragma config(Motor,  port8,           frontR,        tmotorVex393TurboSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           frontL,        tmotorVex393TurboSpeed_MC29, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*        Description: Competition template for VEX EDR                      */
/*                                                                           */
/*---------------------------------------------------------------------------*/

// This code is for the VEX cortex platform
#pragma platform(VEX2)

// Select Download method as "competition"
#pragma competitionControl(Competition)

//Main competition background code...do not modify!
#include "Vex_Competition_Includes.c"

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the cortex has been powered on and    */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

int front = 1;
int back  = -1;
/*
float kp = 5;
float ki = 5;
float kd = 5;
*/
int MAX_POWER = 121;
int MIN_POWER = 40;
int MAX_ERROR = 50;

int g = 0;
int motorEncoderTest = 0;

/*/////////////////////////////*/
/*       Reset Functions       */
/*/////////////////////////////*/

void resetDrive()
{
	motor[frontL] = 0;
	motor[backL] = 0;
	motor[frontR] = 0;
	motor[backR] = 0;
}

void constantDrive(int spdC)
{
	motor[frontL] = spdC;
	motor[backL] = spdC;
	motor[frontR] = spdC;
	motor[backR] = spdC;
}

void resetDriveEncoder()
{
	SensorValue[encoderLeft] = 0;
	SensorValue[encoderRight] = 0;
}

void resetMogoEncoder()
{
	SensorValue[encoderMogo] = 0;
}

/*/////////////////////////////*/
/*    Basic Drive Functions    */
/*/////////////////////////////*/

void move(float distanceInInches)//distance controls direction not spd
{
	float spd = 121;
	float leftSpd = spd;
	float rightSpd = spd;
	float inverter = 1.0;
	if(distanceInInches < 0.0)
	{
		inverter *= -1.0;
	}
	float cir = 4.0*PI; // 12.56
	float degree = (( distanceInInches / cir ) * 360.0) * 0.5714;

	SensorValue[encoderRight] = 0;
	SensorValue[encoderLeft] = 0;
	while(abs(degree) > abs(SensorValue[encoderRight]) && abs(degree) > abs(SensorValue[encoderLeft]))
	{
		rightSpd = spd;
		leftSpd = spd;
		//uhhhhh idk if this works but whatever
		if(leftSpd > 125.0){
			leftSpd = 125.0;
		}
		else if(leftSpd < -125.0)
		{
			leftSpd = -125.0;
		}
		if(rightSpd > 125.0){
			rightSpd = 125.0;
		}
		else if(rightSpd < -125.0)
		{
			rightSpd = -125.0;
		}
		motor[backL] = leftSpd * inverter;
		motor[backR] = rightSpd * inverter;
		motor[frontL] = leftSpd * inverter;
		motor[frontR] = rightSpd * inverter;
	}
	motor[backL] = -(leftSpd * inverter) / 3;
	motor[backR] = -(rightSpd * inverter) / 3;
	motor[frontL] = -(leftSpd * inverter) / 3;
	motor[frontR] = -(rightSpd * inverter) / 3;
	wait1Msec(150);
	motor[backL] = 0;
	motor[backR] = 0;
	motor[frontR] = 0;
	motor[frontL] = 0;
}
/*
void move(float targetDistance)
{
// Variables used to calculate how fast bot is moving
float wheelDiameter = 4.0;
float circumference = wheelDiameter * PI; //Roughly 14.13 inches
float currentDistance = 0;
float error = -10;
float lastError = 0;
float integral = 0;
float derivative = 0;
float power = 0;
resetDriveEncoder();
while(error < 5 && error > -5)
{
currentDistance = (float)SensorValue[encoderRight] / 360.0 * circumference;
error = targetDistance - currentDistance;
integral = (error > MAX_ERROR) ? 0 : (integral + error);
derivative = error - lastError;
power = (error * kp) + (integral * ki) + (derivative * kd);
if(power > 0)
{
if(power > MAX_POWER)
{
power = MAX_POWER;
}
else if(power < MIN_POWER)
{
power = MIN_POWER;
}
}
else if(power < 0)
{
if(power < -MAX_POWER)
{
power = -MAX_POWER;
}
else if(power > -MIN_POWER)
{
power = -MIN_POWER;
}
}
motor[frontL] = power;
motor[backL] = power;
motor[frontR] = power;
motor[backR] = power;
lastError = error;
}
resetDrive();
}*/
void turn(float bearing)//assume positive is right and negative is left
{
	//bearing*=(4/5);
	bearing = (bearing * 9) / 10;//6.755
	float currentBearing = SensorValue[gyro]/10;
	float targetBearing = bearing + currentBearing;
	float TURN_SLOWDOWN = 10;

	//float gyr = SensorValue[Gyro]/10;


	//MAKE TURNING ACCURATE BY MAKING REVERSE FOR 80ms


	if (targetBearing < SensorValue[gyro]/10) // Left turn
	{

		if(targetBearing > 0 && targetBearing > 360)
		{
		targetBearing = targetBearing - 360;
		}
		else if(targetBearing < 0 && targetBearing < -360)
		{
		targetBearing = targetBearing + 360;
		}

		while (SensorValue[gyro]/10 > targetBearing + TURN_SLOWDOWN)
		{
			motor[frontL] = -100;
			motor[backL] = -100;
			motor[frontR] = 100;
			motor[backR] = 100;
			//gyr = SensorValue[Gyro]/10;
		}
		while (SensorValue[gyro]/10 > targetBearing)
		{
			motor[frontL] = -50;
			motor[backL] = -50;
			motor[frontR] = 50;
			motor[backR] = 50;
			//gyr = SensorValue[Gyro]/10;
		}

		motor[frontL] = 40;
		motor[backL] = 40;
		motor[frontR] = -40;
		motor[backR] = -40;
		wait1Msec(200);

		resetDrive();
	}
	else // Right turn
	{

		if(targetBearing > 0 && targetBearing > 360)
		{
		targetBearing = targetBearing - 360;
		}
		else if(targetBearing < 0 && targetBearing < -360)
		{
		targetBearing = targetBearing + 360;
		}

		while (SensorValue[gyro]/10 < targetBearing - TURN_SLOWDOWN)
		{
			motor[frontL] = 100;
			motor[backL] = 100;
			motor[frontR] = -100;
			motor[backR] = -100;
			//gyr = SensorValue[Gyro]/10;
		}
		while (SensorValue[gyro]/10 < targetBearing)
		{
			motor[frontL] = 50;
			motor[backL] = 50;
			motor[frontR] = -50;
			motor[backR] = -50;
			//gyr = SensorValue[gyro]/10;
		}
		motor[frontL] = -40;
		motor[backR] = -40;
		motor[frontR] = 40;
		motor[backR] = 40;

		wait1Msec(200);

		resetDrive();
	}
}


void mogo(int choice)
{
	resetMogoEncoder();

	//lower mogo intake
	if(choice == front)
	{
		while(abs(SensorValue[encoderMogo]) <= 95)
		{
			motor[mogoL] = 121;
			motor[mogoR] = 121;
		}
		motor[mogoL] = 15;
		motor[mogoR] = 15;
	}
	//raise mogo intake
	else if(choice == back)
	{
		while(abs(SensorValue[encoderMogo]) <= 95)
		{
			motor[mogoL] = -121;
			motor[mogoR] = -121;
		}
		motor[mogoL] = -15;
		motor[mogoR] = -15;
	}
}



/*/////////////////////////////*/
/*    Placing in Point Zones   */
/*/////////////////////////////*/



void pre_auton()
{
	// Set bStopTasksBetweenModes to false if you want to keep user created tasks
	// running between Autonomous and Driver controlled modes. You will need to
	// manage all user created tasks if set to false.
	bStopTasksBetweenModes = true;

	resetDriveEncoder();
	resetMogoEncoder();

	SensorType[gyro] = sensorNone;
	wait1Msec(500);
	SensorType[gyro] = sensorGyro;
	wait1Msec(1100);

	// Set bDisplayCompetitionStatusOnLcd to false if you don't want the LCD
	// used by the competition include file, for example, you might want
	// to display your team name on the LCD in this function.
	// bDisplayCompetitionStatusOnLcd = false;

	// All activities that occur before the competition starts
	// Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

task autonomous()
{
	// ..........................................................................
	// Insert user code here.
	// ..........................................................................

	// Remove this function call once you have "real" code.
	//AutonomousCodePlaceholderForTesting();

	mogo(front);
	move(53);
	mogo(back);
	wait1Msec(200);
	move(-46);
	turn(-45);//degree of turn
	move(-24);
	turn(-90);
	wait1Msec(200);
	constantDrive(120);
	wait1Msec(700);
	constantDrive(30);
	mogo(front);
	constantDrive(-50);
	wait1Msec(300);
	mogo(back);
	move(-11);
	/*constantDrive(121);//positive is forward
	wait1Msec(1700);
	constantDrive(40);
	mogo(front);
	wait1Msec(200);
	constantDrive(-50);
	wait1Msec(500);
	constantDrive(0);
	mogo(back);*/ //score 20 points
	//total 20

	turn(-90);//wall crash
	move(-40);//wall                       maybe make this constantDrive so it doesn't stop********************(maybe)
	constantDrive(-60);
	wait1Msec(1000);
	resetDriveEncoder(); //wall sensor reset
	mogo(front);
	move(43);
	mogo(back);
	turn(135);
	move(47);
	mogo(front);
	move(-10);
	mogo(back);//score 10 points
	//total 30

	turn(-90);
	move(21);
	turn(-90);
	mogo(front);
	move(34);
	mogo(back);
	wait1Msec(300);
	turn(-180);
	move(40);
	mogo(front);
	move(-10);//score 10 points
	mogo(back);
	//total 40

	turn(-180);
	mogo(front);
	move(70);
	mogo(back);
	move(30);
	turn(-90);
	move(18);
	turn(90);
	constantDrive(121);
	wait1Msec(1700);
	resetDrive();
	mogo(front);
	constantDrive(-35);
	wait1Msec(450);
	resetDrive();
	mogo(back);//score 20 points
	move(-33);
	//total 60
	/*
	turnToBearing(-90);
	driveAlto(8, 110);
	turnToBearing(-90);
	SensorValue[QEBase] = 0;
	motor[baseLiftL] = baseSpd;
	motor[baseLiftR] = baseSpd;
	driveAlto(15, 100);//(distance, spd)
	reset = false;
	pickUpBase();
	turnToBearing(180);
	driveAlto(15, 100);
	putDownBase();
	//total 72
	//If there is time then move on to this code \/
	turnToBearing(-90);
	driveAlto(16, 120);//crash into wall here
	driveAlto(-3, 100);//needs to be fine tuned for the wall
	turnToBearing(-45);
	SensorValue[QEBase] = 0;
	motor[baseLiftL] = baseSpd;
	motor[baseLiftR] = baseSpd;
	driveAlto(28, 100);//(distance, spd)
	reset = false;
	pickUpBase();
	wait1Msec(200);
	driveAlto(-23, 110);
	turnToBearing(-45);//degree of turn
	driveAlto(-10, 110);
	turnToBearing(-90);
	driveAlto(10, 125);
	putDownBase();//score 10 points
	//total 82
	*/











	/*driveAlto(60, 121);
	turn(-90);
	*/

	//RED LEFT
	/*
	mogo(front);
	driveAlto(62, 121);
	wait1Msec(200);
	mogo(back);
	wait1Msec(200);
	driveAlto(-48, 121);
	turn(-45);
	driveAlto(-22, 121);
	turn(-90);
	driveAlto(30, 121);
	wait1Msec(200);
	motor[frontL] = 50;
	motor[backR] = 50;
	motor[frontR] = 50;
	motor[backR] = 50;
	mogo(front);
	wait1Msec(200);
	driveAlto(-5, 121);
	mogo(back);
	wait1Msec(200);
	driveAlto(-25, 121);
	*/



	//BLUE RIGHT
	/*
	mogo(front);
	driveAlto(62, 121);
	mogo(back);
	wait1Msec(200);
	driveAlto(-48, 121);
	turn(45);
	driveAlto(-22, 121);
	turn(90);
	driveAlto(30, 121);
	wait1Msec(200);
	motor[frontL] = 50;
	motor[backR] = 50;
	motor[frontR] = 50;
	motor[backR] = 50;
	mogo(front);
	wait1Msec(200);
	driveAlto(-5, 121);
	mogo(back);
	wait1Msec(200);
	driveAlto(-25, 121);
	*/
	/*
	turn(90);
	driveAlto(-50,121);
	mogo(front);
	wait1Msec(200);
	driveAlto(32, 121);
	turn(-135);
	driveAlto(45, 121);
	mogo(front);
	driveAlto(-9, 121);
	turn(90);
	*/
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/


int threshold = 8;

int channel1;
int channel2;
int channel3;
int channel4;

void deadZoneCheck()
{
	channel1 = (abs(vexRT[Ch1]) > threshold) ? vexRT[Ch1] : 0;
	channel2 = (abs(vexRT[Ch2]) > threshold) ? vexRT[Ch2] : 0;
	channel3 = (abs(vexRT[Ch3]) > threshold) ? vexRT[Ch3] : 0;
	channel4 = (abs(vexRT[Ch4]) > threshold) ? vexRT[Ch4] : 0;

	if(channel2 >= 121)
	{
		channel2 = 120;
	}
	else if(channel2 <= -121)
	{
		channel2 = -120;
	}
	if(channel3 >= 121)
	{
		channel3 = 120;
	}
	else if(channel3 <= -121)
	{
		channel3 = -120;
	}
	/*if (abs(vexRT[Ch1]) > threshold)
	channel1 = vexRT[Ch1];
	else
	channel1 = 0;
	if (abs(vexRT[Ch2]) > threshold)
	channel2 = vexRT[Ch2];
	else
	channel2 = 0;
	if(abs(vexRT[Ch3]) > threshold)
	channel3 = vexRT[Ch3];
	else
	channel3 = 0;
	if(abs(vexRT[Ch4]) > threshold)
	channel4 = vexRT[Ch4];
	else
	channel4 = 0;*/
}



task usercontrol()
{
	// User control code here, inside the loop
	while (true)
	{
		g = SensorValue[gyro];
		motorEncoderTest = SensorValue[encoderMogo];

		if(vexRT[Btn7U]  == 1)
		{
			motor[backL] = motorType[NULL];
			motor[backR] = motorType[NULL];
			motor[frontL] = motorType[NULL];
			motor[frontR] = motorType[NULL];
			wait1Msec(200);
			motor[backL] = motorType[tmotorVex393TurboSpeed_MC29];
			motor[backR] = motorType[tmotorVex393TurboSpeed_MC29];
			motor[frontL] = motorType[tmotorVex393TurboSpeed_MC29];
			motor[frontR] = motorType[tmotorVex393TurboSpeed_MC29];
			wait1Msec(3000);
			resetDrive();
		}

		// This is the main execution loop for the user control program.
		// Each time through the loop your program should update motor + servo
		// values based on feedback from the joysticks.

		// ........................................................................
		// Insert user code here. This is where you use the joystick values to
		// update your motors, etc.
		// ........................................................................

		// Remove this function call once you have "real" code.
		//UserControlCodePlaceholderForTesting();

		deadZoneCheck();

		//drive motors
		motor[frontL] = channel3;
		motor[backL] = channel3;
		motor[frontR] = channel2;
		motor[backR] = channel2;

		//mobile lift goes up on button 6U
		if (vexRT[Btn6U] == 1)
		{
			motor[mogoR] = -100;
			motor[mogoL] = -100;

		}
		//mobile lift goes down on button 6D
		else if (vexRT[Btn6D] == 1)
		{
			motor[mogoL] = 100;
			motor[mogoR] = 100;
		}
		else
		{
			motor[mogoL] = 0;
			motor[mogoR] = 0;
		}

		if(vexRT[Btn8D] == 1)
		{
			motor[tipL] = -23;
			motor[tipR] = -23;
		}


		if (vexRT[Btn5U] == 1)
		{
			motor[tipL] = -80;
			motor[tipR] = -80;

		}
		//tipper goes down on button 5D
		else if (vexRT[Btn5D] == 1)
		{
			motor[tipL] = 80;
			motor[tipR] = 80;
		}
		else
		{
			motor[tipL] = 0;
			motor[tipR] = 0;
		}
	}
}
