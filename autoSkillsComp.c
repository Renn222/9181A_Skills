#pragma config(Sensor, in1,    lineLeft,       sensorLineFollower)
#pragma config(Sensor, in2,    lineCenter,     sensorLineFollower)
#pragma config(Sensor, in3,    lineRight,      sensorLineFollower)
#pragma config(Sensor, in4,    lineFront,      sensorLineFollower)
#pragma config(Sensor, in5,    lineBack,       sensorLineFollower)
#pragma config(Sensor, in6,    gyroLeft,       sensorGyro)
#pragma config(Sensor, in7,    gyroRight,      sensorGyro)
#pragma config(Sensor, dgtl1,  encoderMogo,    sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  encoderRight,   sensorQuadEncoder)
#pragma config(Sensor, dgtl5,  encoderLeft,    sensorQuadEncoder)
#pragma config(Motor,  port1,           test,          tmotorVex393_HBridge, openLoop)
#pragma config(Motor,  port2,           frontR,        tmotorVex393TurboSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port3,           midR,          tmotorVex393TurboSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port4,           backR,         tmotorVex393TurboSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port5,           mogoR,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           mogoL,         tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port7,           backL,         tmotorVex393TurboSpeed_MC29, openLoop)
#pragma config(Motor,  port8,           midL,          tmotorVex393TurboSpeed_MC29, openLoop)
#pragma config(Motor,  port9,           frontL,        tmotorVex393TurboSpeed_MC29, openLoop)

// This code is for the VEX cortex platform
#pragma platform(VEX2)

// Select Download method as "competition"
#pragma competitionControl(Competition)

//Main competition background code...do not modify!
#include "Vex_Competition_Includes.c"

int front = 1;
int back	= -1;
int right = 1;
int left = -1;

int MAX_POWER = 121;
int MIN_POWER = 40;
int MAX_ERROR = 50;

int g = 0;
int motorEncoderTest = 0;

float gyroScale = 1;
int mogoTick = 0;
int whiteLine = 0;

/*/////////////////////////////*/
/*			 Reset Functions			 */
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
/*		Basic Drive Functions		 */
/*/////////////////////////////*/


void move(float targetDistance, int maxPower)
{
	float kp = 0.75;
	float ki = 0;
	float kd = 0;

	/* Variables used to calculate how fast bot is moving*/
	float wheelDiameter = 4.5;
	float circumference = wheelDiameter * PI; //Roughly 14.13 inches
	float currentDistance = 0;
	float error = 1;
	float lastError = 0;
	float integral = 0;
	float derivative = 0;
	int power = 0;
	int powerLeft = 0;
	int powerRight = 0;

	resetDriveEncoder();

	while(error != 0)
	{
		currentDistance = SensorValue[encoderRight];

		error = targetDistance - currentDistance;

		integral = (error > MAX_ERROR ) ? 0 : (integral + error);
		derivative = error - lastError;
		lastError = error;

		power = (error * kp) + (integral * ki) + (derivative * kd);

		if(power > 0)
		{
			if(power > maxPower)
			{
				power = maxPower;
			}
			else if(power < MIN_POWER)
			{
				power = MIN_POWER;
			}
		}
		else if(power < 0)
		{
			if(power < -maxPower)
			{
				power = -maxPower;
			}
			else if(power > -MIN_POWER)
			{
				power = -MIN_POWER;
			}
		}
		powerLeft = power;
		powerRight = power;

		motor[frontL] = powerLeft;
		motor[backL] = powerLeft;
		motor[frontR] = powerRight;
		motor[backR] = powerRight;

	}
	resetDrive();
}

int encoderPID(int power, int side)
{
	int kp = 0;

	int master = 0;
	int slave = 0;
	int error = 1;

	master = (SensorValue[encoderRight] >= SensorValue[encoderLeft]) ? SensorValue[encoderRight] : SensorValue[encoderLeft];
	slave = (SensorValue[encoderRight] >= SensorValue[encoderLeft]) ? SensorValue[encoderLeft] : SensorValue[encoderRight];

	error = master - slave;

	if(side == right)
	{
		if(SensorValue[encoderRight] >= SensorValue[encoderLeft])
		{
			power = power - error * kp;
		}
	}
	else if(side == left)
	{
		if(SensorValue[encoderRight] < SensorValue[encoderLeft])
		{
			power = power + error * kp;
		}
	}
	return power;
}

void turn(float bearing)
{
	float kp = 0;
	float ki = 0;
	float kd = 0;

	float integral = 0;
	float derivative = 0;

	float lastError = 0;
	float power = 0;
	int powerLeft = 0;
	int powerRight = 0;

	float error = 1;
	float currentBearing = (SensorValue[gyroLeft] + SensorValue[gyroRight]) / 2 * gyroScale;
	float targetBearing = bearing * 10 + currentBearing;

	while(error != 0)
	{
		error = targetBearing - currentBearing * gyroScale;
		integral = (error > MAX_ERROR / ki) ? 0 : (integral + error);
		derivative = error - lastError;

		lastError = error;

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
		powerLeft = (error >= 0) ? power : -power;
		powerRight = (error >= 0) ? -power : power;

		motor[frontL] = powerLeft;
		motor[backL] = powerLeft;
		motor[frontR] = powerRight;
		motor[backR] = powerRight;
	}
}

void mogo(int choice)
{
	resetMogoEncoder();

	//lower mogo intake
	if(choice == front)
	{
		while(abs(SensorValue[encoderMogo]) <= mogoTick)
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
		while(abs(SensorValue[encoderMogo]) <= mogoTick)
		{
			motor[mogoL] = -121;
			motor[mogoR] = -121;
		}
		motor[mogoL] = -15;
		motor[mogoR] = -15;
	}
}

void lineTurn(int choice)
{
	while(SensorValue[lineFront] != whiteLine && SensorValue[lineCenter] != whiteLine && SensorValue[lineBack] != whiteLine)
	{
		if(SensorValue[lineLeft] == whiteLine)
		{
			turn(3);
		}
		else if(SensorValue[lineRight] == whiteLine)
		{
			turn(-3);
		}
	}
	resetDrive();
}

void pre_auton()
{
	bStopTasksBetweenModes = true;

	resetDriveEncoder();
	resetMogoEncoder();

	SensorType[gyroRight] = sensorNone;
	SensorType[gyroLeft] = sensorNone;
	wait1Msec(500);
	SensorType[gyroRight] = sensorGyro;
	SensorType[gyroLeft] = sensorGyro;
	wait1Msec(1100);
}

task autonomous()
{
	move(902, 121);
	/*mogo(front);
	move(425, 121);
	move(20, 70);

	mogo(back);
	wait1Msec(100);

	move(-360, 121);

	turn(84);//degree of turn
	move(260, 121);

	turn(84);
	move(265, 121);
	constantDrive(50);
	mogo(front);
	wait1Msec(50);
	mogo(back);
	wait1Msec(150);
	constantDrive(0);

	move(-242, 121);

	move(80, 70);
	move(-80, 70);

	turn(-86);
	move(192, 121);

	turn(-93);

	mogo(front);
	move(265, 121);
	move(20, 70);
	mogo(back);
	move(-300, 121);

	turn(168);
	mogo(front);
	move(80, 121);
	move(-80, 121);
	*/
}

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

	if(channel2 >= 101)
	{
		channel2 = 100;
	}
	else if(channel2 <= -101)
	{
		channel2 = -100;
	}
	if(channel3 >= 101)
	{
		channel3 = 100;
	}
	else if(channel3 <= -101)
	{
		channel3 = -100;
	}
}

task usercontrol()
{
	// User control code here, inside the loop
	while (true)
	{
		g = SensorValue[gyroLeft];
		int g2 = SensorValue[gyroRight];
		motorEncoderTest = SensorValue[encoderMogo];

		motor[test] = 100;

		if(vexRT[Btn7U]	 == 1)
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
		deadZoneCheck();

		//drive motors
		motor[frontL] = channel3;
		motor[midL] = channel3;
		motor[backL] = channel3;
		motor[frontR] = channel2;
		motor[midR] = channel2;
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
		if(SensorValue[encoderMogo] == 95)
		{
			motor[mogoL] = 0;
			motor[mogoR] = 0;
		}

		if(SensorValue[encoderMogo] == 0)
		{
			motor[mogoL] = 0;
			motor[mogoR] = 0;
		}
		/*
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
		*/
	}
}
