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

// This code is for the VEX cortex platform
#pragma platform(VEX2)

// Select Download method as "competition"
#pragma competitionControl(Competition)

//Main competition background code...do not modify!
#include "Vex_Competition_Includes.c"

int front = 1;
int back  = -1;

float kp = 5;
float ki = 5;
float kd = 5;

int MAX_POWER = 100;
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

void move(float targetDistance, int power)
{
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

	while(error > 0)
	{
		currentDistance = SensorValue[encoderRight] / 360 * circumference;

		error = targetDistance - currentDistance;

		integral = (error > MAX_ERROR / ki) ? 0 : (integral + error);
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
		powerLeft = encoderPID(power, left);
		powerRight = encoderPID(power, right);
		
		motor[frontL] = powerLeft;
		motor[backL] = powerLeft;
		motor[frontR] = powerRight;
		motor[backR] = powerRight;

		lastError = error;
	}
	resetDrive();
}

int encoderPID(int power, int side)
{
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
			power = power + error * kp;
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
	bStopTasksBetweenModes = true;

	resetDriveEncoder();
	resetMogoEncoder();

	SensorType[gyro] = sensorNone;
	wait1Msec(500);
	SensorType[gyro] = sensorGyro;
	wait1Msec(1100);	
}

task autonomous()
{
	mogo(front);
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
	move(front);
	move(80, 121);
	move(-80, 121);
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
