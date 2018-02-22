#pragma config(Sensor, in1,    lineLeft,       sensorLineFollower)
#pragma config(Sensor, in2,    lineCenter,     sensorLineFollower)
#pragma config(Sensor, in3,    lineRight,      sensorLineFollower)
#pragma config(Sensor, in4,    gyro,           sensorGyro)
#pragma config(Sensor, dgtl1,  encoderMogo,    sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  encoderRight,   sensorQuadEncoder)
#pragma config(Sensor, dgtl5,  encoderLeft,    sensorQuadEncoder)
#pragma config(Sensor, dgtl7,  limitLeft,      sensorTouch)
#pragma config(Sensor, dgtl8,  limitRight,     sensorTouch)
#pragma config(Motor,  port2,           backR,         tmotorVex393TurboSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port3,           backL,         tmotorVex393TurboSpeed_MC29, openLoop)
#pragma config(Motor,  port4,           tipR,          tmotorVex393HighSpeed_MC29, openLoop)
#pragma config(Motor,  port5,           mogoR,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           mogoL,         tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port7,           tipL,          tmotorVex393HighSpeed_MC29, openLoop)
#pragma config(Motor,  port8,           frontL,        tmotorVex393TurboSpeed_MC29, openLoop)
#pragma config(Motor,  port9,           frontR,        tmotorVex393TurboSpeed_MC29, openLoop, reversed)


// This code is for the VEX cortex platform
#pragma platform(VEX2)

// Select Download method as "competition"
#pragma competitionControl(Competition)

//Main competition background code...do not modify!
#include "Vex_Competition_Includes.c"


int front = 1;
int back  = -1;
int right = 1;
int left = -1;

int final MAX_POWER = 100;
int final MIN_POWER = 40;
int final MAX_ERROR = 50;

void move(float targetDistance, int maxPower)
{
	float kp = 0;
	float ki = 0;
	float kd = 0;
	
	float gyroScale = 0;
	
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
	
	resetDriveEncoders();
	
	while(error != 0)
	{
		currentDistance = SensorValue[encoderRight] * gyroScale;

		error = targetDistance - currentDistance;

		integral = (error > MAX_ERROR / ki) ? 0 : (integral + error);
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
		powerLeft = encoderPID(power, left);
		powerRight = encoderPID(power, right);
		
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

void turnPID(float bearing)
{
	float kp = 0;
	float ki = 0;
	float kd = 0;
	
	float error = 1;
	float currentBearing = SensorValue[gyro];
	float targetBearing = bearing * 10 + currentBearing;
	
	while(error != 0)
	{
		error = targetBearing - currentBearing;
		integral = (error > MAX_ERROR / ki) ? 0 : (integral + error);
		derivative = error - lastError;
		
		if(power > 0)
		{
			if(power > maxPower)
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
