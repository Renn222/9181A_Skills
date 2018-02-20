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

int kp = 0;
int ki = 0;
int kd = 0;

int MAX_POWER = 121;
int MIN_POWER = 40;
int MAX_ERROR = 50;

int constant = 1;

float distanceMogo1 = (48/circumference) * 360; //Distance for the first mogo
float distanceTest = (10/circumference) * 360;

int timeMogoIntake = 61; //number of ticks for the mobo intake to go down
int g = 0;


void move(float targetDistance, int power)
{
	/* Variables used to calculate how fast bot is moving*/
	float wheelDiameter = 4.5;
	float circumference = wheelDiameter * PI; //Roughly 14.13 inches
	float currentDistance = 0;
	float error = 0;
	float lastError = 0;
	float integral = 0;
	float derivative = 0;
	int power = 0;

	while(error < 5 && error > -5)
	{
		currentDistance = SensorValue[encoderRight] / 360 * circumference;

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
}
