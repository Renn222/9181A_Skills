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

int g = 0;
int motorEncoderTest = 0;

int channel1 = 0;
int channel2 = 0;
int channel3 = 0;
int channel4 = 0;

int threshold = 10;

int R_FREQ = 75; // 100 the frequency of the recording in milisecons. A lower number will attempt to capture more data, but may result in errors in the recording
int R_Batt; // the battery voltage at the time of the recording.

int lineCounter = 0; // the number of each line of recorded code
int timeCounter = 0 * R_FREQ; // the time in mS of each line
int C1, C2, C3, C4, B5D, B5U, B6D, B6U, B7D, B7L, B7R, B7U, B8D, B8L, B8R, B8U; // signal values from main controller

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

		motor[frontL] = motor[frontR] = motor[backL] = motor[backR] = 0;
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

		motor[frontL] = motor[frontR] = motor[backL] = motor[backR] = 0;
	}
}

// task to record vexRT signals to the debug stream once every R_FREQ ms
task recordVexRT()
{
	while(true)
	{
		// main controller values
		C1 = vexRT[Ch1];
		C2 = vexRT[Ch2];
		C3 = vexRT[Ch3];
		C4 = vexRT[Ch4];
		B5D = vexRT[Btn5D];
		B5U = vexRT[Btn5U];
		B6D = vexRT[Btn6D];
		B6U = vexRT[Btn6U];
		B7D = vexRT[Btn7D];
		B7L = vexRT[Btn7L];
		B7R = vexRT[Btn7R];
		B7U = vexRT[Btn7U];
		B8D = vexRT[Btn8D];
		B8L = vexRT[Btn8L];
		B8R = vexRT[Btn8R];
		B8U = vexRT[Btn8U];

		// increment lineCounter and timeCounter
		lineCounter++;
		timeCounter = lineCounter * R_FREQ;

		// write the values from the  main controller to the debug stream window
		writeDebugStream("main_C(%d", C1);
		writeDebugStream(",%d", C2);
		writeDebugStream(",%d", C3);
		writeDebugStream(",%d", C4); // break signals into channel and button groups
		writeDebugStream("); main_B(%d", B5D);
		writeDebugStream(",%d", B5U);
		writeDebugStream(",%d", B6D);
		writeDebugStream(",%d", B6U);
		writeDebugStream(",%d", B7D);
		writeDebugStream(",%d", B7L);
		writeDebugStream(",%d", B7R);
		writeDebugStream(",%d", B7U);
		writeDebugStream(",%d", B8D);
		writeDebugStream(",%d", B8L);
		writeDebugStream(",%d", B8R);
		writeDebugStream(",%d", B8U);
		writeDebugStream(");  ");

		writeDebugStreamLine("replayVexRT(); // %d", timeCounter);

		wait1Msec((R_FREQ)+1);

	}
}

void deadZoneCheck()
{
	channel1 = (abs(vexRT[Ch1]) > threshold) ? vexRT[Ch1] : 0;
	channel2 = (abs(vexRT[Ch2]) > threshold) ? vexRT[Ch2] : 0;
	channel3 = (abs(vexRT[Ch3]) > threshold) ? vexRT[Ch3] : 0;
	channel4 = (abs(vexRT[Ch4]) > threshold) ? vexRT[Ch4] : 0;

	if(channel2 >= 120)
	{
		channel2 = 119;
	}
	else if(channel2 <= -120)
	{
		channel2 = -119;
	}
	if(channel3 >= 120)
	{
		channel3 = 119;
	}
	else if(channel3 <= -120)
	{
		channel3 = -119;
	}
}

task main()
{
	// clear previous recordings
	clearDebugStream();
	wait1Msec(100);
	// save current battery voltage to R_Batt
	R_Batt = nImmediateBatteryLevel;

	// countdown in debug stream
	writeDebugStreamLine("// Begin recording in: 3");
	wait1Msec(1000);
	writeDebugStreamLine("// Begin recording in: 2");
	wait1Msec(1000);
	writeDebugStreamLine("// Begin recording in: 1");
	wait1Msec(1000);

	// write header for autonomous code
	writeDebugStreamLine("// BEGIN AUTONOMOUS RECORDING");
	writeDebugStreamLine("// R_FREQ = %d", R_FREQ);
	writeDebugStream("R_Batt = %d", R_Batt);
	writeDebugStreamLine(";");
	writeDebugStreamLine("replayFreq();");

	// start recording the vexRT signals
	startTask (recordVexRT);

	// allow driver control for 15 seconds
	clearTimer(T1);

	while (time1[T1] < 60000)
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
			motor[frontL] = motor[frontR] = motor[backL] = motor[backR] = 0;
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

		if(vexRT[Btn8R] == 1)
		{
			turn(84);
		}
		else if(vexRT[Btn8L] == 1)
		{
			turn(-84);
		}
}
	// stop recording the vexRT signals
	stopTask (recordVexRT);
	writeDebugStreamLine("motor[port1] = motor[port2] = motor[port3] = motor[port4] = motor[port5] = motor[port6] = motor[port7] = motor[port8] = motor[port9] = motor[port10] = 0;");

	// stop all motors
	motor[frontL] = motor[frontR] = motor[backL] = motor[backR] = motor[mogoR] = motor[mogoL] = motor[tipL] = motor[tipR] = 0;
}
