/***********************************************************************/
/*  Supported Microcontroller:RX62T                                    */
/*  File:                   kit12_rx62t.c                              */
/*  File Contents:          MCU Car Trace Basic Program(RX62T version) */
/*  Version number:         Ver.1.00                                   */
/*  Date:                   2013.09.01                                 */
/*  Copyright:              Renesas Micom Car Rally Secretariat        */
/***********************************************************************/
/*
 This program supports the following boards:
 * RMC-RX62T board
 * Sensor board Ver. 5
 * Motor drive board Ver. 5
 */

/*======================================*/
/* Include                              */
/*======================================*/
#include <machine.h>
#include "iodefine.h"
#include "stdlib.h"
#include "math.h"
#include "debug.h"
#include "customMath.h"
#include "SensorInfo.h"
#include "CarControl.h"
/*======================================*/
/* Symbol definitions                   */
/*======================================*/

/* Masked value settings X:masked (disabled) O:not masked (enabled) */
#define MASK2_2         0x66            /* X O O X  X O O X            */
#define MASK2_0         0x60            /* X O O X  X X X X            */
#define MASK0_2         0x06            /* X X X X  X O O X            */
#define MASK3_3         0xe7            /* O O O X  X O O O            */
#define MASK0_3         0x07            /* X X X X  X O O O            */
#define MASK3_0         0xe0            /* O O O X  X X X X            */
#define MASK4_0         0xf0            /* O O O O  X X X X            */
#define MASK0_4         0x0f            /* X X X X  O O O O            */
#define MASK1_4         0x1f            /* X X X 0  O O O O            */
#define MASK4_1         0xf8            /* 0 0 0 0  O X X X            */
#define MASK2_4         0x3f            /* X X 0 0  O O O O            */
#define MASK4_2         0xfc            /* 0 0 0 0  O 0 X X            */
#define MASK4_4         0xff            /* O O O O  O O O O            */
#define MASK2_1         0x38            /* X X 0 O  O X X X            */
#define MASK1_2         0x1c            /* X X X O  O 0 X X            */
#define LEFT_SENSORS_2  0xc0            /* 0 0 X X  X X X X            */
#define RIGHT_SENSORS_2 0x03            /* X X X X  X X 0 0            */

#define LEFT_MASK MASK4_2
#define RIGHT_MASK MASK2_4
#define NORMAL_MASK MASK4_4

#define RIGHT_LINE 	0x1f //O O O X  X X X X
#define LEFT_LINE 	0xf8 //X X X X  X O O O
#define CROSS_LINE  0xff //X X X X  X X X X

#define WAIT_FOR_SWITCH 0
#define WAIT_FOR_STARTBAR 1
#define WAIT_FOR_LOST_TRACK 2
#define NORMAL_TRACE 3
#define DETECT_SHARP_CORNER 4
#define SHARP_CORNER_LEFT 5
#define SHARP_CORNER_RIGHT 6
#define WAIT_HALF_LINE 8
#define SEARCH_LINE_RIGHT 10
#define SEARCH_LINE_LEFT 11

#define LEFT -1
#define RIGHT 1
#define NORMAL 0
/*======================================*/
/* Prototype declarations               */
/*======================================*/
void timer(unsigned long timer_set);
void traceTrack();
int interpolateAngle(int measurement);
int getSteeringAngle(SensorInfo sensorResult);
int getSteeringAngleNew(int sensorResult);
void setSpeed(int handleAngle, int divisor);
void setSpeedAndHandleAngle(int maxSpeed);
int setHandleAngleWithMask(unsigned char mask);
int setHandleAngle();
int setHandleAngleFromResult(SensorInfo sensorResult);
/*======================================*/
/* Global variable declarations         */
/*======================================*/
unsigned long cnt0;
unsigned long cnt1;
unsigned long cnt2 = 1001;
int pattern;
int nextPattern = 0;

int previousHandleAngle = 0;
int trackPosition = 0;
int traceMask = NORMAL_MASK;

/* 90° Turn Counter */
int sharpTurnCounter = 0;
#define TOTAL_SHARP_TURNS 3
#define NUM_SHARP_TURN (sharpTurnCounter % TOTAL_SHARP_TURNS)

int driveTimeForSharpTurns[TOTAL_SHARP_TURNS] = { 0, 400, 100 };
int breakTimeForSharpTurns[TOTAL_SHARP_TURNS] = { 650, 100, 300 };


void emergencyExit(void)
{
	if (!readSensor())
	{
		PRINT_L("emergency exit");
		motor(0, 0);
		pattern = WAIT_FOR_LOST_TRACK;
	}
}



/***********************************************************************/
/* Main program                                                        */
/***********************************************************************/
void main(void)
{
	/* Initialize MCU functions */
	init();
	debug_init();

	/* Initialize micom car state */
	handle(0);
	motor(0, 0);

	//BREAK2();

	while (1)
	{
		switch (pattern)
		{
			case WAIT_FOR_SWITCH:
				/* Wait for switch input */
				PRINT_L("wait for switch");
				if (pushsw_get())
				{
					pattern = WAIT_FOR_STARTBAR;
					cnt1 = 0;
					break;
				}
				if (cnt1 < 100)
				{ /* LED flashing processing     */
					led_out(0x1);
				}
				else if (cnt1 < 200)
				{
					led_out(0x2);
				}
				else
				{
					cnt1 = 0;
				}
				break;
			case WAIT_FOR_STARTBAR:
				/* Check if start bar is open */
				PRINT_L("wait for startbar");
				if (!startbar_get())
				{
					/* Start!! */
					led_out(0x0);
					pattern = NORMAL_TRACE;
					cnt1 = 0;
					break;
				}
				if (cnt1 < 50)
				{ /* LED flashing processing     */
					led_out(0x1);
				}
				else if (cnt1 < 100)
				{
					led_out(0x2);
				}
				else
				{
					cnt1 = 0;
				}
				break;
			case WAIT_FOR_LOST_TRACK:
			{
				unsigned char sensorResult = readSensor();
				PRINT_L("wait for lost track");
				PRINT_I(sensorResult);
				if (sensorResult != 0x00)
				{
					pattern = NORMAL_TRACE;
				}
				break;
			}
			case NORMAL_TRACE:
				/* Normal trace */
				if(cnt2 < 250){ // 250 ms after a cross line / left line / right line only use normal trace
					PRINT_L("normal trace ignore cross/left/right line");
					PRINT_I(cnt2);
					led_out(0x0);
					traceTrack();
					break;
				}
				switch (readSensorInfo().Byte)
				{
					case CROSS_LINE:
						pattern = CROSS_LINE;
						cnt1 = 0;
						break;
					case 0xf0: // left line detected but with only 4 leds because the car could be a bit of center
					case LEFT_LINE:
						pattern = WAIT_HALF_LINE;
						nextPattern = LEFT_LINE;
						cnt1 = 0;
						break;
					case 0x0f: // right line detected but with only 4 leds because the car could be a bit of center
					case RIGHT_LINE:
						pattern = WAIT_HALF_LINE;
						nextPattern = RIGHT_LINE;
						cnt1 = 0;
						break;
					default:
						led_out(0x0);
						traceTrack();
						break;
				}
				break;
			case WAIT_HALF_LINE:
				// it can happen that we are at a cross line but the 4/5 leds at the left get
				// triggered at first and we detect the cross line as a left line. To prevent
				// this we wait a small amount and get a new reading from the sensors, if the
				// new reading is now a cross line than the previous reading was false and we
				// go into the cross line state otherwise it really was a left line and we go
				// into the left line state.
				timer(10);
				led_out(0x2);
				if (readSensor() == CROSS_LINE)
				{
					PRINT_L("wait half line => crossline");
					pattern = CROSS_LINE;
				}
				else
				{
					PRINT("wait half line => %d\r\n", nextPattern);
					pattern = nextPattern;
				}
				cnt1 = 0;
				break;
			case CROSS_LINE:
			{
				SensorInfo sensorResult = readSensorInfo();
				// we wait 200 ms to ignore the second cross line which can not be detected when
				// the car is really fast so we just ignore it.
				if (maskSensorInfo(sensorResult, MASK4_0).Byte == 0xf0 && cnt1 > 200)
				{
					// we ignore the right sensors and only evaluate the left ones if all
					// the left ones detect the line it means it is a left turn. So we set
					// the handle angle to the highest that the car can handle and also set
					// the motor to steering.
					PRINT_L("90° left");
					handle(-46);
					motor(10, 60);
					pattern = SHARP_CORNER_LEFT;
					cnt1 = 0;
				}
				else if (maskSensorInfo(sensorResult, MASK0_4).Byte == 0x0f && cnt1 > 200)
				{
					// we ignore the left sensors and only evaluate the right ones if all
					// the right ones detect the line it means it is a right turn. So we set
					// the handle angle to the highest that the car can handle and also set
					// the motor to steering.
					PRINT_L("90° right");
					handle(46);
					motor(60, 10);
					pattern = SHARP_CORNER_RIGHT;
					cnt1 = 0;
				}
				else
				{
					// as long as we haven't detected in which direction we have to steer,
					// we should still follow the line.
					if (cnt1 < driveTimeForSharpTurns[NUM_SHARP_TURN]){
						// drive with the same speed than before
						setSpeedAndHandleAngle(100);
						PRINT("wait for sharp corner #%d cnt1 = %d drive fast", NUM_SHARP_TURN, cnt1);
					}
					else if (cnt1 < breakTimeForSharpTurns[NUM_SHARP_TURN] + driveTimeForSharpTurns[NUM_SHARP_TURN])
					{
						// in the first 200 ms we stop the motors completely to break even
						// harder to faster get to the desired speed of 40%. Eventually we
						// could break with negative values as well.
						setSpeedAndHandleAngle(-100);
						PRINT("wait for sharp corner #%d cnt1 = %d break", NUM_SHARP_TURN, cnt1);
					}
					else
					{
						setSpeedAndHandleAngle(40);
						PRINT("wait for sharp corner #%d cnt1 = %d drive slow", NUM_SHARP_TURN, cnt1);
					}
					emergencyExit();
				}
				break;
			}
			case SHARP_CORNER_LEFT:{
				handle(-46);
				if (cnt1 < 100)
				{
					// I tried to steer even harder by reversing the one motor for 100 ms,
					// I don't know if that is really working.
					motor(-100, 60);

				}
				else
				{
					motor(10, 60);
				}
				SensorInfo sensorResult = readSensorInfoWithMask(LEFT_SENSORS_2);
				if (cnt1 > 200 && sensorResult.Byte != 0x00)
				{
					// to prevent a wrong reading as long as we are on the line with most
					// of the sensors we wait 200 ms. After that we wait till one of the
					// 2 left most sensors detect a line then we change to the normal trace.
					pattern = NORMAL_TRACE;
					traceMask = LEFT_MASK;
					cnt2 = 0;
					sharpTurnCounter++;
					PRINT("sharp corner left => normal trace, sensor %d", sensorResult.Byte);
				}else if(cnt1 < 200){
					PRINT_L("sharp corner left wait 200ms");
				}else{
					PRINT("sharp corner left wait, sensor %d", sensorResult.Byte);
				}
				break;
			}
			case SHARP_CORNER_RIGHT:
			{
				handle(46);
				if (cnt1 < 100)
				{
					// I tried to steer even harder by reversing the one motor for 100 ms,
					// I don't know if that is really working.
					motor(60, -100);
				}
				else
				{
					motor(60, 10);
				}
				SensorInfo sensorResult = readSensorInfoWithMask(RIGHT_SENSORS_2);
				if (cnt1 > 200 && sensorResult.Byte != 0x00)
				{
					// to prevent a wrong reading as long as we are on the line with most
					// of the sensors we wait 200 ms. After that we wait till one of the
					// 2 right most sensors detect a line then we change to the normal trace.
					pattern = NORMAL_TRACE;
					traceMask = RIGHT_MASK;
					cnt2 = 0;
					sharpTurnCounter++;
					PRINT("sharp corner right => normal trace, sensor %d", sensorResult.Byte);
				}else if(cnt1 < 200){
					PRINT_L("sharp corner right wait 200ms");
				}else{
					PRINT("sharp corner right wait, sensor %d", sensorResult.Byte);
				}
				break;
			}
			case LEFT_LINE:
				led_out(0x1);
				if (readSensor() == 0x00)
				{
					// after we lost the line we steer 25° in the direction where the line should be and go into pattern search line
					pattern = SEARCH_LINE_LEFT;
					PRINT_L("lost line search line left");
				}
				else
				{
					// while we wait for the line switch we are still steering and turning the motor down to 40%
					setSpeedAndHandleAngle(40);
					PRINT_L("wait for line change left");
				}
				break;
			case RIGHT_LINE:
				led_out(0x1);
				if (readSensor() == 0x00)
				{
					// after we lost the line we go into pattern search line
					pattern = SEARCH_LINE_RIGHT;
					PRINT_L("lost line search line right");
				}
				else
				{
					// while we wait for the line switch we are still steering and turning the motor down to 40%
					setSpeedAndHandleAngle(40);
					PRINT_L("wait for line change left");
				}
				break;
			case SEARCH_LINE_LEFT:
			{
				handle(-25);
				setSpeed(-25, 40);

				SensorInfo result = readSensorInfo();
				SensorInfo maskedResult = maskSensorInfo(result, MASK2_4);
				PRINT("search line left, sensor %d, masked %d", result, maskedResult);
				led_out(0x3);
				if (maskedResult.Byte != 0x00)
				{
					// we go into pattern normal trace if the 2 outer most sensors are off and another sensor is on
					// if we would go directly into the normal trace state if the outer most sensor detects the line
					// we would steer from 25° to 45° which is in the wrong direction so we disabled the detection of
					// the outer most sensor. We also disabled the second most outer sensor to prevent the normal trace
					// to enter the line switch state again if the outer most sensor would turn on because it gets another
					// reading.
					PRINT_L("search line left => normal trace");
					pattern = NORMAL_TRACE;
					traceMask = RIGHT_MASK;
					cnt2 = 0;
				}
				break;
			}
			case SEARCH_LINE_RIGHT:
			{
				handle(25);
				setSpeed(25, 40);


				SensorInfo result = readSensorInfo();
				SensorInfo maskedResult = maskSensorInfo(result, MASK4_2);
				PRINT("search line right, sensor %d, masked %d", result, maskedResult);
				led_out(0x3);
				if (maskedResult.Byte != 0x00)
				{
					// we go into pattern normal trace if the 2 outer most sensors are off and another sensor is on
					// if we would go directly into the normal trace state if the outer most sensor detects the line
					// we would steer from 25° to 45° which is in the wrong direction so we disabled the detection of
					// the outer most sensor. We also disabled the second most outer sensor to prevent the normal trace
					// to enter the line switch state again if the outer most sensor would turn on because it gets another
					// reading.
					PRINT_L("search line right => normal trace");
					pattern = NORMAL_TRACE;
					traceMask = LEFT_MASK;
					cnt2 = 0;
				}
				break;
			}
			default:
				/* If neither, return to standby state */
				motor(0, 0);
				pattern = WAIT_FOR_SWITCH;
				break;
		}
	}
}

void setSpeedAndHandleAngle(int maxSpeed){
	int angle = setHandleAngle();
	setSpeed(angle, maxSpeed);
}

int setHandleAngleWithMask(unsigned char mask){
	SensorInfo maskedSensorResult = readSensorInfoWithMask(mask);
	return setHandleAngleFromResult(maskedSensorResult);
}
int setHandleAngle(){
	return setHandleAngleWithMask(MASK4_4);
}
int setHandleAngleFromResult(SensorInfo sensorResult){
	int handleAngle = getSteeringAngle(sensorResult);
//	handleAngle = interpolateAngle(handleAngle);
	handle(handleAngle);
	previousHandleAngle = handleAngle;
	return handleAngle;
}

void traceTrack()
{
	SensorInfo unmaskedSensorResult = readSensorInfo();
	SensorInfo maskedSensorResult = maskSensorInfo(unmaskedSensorResult, traceMask);

	if (maskedSensorResult.Byte == 0x00)
	{
		if(cnt1 > 1000){
			// after 1 second stop, you wont find the track again ;)
			motor(0,0);
			pattern = WAIT_FOR_LOST_TRACK;
			return;
		}
		// lost track => steer more in the current direction until the track is found again
		// and slow down to prevent crashing into things

//		int handleAngle = 45*trackPosition;
//		handleAngle = interpolateAngle(handleAngle);

		int handleAngle = previousHandleAngle;

		if (abs(handleAngle) > 45)
			handleAngle = 45 * trackPosition;

		if (traceMask == LEFT_MASK)
			traceMask = MASK4_0;
		else if (traceMask == RIGHT_MASK)
			traceMask = MASK0_4;

		handle(handleAngle);
		setSpeed(handleAngle, 40);
		return;
	}
	int handleAngle = setHandleAngleFromResult(maskedSensorResult);

	// to set the mask we must check the masked result because an unmasked result would
	// return a wrong angle or zero if the 2 outer most sensors are active which would
	// reset the mask.
	if (abs(handleAngle) < 5)
	{
		traceMask = NORMAL_MASK;
	}
	else if (handleAngle < 0)
	{
		traceMask = LEFT_MASK;
		trackPosition = LEFT;
	}
	else if (handleAngle > 0)
	{
		traceMask = RIGHT_MASK;
		trackPosition = RIGHT;
	}

	setSpeed(handleAngle, 100);
	cnt1 = 0;
}

float previousAngle = 0;
int interpolateAngle(int measurement){
	float angle = previousAngle;
	float difference = previousAngle - measurement;
	float scale = (difference*difference)/90*90;
	if(previousAngle < measurement){
		angle += 0.005*scale;
	}else if (previousAngle > measurement){
		angle -= 0.005*scale;
	}else{
		angle += 0;
	}
	previousAngle = angle;
	return angle;
}

int getSteeringAngle(SensorInfo sensorInfoResult)
{
	static int steeringTable[] = { 0, 3, 8, 12, 20, 28, 36, 44 };

	int sensorResult = sensorInfoResult.Byte;
	if (sensorResult == 0)
		return steeringTable[0];

	int sum = 0, c = 0, i = 0;
	// solange noch aktive Bits in sensorResult
	while (sensorResult != 0)
	{
		// ist das 1. (rechte) bit 1?
		if (sensorResult % 2)
		{
			// wenn ja addiere die aktuelle bit position auf die summe
			sum += i;
			// und erhöhe counter
			c++;
		}
		// schiebe bits nach rechts
		sensorResult >>= 1;
		// anpassen der aktuellen bitposition
		i -= 2;
	}
	// ermittle index
	int idx = (sum - c / 2) / c + 7;
	// wenn index negativ => negiere agle
	if (idx < 0)
		return -steeringTable[-idx];
	else
		return steeringTable[idx];
}

void setSpeed(int handleAngle, int maxSpeed)
{
	int angleFactor = abs(handleAngle) * abs(maxSpeed) / 45;

	int fasterSpeed = abs(maxSpeed) - angleFactor * 0.1f;
	int slowerSpeed = fasterSpeed - (fasterSpeed * (angleFactor / 200.0f));

	if(maxSpeed < 0){
		fasterSpeed = -fasterSpeed;
		slowerSpeed = -slowerSpeed;
	}

	int motorSpeedLeft;
	int motorSpeedRight;
	if (handleAngle < 0)
	{
		motorSpeedRight = fasterSpeed;
		motorSpeedLeft = slowerSpeed;
	}
	else
	{
		motorSpeedRight = slowerSpeed;
		motorSpeedLeft = fasterSpeed;
	}
	motor(motorSpeedLeft, motorSpeedRight);
}

/***********************************************************************/
/* Interrupt                                                           */
/***********************************************************************/
#pragma interrupt Excep_CMT0_CMI0(vect=28)
void Excep_CMT0_CMI0(void)
{
	cnt0++;
	cnt1++;
	cnt2++;
}

/***********************************************************************/
/* Timer unit                                                          */
/* Arguments: timer value, 1 = 1 ms                                    */
/***********************************************************************/
void timer(unsigned long timer_set)
{
	cnt0 = 0;
	while (cnt0 < timer_set);
}
