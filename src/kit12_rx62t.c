﻿/***********************************************************************/
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
#define WAIT_LEFT_LINE 8
#define WAIT_RIGHT_LINE 9
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
int dasKalman(int measurement);
int getSteeringAngle(SensorInfo sensorResult);
int getSteeringAngleNew(int sensorResult);
void setSpeed(int handleAngle, int divisor);
/*======================================*/
/* Global variable declarations         */
/*======================================*/
unsigned long cnt0;
unsigned long cnt1;
unsigned long cnt2 = 1001;
int pattern;

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
				if (readSensor() != 0x00)
				{
					pattern = NORMAL_TRACE;
				}
				break;
			case NORMAL_TRACE:
				/* Normal trace */
				if(cnt2 < 250){ // 250 ms after a cross line / left line / right line only use normal trace
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
						pattern = WAIT_LEFT_LINE;
						cnt1 = 0;
						break;
					case 0x0f: // right line detected but with only 4 leds because the car could be a bit of center
					case RIGHT_LINE:
						pattern = WAIT_RIGHT_LINE;
						cnt1 = 0;
						break;
					default:
						led_out(0x0);
						traceTrack();
						break;
				}
				break;
			case WAIT_LEFT_LINE:
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
					pattern = CROSS_LINE;
					cnt1 = 0;
				}
				else
				{
					pattern = LEFT_LINE;
					cnt1 = 0;
				}
				break;
			case WAIT_RIGHT_LINE:
				// it can happen that we are at a cross line but the 4/5 leds at the right get
				// triggered at first and we detect the cross line as a right line. To prevent
				// this we wait a small amount and get a new reading from the sensors, if the
				// new reading is now a cross line than the previous reading was false and we
				// go into the cross line state otherwise it really was a right line and we go
				// into the left line state.
				timer(10);
				led_out(0x2);
				if (readSensor() == CROSS_LINE)
				{
					pattern = CROSS_LINE;
					cnt1 = 0;
				}
				else
				{
					pattern = RIGHT_LINE;
					cnt1 = 0;
				}
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
					handle(-46);
					motor(10, 60);
					pattern = SHARP_CORNER_LEFT;
					cnt1 = 0;
					break;
				}
				else if (maskSensorInfo(sensorResult, MASK0_4).Byte == 0x0f && cnt1 > 200)
				{
					// we ignore the left sensors and only evaluate the right ones if all
					// the right ones detect the line it means it is a right turn. So we set
					// the handle angle to the highest that the car can handle and also set
					// the motor to steering.
					handle(46);
					motor(60, 10);
					pattern = SHARP_CORNER_RIGHT;
					cnt1 = 0;
					break;
				}
				else
				{
					// as long as we haven't detected in which direction we have to steer,
					// we should still follow the line.
					handle(getSteeringAngle(readSensorInfo()));
					if (cnt1 < driveTimeForSharpTurns[NUM_SHARP_TURN]){
						// drive with the same speed than before
						motor(100,100);
					}
					else if (cnt1 < breakTimeForSharpTurns[NUM_SHARP_TURN] + driveTimeForSharpTurns[NUM_SHARP_TURN])
					{
						// in the first 200 ms we stop the motors completely to break even
						// harder to faster get to the desired speed of 40%. Eventually we
						// could break with negative values as well.
						motor(-100, -100);
					}
					else
					{
						motor(40, 40);
					}
					emergencyExit();
				}
				break;
			}
			case SHARP_CORNER_LEFT:
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
				if (cnt1 > 200 && readSensorInfoWithMask(LEFT_SENSORS_2).Byte != 0x00)
				{
					// to prevent a wrong reading as long as we are on the line with most
					// of the sensors we wait 200 ms. After that we wait till one of the
					// 2 left most sensors detect a line then we change to the normal trace.
					pattern = NORMAL_TRACE;
					traceMask = LEFT_MASK;
					cnt2 = 0;
					sharpTurnCounter++;
				}
				break;
			case SHARP_CORNER_RIGHT:
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
				if (cnt1 > 200 && readSensorInfoWithMask(RIGHT_SENSORS_2).Byte != 0x00)
				{
					// to prevent a wrong reading as long as we are on the line with most
					// of the sensors we wait 200 ms. After that we wait till one of the
					// 2 right most sensors detect a line then we change to the normal trace.
					pattern = NORMAL_TRACE;
					traceMask = RIGHT_MASK;
					cnt2 = 0;
					sharpTurnCounter++;
				}
				break;
			case LEFT_LINE:
				led_out(0x1);

				if (readSensor() == 0x00)
				{
					// after we lost the line we steer 25° in the direction where the line should be and go into pattern search line
					pattern = SEARCH_LINE_LEFT;
					break;
				}
				else
				{
					// while we wait for the line switch we are still steering and turning the motor down to 40%
					handle(getSteeringAngle(readSensorInfo()));
					motor(40, 40);

				}
				break;
			case RIGHT_LINE:
				led_out(0x1);
				if (readSensor() == 0x00)
				{
					// after we lost the line we go into pattern search line
					pattern = SEARCH_LINE_RIGHT;
					break;
				}
				else
				{
					// while we wait for the line switch we are still steering and turning the motor down to 40%
					handle(getSteeringAngle(readSensorInfo()));
					motor(40, 40);
				}
				break;
			case SEARCH_LINE_LEFT:
			{
				handle(-25);
				motor(40, 40);

				SensorInfo result = readSensorInfoWithMask(MASK2_4);
				led_out(0x3);
				if (result.Byte != 0x00)
				{
					// we go into pattern normal trace if the 2 outer most sensors are off and another sensor is on
					// if we would go directly into the normal trace state if the outer most sensor detects the line
					// we would steer from 25° to 45° which is in the wrong direction so we disabled the detection of
					// the outer most sensor. We also disabled the second most outer sensor to prevent the normal trace
					// to enter the line switch state again if the outer most sensor would turn on because it gets another
					// reading.
					pattern = NORMAL_TRACE;
					traceMask = RIGHT_MASK;
					cnt2 = 0;
					break;
				}
				break;
			}
			case SEARCH_LINE_RIGHT:
			{
				handle(25);
				motor(40, 40);

				SensorInfo result = readSensorInfoWithMask(MASK4_2);
				led_out(0x3);
				if (result.Byte != 0x00)
				{
					// we go into pattern normal trace if the 2 outer most sensors are off and another sensor is on
					// if we would go directly into the normal trace state if the outer most sensor detects the line
					// we would steer from 25° to 45° which is in the wrong direction so we disabled the detection of
					// the outer most sensor. We also disabled the second most outer sensor to prevent the normal trace
					// to enter the line switch state again if the outer most sensor would turn on because it gets another
					// reading.
					pattern = NORMAL_TRACE;
					traceMask = LEFT_MASK;
					cnt2 = 0;
					break;
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
//		handleAngle = dasKalman(handleAngle);

		int handleAngle = previousHandleAngle;

		if (abs(handleAngle) > 45)
			handleAngle = 45 * trackPosition;

		if (traceMask == LEFT_MASK)
			traceMask = MASK4_0;
		else if (traceMask == RIGHT_MASK)
			traceMask = MASK0_4;

		handle(handleAngle);
		setSpeed(handleAngle, 10);
		return;
	}
	int handleAngle = getSteeringAngle(maskedSensorResult);

//	handleAngle = dasKalman(handleAngle);

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

	handle(handleAngle);
	previousHandleAngle = handleAngle;
	setSpeed(handleAngle, 1);
	cnt1 = 0;
}

float errorEstimate = 2;
float errorMeasurement = 8;
float estimate = 0;
int previousMeasurement = 0;
int dasKalman(int measurement){
	if(measurement != previousMeasurement){
		previousMeasurement = measurement;
		errorEstimate = 2;
	}
	float kalmanGain = errorEstimate/(errorEstimate+errorMeasurement);
	float estimate = estimate + kalmanGain * (measurement-estimate);
	errorEstimate = (1-kalmanGain)*errorEstimate;
	return estimate;
}

int getSteeringAngleOld(SensorInfo sensorResult) {
	int baseSteering;
	switch(sensorResult.Byte) {
		case 0x38: baseSteering = -2; break; //0b0011 1000
		case 0x1c: baseSteering = 2; break;  //0b0001 1100

		case 0x08: baseSteering = 3; break;  //0b0000 1000
		case 0x04: baseSteering = 12; break; //0b0000 0100
		case 0x02: baseSteering = 28; break; //0b0000 0010
		case 0x01: baseSteering = 44; break; //0b0000 0001

		case 0x0C: baseSteering = 8; break;  //0b0000 1100
		case 0x06: baseSteering = 20; break; //0b0000 0110
		case 0x03: baseSteering = 36; break; //0b0000 0011

		case 0x0e: baseSteering = 12; break; //0b0000 1110
		case 0x07: baseSteering = 28; break; //0b0000 0111

		case 0x0f: baseSteering = 20; break; //0b0000 1111


		case 0x10: baseSteering = -3; break;  //0b0001 0000
		case 0x20: baseSteering = -12; break; //0b0010 0000
		case 0x40: baseSteering = -28; break; //0b0100 0000
		case 0x80: baseSteering = -44; break; //0b1000 0000

		case 0x30: baseSteering = -8;  break; //0b0011 0000
		case 0x60: baseSteering = -20; break; //0b0110 0000
		case 0xc0: baseSteering = -36; break; //0b1100 0000

		case 0x70: baseSteering = -12; break; //0b0111 0000
		case 0xe0: baseSteering = -28; break; //0b1110 0000

		case 0xf0: baseSteering = -20; break; //0b1111 0000

		case 0xf8: baseSteering = -12; break; //0b1111 1000
		case 0x78: baseSteering = -8; break; //0b0111 1000

		case 0x1f: baseSteering = 12; break; //0b0001 1111
		case 0x1e: baseSteering = 8; break; //0b0001 1110

		//0b0001 1000
		default: baseSteering = 0; break;
	}
	return baseSteering;
}
int steeringTable[] = { 0, 3, 8, 12, 20, 28, 36, 44 };
int getSteeringAngle(SensorInfo sensorInfoResult)
{
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

void setSpeed(int handleAngle, int divisor)
{
	int angleFactor = abs(handleAngle) * 100 / 45;

	int fasterSpeed = 100 - angleFactor * 0.1f;
	int slowerSpeed = fasterSpeed - (fasterSpeed * (angleFactor / 200.0f));

	fasterSpeed = fasterSpeed / divisor;
	slowerSpeed = slowerSpeed / divisor;

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
