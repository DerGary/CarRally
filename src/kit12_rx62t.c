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

#define CAR 5
#define SERVO_CENTER_CAR_1 2406
#define SERVO_CENTER_CAR_2 2291
#define SERVO_CENTER_CAR_3 2321
#define SERVO_CENTER_CAR_4 2418
#define SERVO_CENTER_CAR_5 2226
#if CAR == 1
	#define SERVO_CENTER    SERVO_CENTER_CAR_1          /* Servo center value          */
	#define DRIVETIMES obstacleDriveTime1
	#define BREAKTIMES obstacleBreakTime1
	#define ACCELERATIONTIMES obstacleAccelerationTimeCar1
	#define HANDLE_FACTOR -1
	#define SPEED_FACTOR 100
#elif CAR == 2
	#define SERVO_CENTER    SERVO_CENTER_CAR_2          /* Servo center value          */
	#define DRIVETIMES obstacleDriveTime2
	#define BREAKTIMES obstacleBreakTime2
	#define ACCELERATIONTIMES obstacleAccelerationTimeCar2
	#define HANDLE_FACTOR 1
	#define SPEED_FACTOR 100
#elif CAR == 3
	#define SERVO_CENTER    SERVO_CENTER_CAR_3          /* Servo center value          */
	#define DRIVETIMES obstacleDriveTime3
	#define BREAKTIMES obstacleBreakTime3
	#define ACCELERATIONTIMES obstacleAccelerationTimeCar3
	#define HANDLE_FACTOR 1
	#define SPEED_FACTOR 100
#elif CAR == 4
	#define SERVO_CENTER    SERVO_CENTER_CAR_4          /* Servo center value          */
	#define DRIVETIMES obstacleDriveTime4
	#define BREAKTIMES obstacleBreakTime4
	#define ACCELERATIONTIMES obstacleAccelerationTimeCar4
	#define HANDLE_FACTOR 1
	#define SPEED_FACTOR 65
#elif CAR == 5
	#define SERVO_CENTER    SERVO_CENTER_CAR_5          /* Servo center value          */
	#define DRIVETIMES obstacleDriveTime4
	#define BREAKTIMES obstacleBreakTime4
	#define ACCELERATIONTIMES obstacleAccelerationTimeCar4
	#define HANDLE_FACTOR 1
	#define SPEED_FACTOR 65
#else
	#error Unknown CAR
#endif

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
#define MASK2_2         0x3C            /* X X O 0  0 O X X            */
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

#define LEFT_MASK MASK4_1
#define RIGHT_MASK MASK1_4
#define NORMAL_MASK MASK4_4

#define RIGHT_LINE 	0x1f //O O O 0  X X X X
#define LEFT_LINE 	0xf8 //X X X X  0 O O O
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
void handle(int angle);
/*======================================*/
/* Global variable declarations         */
/*======================================*/
unsigned long cnt0;
unsigned long cnt1;
unsigned long cnt2;
unsigned long cnt3;
unsigned char nextPattern = 0;

// Holds the program state
Message state = {0};

int previousHandleAngle = 0;
int trackPosition = 0;

/* 90° Turn and Line Switch Counter */
int obstacleCounter = 0;
#define TOTAL_OBSTACLES 8
#define CURRENT_OBSTACLE (obstacleCounter % TOTAL_OBSTACLES)

//todo: change these factors for each car
#define WAIT_HALF_LINE_TIMER 10
#define WAIT_FOR_LANE_CHANGE_SPEED 70*(SPEED_FACTOR/100.0)
#define SHARP_CORNER_HANDLE_ANGLE 46
#define SHARP_CORNER_SPEED_FAST 60
#define SHARP_CORNER_SPEED_SLOW 10
#define LANE_SWITCH_HANDLE_ANGLE 35
#define LANE_SWITCH_SPEED 30
#define ANGLE_SPEED_FACTOR 0.15f
#define SLOPE_DOWN_SPEED_FACTOR 50
#define OBSTACLE_DRIVE_SPEED SPEED_FACTOR

//LSR LSL 90° 90° 90° 90° 90° 90°
int obstacleDriveTime1[TOTAL_OBSTACLES] = {0, 	200, 	350, 	0,		0, 		100, 0,0 };
int obstacleBreakTime1[TOTAL_OBSTACLES] = {650, 200, 	200, 	0, 		0, 		300, 0,0};
int obstacleDriveTime2[TOTAL_OBSTACLES] = {0, 	200, 	500, 	0, 		0, 		100, 0,0 };
int obstacleBreakTime2[TOTAL_OBSTACLES] = {650, 200, 	100, 	0, 		0, 		300, 0,0 };
int obstacleDriveTime3[TOTAL_OBSTACLES] = {100, 200, 	500, 	0, 		0, 		100, 0,0 };
int obstacleBreakTime3[TOTAL_OBSTACLES] = {500, 200, 	100, 	0, 		0, 		300, 0,0 };
int obstacleDriveTime4[TOTAL_OBSTACLES] = {0, 	0, 	100, 	150, 	0, 0, 0,0 };
int obstacleBreakTime4[TOTAL_OBSTACLES] = {0, 	0, 	150, 	100, 	50, 100, 100, 150 };

int obstacleAccelerationTimeCar1[TOTAL_OBSTACLES] = { 0, 0, 0, 0, 0, 0, 0,0 };
int obstacleAccelerationTimeCar2[TOTAL_OBSTACLES] = { 0, 0, 0, 0, 0, 0, 0,0 };
int obstacleAccelerationTimeCar3[TOTAL_OBSTACLES] = { 0, 0, 0, 0, 0, 0, 0,0 };
int obstacleAccelerationTimeCar4[TOTAL_OBSTACLES] = { 700, 500, 700, 400, 1000, 100, 300, 400};

int* obstacleDriveTime = DRIVETIMES;
int* obstacleBreakTime = BREAKTIMES;
int* obstacleAccelerationTime = ACCELERATIONTIMES;
char slopeup = 0;
char ontop = 0;
char slopedown = 0;


void handle(int steeringAngle){
	if(steeringAngle > 45){
		steeringAngle = 45;
	}else if(steeringAngle < -45){
		steeringAngle = -45;
	}
	setServo(steeringAngle*HANDLE_FACTOR);
	state.Angle = steeringAngle;
}

void motor(int speedMotorLeft, int speedMotorRight){
	setMotor(speedMotorLeft, speedMotorRight);
	state.MotorLeft = speedMotorLeft;
	state.MotorRight = speedMotorRight;
}

void emergencyExit(void)
{
	if (!state.Sensor.Byte)
	{
		motor(0, 0);
		state.Pattern = WAIT_FOR_LOST_TRACK;
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
	state.Pattern = WAIT_FOR_SWITCH;
	state.TraceMask = NORMAL_MASK;

	while (1)
	{
		state.Sensor = readSensorInfo();

		switch (state.Pattern)
		{
			case WAIT_FOR_SWITCH:
				/* Wait for switch input */
				if (pushsw_get())
				{
					state.Pattern = WAIT_FOR_STARTBAR;
					cnt1 = 0;
					cnt2 = 0;
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
					state.Pattern = NORMAL_TRACE;
					cnt1 = 0;
					cnt2 = 0;
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
				if (state.Sensor.Byte != 0x00)
				{
					state.Pattern = NORMAL_TRACE;
				}
				if (pushsw_get())
				{
					sendDebugBuffer();
				}
				break;
			}
			case NORMAL_TRACE:
			{
				/* Normal trace */
				if (cnt2 < 250)
				{ // 250 ms after a cross line / left line / right line only use normal trace
					led_out(0x0);
					traceTrack();
					break;
				}
				if (state.Sensor.Byte == CROSS_LINE)
				{
					state.Pattern = CROSS_LINE;
					cnt1 = 0;
				}
				else if (maskSensorInfo(state.Sensor, MASK4_1).Byte == LEFT_LINE)
				{
					state.Pattern = WAIT_HALF_LINE;
					nextPattern = LEFT_LINE;
					cnt1 = 0;
				}
				else if (maskSensorInfo(state.Sensor, MASK1_4).Byte == RIGHT_LINE)
				{
					state.Pattern = WAIT_HALF_LINE;
					nextPattern = RIGHT_LINE;
					cnt1 = 0;
				}
				else
				{
					led_out(0x0);
					traceTrack();
				}
				DBG();
				break;
			}
			case WAIT_HALF_LINE:
				// it can happen that we are at a cross line but the 4/5 leds at the left get
				// triggered at first and we detect the cross line as a left line. To prevent
				// this we wait a small amount and get a new reading from the sensors, if the
				// new reading is now a cross line than the previous reading was false and we
				// go into the cross line state otherwise it really was a left line and we go
				// into the left line state.
				state.Sensor = readSensorInfo();
				led_out(0x2);
				if (state.Sensor.Byte == CROSS_LINE)
				{
					state.Pattern = CROSS_LINE;
					cnt1 = 0;
					DBG();
				}
				else if(cnt1 > 20)
				{
					state.Pattern = nextPattern;
					cnt1 = 0;
					DBG();
				}
				break;
			case CROSS_LINE:
			{
				// we wait 200 ms to ignore the second cross line which can not be detected when
				// the car is really fast so we just ignore it.
				if (maskSensorInfo(state.Sensor, MASK4_1).Byte == LEFT_LINE && cnt1 > 100)
				{
					// we ignore the right sensors and only evaluate the left ones if all
					// the left ones detect the line it means it is a left turn. So we set
					// the handle angle to the highest that the car can handle and also set
					// the motor to steering.
					handle(-SHARP_CORNER_HANDLE_ANGLE);
					motor(SHARP_CORNER_SPEED_SLOW, SHARP_CORNER_SPEED_FAST);
					state.Pattern = SHARP_CORNER_LEFT;
					cnt1 = 0;
				}
				else if (maskSensorInfo(state.Sensor, MASK1_4).Byte == RIGHT_LINE && cnt1 > 100)
				{
					// we ignore the left sensors and only evaluate the right ones if all
					// the right ones detect the line it means it is a right turn. So we set
					// the handle angle to the highest that the car can handle and also set
					// the motor to steering.
					handle(SHARP_CORNER_HANDLE_ANGLE);
					motor(SHARP_CORNER_SPEED_FAST, SHARP_CORNER_SPEED_SLOW);
					state.Pattern = SHARP_CORNER_RIGHT;
					cnt1 = 0;
				}
				else
				{
					// as long as we haven't detected in which direction we have to steer,
					// we should still follow the line.
					if (cnt1 < obstacleDriveTime[CURRENT_OBSTACLE]){
						// drive with the same speed than before
						setSpeedAndHandleAngle(OBSTACLE_DRIVE_SPEED);
					}
					else if (cnt1 < obstacleBreakTime[CURRENT_OBSTACLE] + obstacleDriveTime[CURRENT_OBSTACLE])
					{
						// in the first 200 ms we stop the motors completely to break even
						// harder to faster get to the desired speed of 40%. Eventually we
						// could break with negative values as well.
						setSpeedAndHandleAngle(-100);
					}
					else
					{
						setSpeedAndHandleAngle(SHARP_CORNER_SPEED_FAST);
					}
					emergencyExit();
				}
				DBG();
				break;
			}
			case SHARP_CORNER_LEFT:{
				handle(-SHARP_CORNER_HANDLE_ANGLE);
				if (cnt1 < 100)
				{
					// I tried to steer even harder by reversing the one motor for 100 ms,
					// I don't know if that is really working.
					motor(-100, SHARP_CORNER_SPEED_FAST);
				}
				else
				{
					motor(SHARP_CORNER_SPEED_SLOW, SHARP_CORNER_SPEED_FAST);
				}
				//Todo try without cnt1 > 200
				// we could wait till we see nothing and then go into this section if a sensor is on again
				if (cnt1 > 200 && maskSensorInfo(state.Sensor, LEFT_SENSORS_2).Byte != 0x00)
				{
					// to prevent a wrong reading as long as we are on the line with most
					// of the sensors we wait 200 ms. After that we wait till one of the
					// 2 left most sensors detect a line then we change to the normal trace.
					state.Pattern = NORMAL_TRACE;
					state.TraceMask = LEFT_MASK;
					cnt2 = 0;
					obstacleCounter++;
				}
				DBG();
				break;
			}
			case SHARP_CORNER_RIGHT:
			{
				handle(SHARP_CORNER_HANDLE_ANGLE);
				if (cnt1 < 100)
				{
					// I tried to steer even harder by reversing the one motor for 100 ms,
					// I don't know if that is really working.
					motor(SHARP_CORNER_SPEED_FAST, -100);
				}
				else
				{
					motor(SHARP_CORNER_SPEED_FAST, SHARP_CORNER_SPEED_SLOW);
				}
				if (cnt1 > 200 && maskSensorInfo(state.Sensor, RIGHT_SENSORS_2).Byte != 0x00)
				{
					// to prevent a wrong reading as long as we are on the line with most
					// of the sensors we wait 200 ms. After that we wait till one of the
					// 2 right most sensors detect a line then we change to the normal trace.
					state.Pattern = NORMAL_TRACE;
					state.TraceMask = RIGHT_MASK;
					cnt2 = 0;
					obstacleCounter++;
				}
				DBG();
				break;
			}
			case LEFT_LINE:
				led_out(0x1);
				if (state.Sensor.Byte == 0x00)
				{
					// after we lost the line we steer 25° in the direction where the line should be and go into s.Pattern search line
					state.Pattern = SEARCH_LINE_LEFT;
				}
				else
				{
					// as long as we haven't detected in which direction we have to steer,
					// we should still follow the line.
					if (cnt1 < obstacleDriveTime[CURRENT_OBSTACLE]){
						// drive with the same speed than before
						setSpeedAndHandleAngle(OBSTACLE_DRIVE_SPEED);
					}
					else if (cnt1 < obstacleBreakTime[CURRENT_OBSTACLE] + obstacleDriveTime[CURRENT_OBSTACLE])
					{
						// in the first 200 ms we stop the motors completely to break even
						// harder to faster get to the desired speed of 40%. Eventually we
						// could break with negative values as well.
						setSpeedAndHandleAngle(-100);
					}
					else
					{
						setSpeedAndHandleAngle(WAIT_FOR_LANE_CHANGE_SPEED);
					}
				}
				DBG();
				break;
			case RIGHT_LINE:
				led_out(0x1);
				if (state.Sensor.Byte == 0x00)
				{
					// after we lost the line we go into s.Pattern search line
					state.Pattern = SEARCH_LINE_RIGHT;
				}
				else
				{
					// as long as we haven't detected in which direction we have to steer,
					// we should still follow the line.
					if (cnt1 < obstacleDriveTime[CURRENT_OBSTACLE]){
						// drive with the same speed than before
						setSpeedAndHandleAngle(OBSTACLE_DRIVE_SPEED);
					}
					else if (cnt1 < obstacleBreakTime[CURRENT_OBSTACLE] + obstacleDriveTime[CURRENT_OBSTACLE])
					{
						// in the first 200 ms we stop the motors completely to break even
						// harder to faster get to the desired speed of 40%. Eventually we
						// could break with negative values as well.
						setSpeedAndHandleAngle(-100);
					}
					else
					{
						setSpeedAndHandleAngle(WAIT_FOR_LANE_CHANGE_SPEED);
					}
				}
				DBG();
				break;
			case SEARCH_LINE_LEFT:
			{
				handle(-LANE_SWITCH_HANDLE_ANGLE);
				setSpeed(-LANE_SWITCH_HANDLE_ANGLE, LANE_SWITCH_SPEED);

				led_out(0x3);
				if (maskSensorInfo(state.Sensor, MASK2_2).Byte != 0x00)
				{
					// we go into pattern normal trace if the 2 outer most sensors are off and another sensor is on
					// if we would go directly into the normal trace state if the outer most sensor detects the line
					// we would steer from 25° to 45° which is in the wrong direction so we disabled the detection of
					// the outer most sensor. We also disabled the second most outer sensor to prevent the normal trace
					// to enter the line switch state again if the outer most sensor would turn on because it gets another
					// reading.
					state.Pattern = NORMAL_TRACE;
					state.TraceMask = RIGHT_MASK;
					cnt2 = 0;
					obstacleCounter++;
				}
				DBG();
				break;
			}
			case SEARCH_LINE_RIGHT:
			{
				handle(LANE_SWITCH_HANDLE_ANGLE);
				setSpeed(LANE_SWITCH_HANDLE_ANGLE, LANE_SWITCH_SPEED);
				led_out(0x3);
				
				if (maskSensorInfo(state.Sensor, MASK2_2).Byte != 0x00)
				{
					// we go into pattern normal trace if the 2 outer most sensors are off and another sensor is on
					// if we would go directly into the normal trace state if the outer most sensor detects the line
					// we would steer from 25° to 45° which is in the wrong direction so we disabled the detection of
					// the outer most sensor. We also disabled the second most outer sensor to prevent the normal trace
					// to enter the line switch state again if the outer most sensor would turn on because it gets another
					// reading.
					state.Pattern = NORMAL_TRACE;
					state.TraceMask = LEFT_MASK;
					cnt2 = 0;
					obstacleCounter++;
				}
				DBG();
				break;
			}
			default:
				/* If neither, return to standby state */
				motor(0, 0);
				state.Pattern = WAIT_FOR_SWITCH;
				break;
		}
	}
}

void setSpeedAndHandleAngle(int maxSpeed){
	int angle = setHandleAngle();
	setSpeed(angle, maxSpeed);
}

int setHandleAngleWithMask(unsigned char mask){
	SensorInfo maskedSensorResult = maskSensorInfo(state.Sensor, mask);
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
	SensorInfo maskedSensorResult = maskSensorInfo(state.Sensor, state.TraceMask);

	unsigned char speedFactor = 100;
	if(cnt2 < obstacleAccelerationTime[CURRENT_OBSTACLE]){
		speedFactor = 100;
	}else{
		speedFactor = SPEED_FACTOR;
	}

	if(slopedown == 1){
		speedFactor = SLOPE_DOWN_SPEED_FACTOR;
		if(cnt0 > 1500){
			slopedown = 0;
		}
	}

	if(CURRENT_OBSTACLE == 3 && state.Sensor.Byte == 0x00){
		if(slopeup == 0 && cnt0 > 100){
			slopeup = 1;
			cnt0 = 0;
		}else if(slopeup == 1 && ontop == 1){
			slopedown = 1;
			ontop = 0;
			slopeup = 0;
		}
	}else if(CURRENT_OBSTACLE == 3 && state.Sensor.Byte != 0x00 && slopeup == 1){
		ontop = 1;
	}else if (maskedSensorResult.Byte == 0x00)
	{
		if(cnt1 > 1000){
			// after 1 second stop, you wont find the track again ;)
			motor(0,0);
			state.Pattern = WAIT_FOR_LOST_TRACK;
			return;
		}
		// lost track => steer more in the current direction until the track is found again
		// and slow down to prevent crashing into things

//		int handleAngle = 45*trackPosition;
//		handleAngle = interpolateAngle(handleAngle);

		int handleAngle = previousHandleAngle;

		if (abs(handleAngle) > 45)
			handleAngle = 45 * trackPosition;

		if (state.TraceMask == LEFT_MASK)
			state.TraceMask = MASK4_0;
		else if (state.TraceMask == RIGHT_MASK)
			state.TraceMask = MASK0_4;

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
		state.TraceMask = NORMAL_MASK;
	}
	else if (handleAngle < 0)
	{
		state.TraceMask = LEFT_MASK;
		trackPosition = LEFT;
	}
	else if (handleAngle > 0)
	{
		state.TraceMask = RIGHT_MASK;
		trackPosition = RIGHT;
	}

	setSpeed(handleAngle, speedFactor);
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
	static int steeringTable[] = { 0, 3, 8, 12, 28, 36, 45, 55 };

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
int previousMotorSpeedRight = 0;
int previousMotorSpeedLeft = 0;
void setSpeed(int handleAngle, int maxSpeed)
{
	int angleFactor = abs(handleAngle) * abs(maxSpeed) / 45;

	int fasterSpeed = abs(maxSpeed) - angleFactor * ANGLE_SPEED_FACTOR;
	int slowerSpeed = fasterSpeed - (fasterSpeed * (angleFactor / 300.0f));

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
	cnt3++;
	state.SysTime++;
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
