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
/*======================================*/
/* Symbol definitions                   */
/*======================================*/

typedef union{
	char Byte;
	struct {
		unsigned char b0 : 1;
		unsigned char b1 : 1;
		unsigned char b2 : 1;
		unsigned char b3 : 1;
		unsigned char b4 : 1;
		unsigned char b5 : 1;
		unsigned char b6 : 1;
		unsigned char b7 : 1;
	} Bits;
} SensorInfo;


/* Constant settings */
#define PWM_CYCLE       24575           /* Motor PWM period (16ms)     */
#define CAR_1 2284
#define CAR_2 2291
#define CAR_3 2321
#define CAR_4 2370
#define SERVO_CENTER    CAR_1          /* Servo center value          */
#define HANDLE_STEP     13              /* 1 degree value              */

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

#define LEFT 1
#define RIGHT -1
#define NORMAL 0
/*======================================*/
/* Prototype declarations               */
/*======================================*/
void init(void);
void timer( unsigned long timer_set );
unsigned char sensor_inp( unsigned char mask );
unsigned char startbar_get( void );
unsigned char dipsw_get( void );
unsigned char buttonsw_get( void );
unsigned char pushsw_get( void );
void led_out_m( unsigned char led );
void led_out( unsigned char led );
void motor( int accele_l, int accele_r );
void handle( int angle );
void traceTrack();
void steeringTest();
unsigned char readSensor();
unsigned char readSensorWithMask(unsigned char mask);
int getSteeringAngle(int sensorResult);
SensorInfo readSensorInfo();
SensorInfo readSensorInfoWithMask(unsigned char mask);
/*======================================*/
/* Global variable declarations         */
/*======================================*/
unsigned long   cnt0;
unsigned long   cnt1;
int             pattern;

int ledState;

int currentSteering = NORMAL;
int currentAngle;
int currentSensorResult;
int lastSteeringAdjust;

/* 90° Turn Counter */
unsigned char sharpTurnCounter = 0;
#define TOTAL_SHARP_TURNS 6
#define NUM_SHARP_TURN (sharpTurnCounter % (TOTAL_SHARP_TURNS + 1))

// TODO: Define in another file
int iroundf(float f) {
	return f < 0 ? (int)(f - 0.5f) : (int)(f + 0.5f);
}

float absf(float f) {
	return f < 0 ? -f : f;
}

void emergencyExit(void){
	// TODO: Fix for lane change
	if (!sensor_inp(MASK4_4)) {
		//handle(0);
		motor(0, 0);
		pattern = 2;
	}
}

int currentMask;
int traceMask;

/***********************************************************************/
/* Main program                                                        */
/***********************************************************************/
void main(void)
{
    /* Initialize MCU functions */
    init();
    debug_init();

    /* Initialize micom car state */
    handle( 0 );
    motor( 0, 0 );

    BREAK2();

    while( 1 ) {
    	// If all sensors are off -> emergency exit
//    	emergencyExit();
        switch( pattern ) {

        /****************************************************************
        Pattern-related
         0: wait for switch input
         1: check if start bar is open
         2: Restart if lost track is found again
        11: normal trace
        12: check end of large turn to right
        13: check end of large turn to left
        21: processing at 1st cross line
        22: read but ignore 2nd time
        23: trace, crank detection after cross line
        31: left crank clearing processing ? wait until stable
        32: left crank clearing processing ? check end of turn
        41: right crank clearing processing ? wait until stable
        42: right crank clearing processing ? check end of turn
        51: processing at 1st right half line detection
        52: read but ignore 2nd line
        53: trace after right half line detection
        54: right lane change end check
        61: processing at 1st left half line detection
        62: read but ignore 2nd line
        63: trace after left half line detection
        64: left lane change end check
        ****************************************************************/

        case WAIT_FOR_SWITCH:
            /* Wait for switch input */
        	if( pushsw_get() ) {
                pattern = WAIT_FOR_STARTBAR;
                cnt1 = 0;
                break;
            }
            if( cnt1 < 100 ) {          /* LED flashing processing     */
                led_out( 0x1 );
            } else if( cnt1 < 200 ) {
                led_out( 0x2 );
            } else {
                cnt1 = 0;
            }
            break;

        case WAIT_FOR_STARTBAR:
            /* Check if start bar is open */
            if( !startbar_get() ) {
                /* Start!! */
                led_out( 0x0 );
                pattern = NORMAL_TRACE;
                cnt1 = 0;
                break;
            }
            if( cnt1 < 50 ) {           /* LED flashing processing     */
                led_out( 0x1 );
            } else if( cnt1 < 100 ) {
                led_out( 0x2 );
            } else {
                cnt1 = 0;
            }
            break;

        case WAIT_FOR_LOST_TRACK:
        	if(readSensor() != 0x00){
        		pattern = NORMAL_TRACE;
        	}
        	break;
        case NORMAL_TRACE:
            /* Normal trace */
            if( readSensorWithMask(currentMask) == CROSS_LINE ) {   /* Cross line check during large turn */
                pattern = CROSS_LINE;
                cnt1 = 0;
                break;
            }else if(readSensorWithMask(currentMask) == LEFT_LINE || readSensorWithMask(currentMask) == 0xf0){
            	pattern = WAIT_LEFT_LINE;
            	cnt1 = 0;
            	break;
            }else if(readSensorWithMask(currentMask) == RIGHT_LINE || readSensorWithMask(currentMask) == 0x0f){
				pattern = WAIT_RIGHT_LINE;
				cnt1 = 0;
				break;
            } else {
            	led_out(0x0);
                traceTrack();
            }
        	emergencyExit();
            break;
        case WAIT_LEFT_LINE:
        	timer(10);
			led_out(0x2);
        	if(readSensor() == CROSS_LINE){
        		pattern = CROSS_LINE;
        		cnt1 = 0;
        		break;
        	} else {
        		pattern = LEFT_LINE;
        		cnt1 = 0;
        	}
        	break;
        case WAIT_RIGHT_LINE:
			timer(10);
			led_out(0x2);
			if(readSensor() == CROSS_LINE){
				pattern = CROSS_LINE;
				cnt1 = 0;
				break;
			} else {
				pattern = RIGHT_LINE;
				cnt1 = 0;
			}
			break;
        case CROSS_LINE:
        	/* Processing at 1st cross line */
        	// TODO:
        	// Evaluate sharp turn counter and slow down
        	// according to distance from crossline to turn

            // Center handle and turn off motors


            handle( getSteeringAngle(readSensor()) );
            motor( 40, 40 );
            if(cnt1 > 200){
            	pattern = DETECT_SHARP_CORNER;
            	cnt1 = 0;
                // Count sharp turn
                sharpTurnCounter++;
                break;
            }
            emergencyExit();
            break;
        case DETECT_SHARP_CORNER:
            /* Trace, crank detection after cross line */
            if( sensor_inp(MASK4_0)==0xf0 ) {
                /* Left crank determined -> to left crank clearing processing */

                handle(-46);
                motor( 10 ,60 );
                pattern = SHARP_CORNER_LEFT;
                cnt1 = 0;
                break;
            }
            if( sensor_inp(MASK0_4)==0x0f ) {
                /* Right crank determined -> to right crank clearing processing */

                handle(46);
                motor( 60 ,10 );
                pattern = SHARP_CORNER_RIGHT;
                cnt1 = 0;
                break;
            }
        	handle(getSteeringAngle(readSensor()));
        	if(cnt1 < 200){
        		motor(0,0);
        	}else{
        		motor(40,40);
        	}
        	emergencyExit();
            break;
        case SHARP_CORNER_LEFT:
            /* Left crank clearing processing ? wait until stable */
            handle(-46);
            if(cnt1 < 100){
            	motor(-100,50);
            }else{
            	motor(10, 60);
            }
        	if( readSensorWithMask(MASK2_1) != 0x00 && cnt1 > 200){
            	pattern = NORMAL_TRACE;
            	currentMask = LEFT_MASK;
            }
            break;
        case SHARP_CORNER_RIGHT:
            /* Right crank clearing processing ? wait until stable */
        	handle(46);
        	if(cnt1 < 100){
				motor(50,-100);
			}else{
				motor(60, 10);
			}
        	if( readSensorWithMask(MASK1_2) != 0x00 && cnt1 > 200){
            	pattern = NORMAL_TRACE;
            	currentMask = RIGHT_MASK;
            }
            break;
        case LEFT_LINE:
        	led_out(0x1);
		  if(readSensor() == 0x00){
			handle(-25);
			pattern = SEARCH_LINE_LEFT;
			currentMask = RIGHT_MASK;
			break;
		  }
		  handle( getSteeringAngle(readSensor()) );
		  motor( 40, 40 );
		  break;
	  case RIGHT_LINE:
		  led_out(0x1);
		  if(readSensor() == 0x00){
			handle(25);
			pattern = SEARCH_LINE_RIGHT;
			currentMask = LEFT_MASK;
			break;
		  }
		  handle( getSteeringAngle(readSensor()) );
		  motor( 40, 40 );
		  break;
	  case SEARCH_LINE_LEFT:
	  {
		  SensorInfo result = readSensorInfo();
		  led_out(0x3);
			if(result.Bits.b7 == 0 && result.Bits.b6 == 0 && result.Byte != 0x00){
				pattern = NORMAL_TRACE;
				traceMask = RIGHT_MASK;
				currentMask = RIGHT_MASK;
				break;
			}
			break;
	  }
	  case SEARCH_LINE_RIGHT:
	  {
		  SensorInfo result = readSensorInfo();
		  led_out(0x3);
		  if(result.Bits.b0 == 0 && result.Bits.b1 == 0 && result.Byte != 0x00){
				pattern = NORMAL_TRACE;
				traceMask = LEFT_MASK;
				currentMask = LEFT_MASK;
				break;
			}
			break;
	  }
	  default:
            /* If neither, return to standby state */
		  motor(0,0);
            pattern = WAIT_FOR_SWITCH;
            break;
        }
    }
}



int getSteeringAngle(int sensorResult) {
	int baseSteering;
	switch(sensorResult) {
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


void traceTrack() {
	int unmaskedSensorResult = readSensor();
	int maskedSensorResult = unmaskedSensorResult & traceMask;

	int handleAngle = getSteeringAngle(maskedSensorResult);

	int angleFactor = abs(handleAngle) * 100 / 45;

	int fasterSpeed = 100 - angleFactor * 0.4f;
	int slowerSpeed = fasterSpeed - (fasterSpeed * (angleFactor/200.0f));

	int motorSpeedLeft;
	int motorSpeedRight;
	if (handleAngle < 0) {
		motorSpeedRight = fasterSpeed;
		motorSpeedLeft = slowerSpeed;
	} else {
		motorSpeedRight = slowerSpeed;
		motorSpeedLeft = fasterSpeed;
	}

	int angle = getSteeringAngle(unmaskedSensorResult);
	if(abs(angle) < 5){
		traceMask = NORMAL_MASK;
		currentMask = NORMAL_MASK;
	}else if(angle < 0){
		traceMask = LEFT_MASK;
	}else if(angle > 0){
		traceMask = RIGHT_MASK;
	}

	handle(handleAngle);
	motor(motorSpeedLeft, motorSpeedRight);
}


/***********************************************************************/
/* RX62T Initialization                                                */
/***********************************************************************/
void init(void)
{
    // System Clock
    SYSTEM.SCKCR.BIT.ICK = 0;               //12.288*8=98.304MHz
    SYSTEM.SCKCR.BIT.PCK = 1;               //12.288*4=49.152MHz

    // Port I/O Settings
    PORT1.DDR.BYTE = 0x03;                  //P10:LED2 in motor drive board

    PORT2.DR.BYTE  = 0x08;
    PORT2.DDR.BYTE = 0x1b;                  //P24:SDCARD_CLK(o)
                                            //P23:SDCARD_DI(o)
                                            //P22:SDCARD_DO(i)
                                            //CN:P21-P20
    PORT3.DR.BYTE  = 0x01;
    PORT3.DDR.BYTE = 0x0f;                  //CN:P33-P31
                                            //P30:SDCARD_CS(o)
    //PORT4:input                           //sensor input
    //PORT5:input
    //PORT6:input

    PORT7.DDR.BYTE = 0x7e;                  //P76:LED3 in motor drive board
                                            //P75:forward reverse signal(right motor)
                                            //P74:forward reverse signal(left motor)
                                            //P73:PWM(right motor)
                                            //P72:PWM(left motor)
                                            //P71:PWM(servo motor)
                                            //P70:Push-button in motor drive board
    PORT8.DDR.BYTE = 0x07;                  //CN:P82-P80
    PORT9.DDR.BYTE = 0x7f;                  //CN:P96-P90
    PORTA.DR.BYTE  = 0x0f;                  //CN:PA5-PA4
                                            //PA3:LED3(o)
                                            //PA2:LED2(o)
                                            //PA1:LED1(o)
                                            //PA0:LED0(o)
    PORTA.DDR.BYTE = 0x3f;                  //CN:PA5-PA0
    PORTB.DDR.BYTE = 0xff;                  //CN:PB7-PB0
    PORTD.DDR.BYTE = 0x0f;                  //PD7:TRST#(i)
                                            //PD5:TDI(i)
                                            //PD4:TCK(i)
                                            //PD3:TDO(o)
                                            //CN:PD2-PD0
    PORTE.DDR.BYTE = 0x1b;                  //PE5:SW(i)
                                            //CN:PE4-PE0

    // Compare match timer
    MSTP_CMT0 = 0;                          //CMT Release module stop state
    MSTP_CMT2 = 0;                          //CMT Release module stop state

    ICU.IPR[0x04].BYTE  = 0x0f;             //CMT0_CMI0 Priority of interrupts
    ICU.IER[0x03].BIT.IEN4 = 1;             //CMT0_CMI0 Permission for interrupt
    CMT.CMSTR0.WORD     = 0x0000;           //CMT0,CMT1 Stop counting
    CMT0.CMCR.WORD      = 0x00C3;           //PCLK/512
    CMT0.CMCNT          = 0;
    CMT0.CMCOR          = 96;               //1ms/(1/(49.152MHz/512))
    CMT.CMSTR0.WORD     = 0x0003;           //CMT0,CMT1 Start counting

    // MTU3_3 MTU3_4 PWM mode synchronized by RESET
    MSTP_MTU            = 0;                //Release module stop state
    MTU.TSTRA.BYTE      = 0x00;             //MTU Stop counting

    MTU3.TCR.BYTE   = 0x23;                 //ILCK/64(651.04ns)
    MTU3.TCNT = MTU4.TCNT = 0;              //MTU3,MTU4TCNT clear
    MTU3.TGRA = MTU3.TGRC = PWM_CYCLE;      //cycle(16ms)
    MTU3.TGRB = MTU3.TGRD = SERVO_CENTER;   //PWM(servo motor)
    MTU4.TGRA = MTU4.TGRC = 0;              //PWM(left motor)
    MTU4.TGRB = MTU4.TGRD = 0;              //PWM(right motor)
    MTU.TOCR1A.BYTE = 0x40;                 //Selection of output level
    MTU3.TMDR1.BYTE = 0x38;                 //TGRC,TGRD buffer function
                                            //PWM mode synchronized by RESET
    MTU4.TMDR1.BYTE = 0x00;                 //Set 0 to exclude MTU3 effects
    MTU.TOERA.BYTE  = 0xc7;                 //MTU3TGRB,MTU4TGRA,MTU4TGRB permission for output

    MTU.TSTRA.BYTE  = 0x40;                 //MTU0,MTU3 count function
}

/***********************************************************************/
/* Interrupt                                                           */
/***********************************************************************/
#pragma interrupt Excep_CMT0_CMI0(vect=28)
void Excep_CMT0_CMI0(void)
{
    cnt0++;
    cnt1++;
}

/***********************************************************************/
/* Timer unit                                                          */
/* Arguments: timer value, 1 = 1 ms                                    */
/***********************************************************************/
void timer( unsigned long timer_set )
{
    cnt0 = 0;
    while( cnt0 < timer_set );
}

unsigned char readSensor()
{
    return sensor_inp(MASK4_4);
}
unsigned char readSensorWithMask(unsigned char mask)
{
    return sensor_inp(mask);
}

SensorInfo readSensorInfo()
{
	SensorInfo x = {~PORT4.PORT.BYTE};
	return x;
}
SensorInfo readSensorInfoWithMask(unsigned char mask)
{
	SensorInfo x = {(~PORT4.PORT.BYTE & mask)};
	return x;
}
/***********************************************************************/
/* Sensor state detection                                              */
/* Arguments:       masked values                                      */
/* Return values:   sensor value                                       */
/***********************************************************************/
unsigned char sensor_inp( unsigned char mask )
{
    unsigned char sensor;

    sensor  = ~PORT4.PORT.BYTE;

    sensor &= mask;

    return sensor;
}

/***********************************************************************/
/* Read start bar detection sensor                                     */
/* Return values: Sensor value, ON (bar present):1,                    */
/*                              OFF (no bar present):0                 */
/***********************************************************************/
unsigned char startbar_get( void )
{
    unsigned char b;

    b  = ~PORT4.PORT.BIT.B0 & 0x01;     /* Read start bar signal       */

    return  b;
}

/***********************************************************************/
/* DIP switch value read                                               */
/* Return values: Switch value, 0 to 15                                */
/***********************************************************************/
unsigned char dipsw_get( void )
{
    unsigned char sw,d0,d1,d2,d3;

    d0 = ( PORT6.PORT.BIT.B3 & 0x01 );  /* P63~P60 read                */
    d1 = ( PORT6.PORT.BIT.B2 & 0x01 ) << 1;
    d2 = ( PORT6.PORT.BIT.B1 & 0x01 ) << 2;
    d3 = ( PORT6.PORT.BIT.B0 & 0x01 ) << 3;
    sw = d0 | d1 | d2 | d3;

    return  sw;
}

/***********************************************************************/
/* Push-button in MCU board value read                                 */
/* Return values: Switch value, ON: 1, OFF: 0                          */
/***********************************************************************/
unsigned char buttonsw_get( void )
{
    unsigned char sw;

    sw = ~PORTE.PORT.BIT.B5 & 0x01;     /* Read ports with switches    */

    return  sw;
}

/***********************************************************************/
/* Push-button in motor drive board value read                         */
/* Return values: Switch value, ON: 1, OFF: 0                          */
/***********************************************************************/
unsigned char pushsw_get( void )
{
    unsigned char sw;

    sw  = ~PORT7.PORT.BIT.B0 & 0x01;    /* Read ports with switches    */

    return  sw;
}

/***********************************************************************/
/* LED control in MCU board                                            */
/* Arguments: Switch value, LED0: bit 0, LED1: bit 1. 0: dark, 1: lit  */
/*                                                                     */
/***********************************************************************/
void led_out_m( unsigned char led )
{
    led = ~led;
    PORTA.DR.BYTE = led & 0x0f;
}

/***********************************************************************/
/* LED control in motor drive board                                    */
/* Arguments: Switch value, LED0: bit 0, LED1: bit 1. 0: dark, 1: lit  */
/* Example: 0x3 -> LED1: ON, LED0: ON, 0x2 -> LED1: ON, LED0: OFF      */
/***********************************************************************/
void led_out( unsigned char led )
{
    led = ~led;
    PORT7.DR.BIT.B6 = led & 0x01;
    PORT1.DR.BIT.B0 = ( led >> 1 ) & 0x01;
}

/***********************************************************************/
/* Motor speed control                                                 */
/* Arguments:   Left motor: -100 to 100, Right motor: -100 to 100      */
/*        Here, 0 is stopped, 100 is forward, and -100 is reverse.     */
/* Return value:    None                                               */
/***********************************************************************/
void motor( int accele_l, int accele_r )
{
    int    sw_data;

    sw_data = dipsw_get() + 5;
    accele_l = accele_l * sw_data / 20;
    accele_r = accele_r * sw_data / 20;

    /* Left Motor Control */
    if( accele_l >= 0 ) {
        PORT7.DR.BYTE &= 0xef;
        MTU4.TGRC = (long)( PWM_CYCLE - 1 ) * accele_l / 100;
    } else {
        PORT7.DR.BYTE |= 0x10;
        MTU4.TGRC = (long)( PWM_CYCLE - 1 ) * ( -accele_l ) / 100;
    }

    /* Right Motor Control */
    if( accele_r >= 0 ) {
        PORT7.DR.BYTE &= 0xdf;
        MTU4.TGRD = (long)( PWM_CYCLE - 1 ) * accele_r / 100;
    } else {
        PORT7.DR.BYTE |= 0x20;
        MTU4.TGRD = (long)( PWM_CYCLE - 1 ) * ( -accele_r ) / 100;
    }
}


/***********************************************************************/
/* Servo steering operation                                            */
/* Arguments:   servo operation angle: -90 to 90                       */
/*              -90: 90-degree turn to left, 0: straight,              */
/*               90: 90-degree turn to right                           */
/***********************************************************************/
void handle( int angle )
{
    /* When the servo move from left to right in reverse, replace "-" with "+". */
    MTU3.TGRD = SERVO_CENTER - angle * HANDLE_STEP;
}
/***********************************************************************/
/* end of file                                                         */
/***********************************************************************/
