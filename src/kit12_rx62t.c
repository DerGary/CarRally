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
#include "iodefine.h"
#include "stdlib.h"
#include "math.h"

/*======================================*/
/* Symbol definitions                   */
/*======================================*/

/* Constant settings */
#define PWM_CYCLE       24575           /* Motor PWM period (16ms)     */
#define CAR_1 2284
#define CAR_2 2291
#define CAR_3 2321
#define CAR_4 2370
#define SERVO_CENTER    CAR_3           /* Servo center value          */
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
#define MASK2_4         0x3f            /* X X 0 0  O O O O            */
#define MASK4_2         0x3f            /* 0 0 0 0  O O X X            */
#define MASK4_4         0xff            /* O O O O  O O O O            */


#define RIGHT_LINE 	0x1f //O O O X  X X X X
#define LEFT_LINE 	0xf8 //X X X X  X O O O
#define CROSS_LINE  0xff //X X X X  X X X X

#define DETECT_SHARP_CORNER 23
#define SHARP_CORNER_LEFT 31
#define SHARP_CORNER_RIGHT 41
#define NORMAL_TRACE 11
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
unsigned char readSensor(void);
int getSteeringAngle(int sensorResult);
/*======================================*/
/* Global variable declarations         */
/*======================================*/
unsigned long   cnt0;
unsigned long   cnt1;
int             pattern;

int ledState;

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

/***********************************************************************/
/* Main program                                                        */
/***********************************************************************/
void main(void)
{
    /* Initialize MCU functions */
    init();

    ledState = 0;

    /* Initialize micom car state */
    handle( 0 );
    motor( 0, 0 );

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

        case 0:
            /* Wait for switch input */
            if( pushsw_get() ) {
                pattern = 1;
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

        case 1:
            /* Check if start bar is open */
            if( !startbar_get() ) {
                /* Start!! */
                led_out( 0x0 );
                motor(100, 100);
                pattern = 11;
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

        case 2:
        	if(readSensor() != 0x00){
        		pattern = 11;
        	}
        	break;
        case NORMAL_TRACE:
            /* Normal trace */
            if( readSensor() == CROSS_LINE ) {   /* Cross line check during large turn */
                pattern = CROSS_LINE;
                cnt1 = 0;
                break;
            } else {
                traceTrack();
            }
        	emergencyExit();
            break;

//        case 12:
//            /* Check end of large turn to right */
//            if( readSensor() == CROSS_LINE ) {   /* Cross line check during large turn */
//                pattern = CROSS_LINE;
//                break;
//            }
//            if( readSensor() == RIGHT_LINE ) {   /* Right half line detection check */
//                pattern = 51;
//                break;
//            }
//            if( readSensor() == LEFT_LINE ) {    /* Left half line detection check */
//                pattern = 61;
//                break;
//            }
//            if( sensor_inp(MASK3_3) == 0x06 ) {
//                pattern = 11;
//            }
//            break;

//        case 13:
//            /* Check end of large turn to left */
//            if( readSensor() == CROSS_LINE ) {   /* Cross line check during large turn */
//                pattern = 21;
//                break;
//            }
//            if( readSensor() == RIGHT_LINE ) {   /* Right half line detection check */
//                pattern = 51;
//                break;
//            }
//            if( readSensor() == LEFT_LINE ) {    /* Left half line detection check */
//                pattern = 61;
//                break;
//            }
//            if( sensor_inp(MASK3_3) == 0x60 ) {
//                pattern = 11;
//            }
//            break;

        case CROSS_LINE:
        	/* Processing at 1st cross line */
        	// TODO:
        	// Evaluate sharp turn counter and slow down
        	// according to distance from crossline to turn

            // Center handle and turn off motors

            led_out( 0x3 );
            handle( getSteeringAngle(readSensor()) );
            motor( 40, 40 );
            if(cnt1 > 200){
            	pattern = DETECT_SHARP_CORNER;
            	cnt1 = 0;
                // Count sharp turn
                sharpTurnCounter++;
            }
            emergencyExit();
            break;
        case DETECT_SHARP_CORNER:
        	handle(getSteeringAngle(readSensor()));
        	motor(40,40);
            /* Trace, crank detection after cross line */
            if( sensor_inp(MASK4_4)==0xf8 ) {
                /* Left crank determined -> to left crank clearing processing */
                led_out( 0x1 );
                handle(-38);
                motor( 10 ,50 );
                pattern = SHARP_CORNER_LEFT;
                cnt1 = 0;
                break;
            }
            if( sensor_inp(MASK4_4)==0x1f ) {
                /* Right crank determined -> to right crank clearing processing */
                led_out( 0x2 );
                handle(38);
                motor( 50 ,10 );
                pattern = SHARP_CORNER_RIGHT;
                cnt1 = 0;
                break;
            }
        	emergencyExit();
            break;
        case SHARP_CORNER_LEFT:
            /* Left crank clearing processing ? wait until stable */
        	handle(-38);
            if( readSensor() == 0x18){
            	pattern = NORMAL_TRACE;
            }
            break;
        case SHARP_CORNER_RIGHT:
            /* Right crank clearing processing ? wait until stable */
        	handle(38);
        	if( readSensor() == 0x18){
            	pattern = NORMAL_TRACE;
            }
            break;
//        case 51:
//            /* Processing at 1st right half line detection */
//            led_out( 0x2 );
//            handle( 0 );
//            motor( 0 ,0 );
//            pattern = 52;
//            cnt1 = 0;
//            break;
//
//        case 52:
//            /* Read but ignore 2nd time */
//            if( cnt1 > 100 ){
//                pattern = 53;
//                cnt1 = 0;
//            }
//            break;
//
//        case 53:
//            /* Trace, lane change after right half line detection */
//            if( sensor_inp(MASK4_4) == 0x00 ) {
//                handle( 15 );
//                motor( 40 ,31 );
//                pattern = 54;
//                cnt1 = 0;
//                break;
//            }
//            switch( sensor_inp(MASK3_3) ) {
//                case 0x00:
//                    /* Center -> straight */
//                    handle( 0 );
//                    motor( 40 ,40 );
//                    break;
//                case 0x04:
//                case 0x06:
//                case 0x07:
//                case 0x03:
//                    /* Left of center -> turn to right */
//                    handle( 8 );
//                    motor( 40 ,35 );
//                    break;
//                case 0x20:
//                case 0x60:
//                case 0xe0:
//                case 0xc0:
//                    /* Right of center -> turn to left */
//                    handle( -8 );
//                    motor( 35 ,40 );
//                    break;
//                default:
//                    break;
//            }
//            break;
//
//        case 54:
//            /* Right lane change end check */
//            if( sensor_inp( MASK4_4 ) == 0x3c ) {
//                led_out( 0x0 );
//                pattern = 11;
//                cnt1 = 0;
//            }
//            break;
//
//        case 61:
//            /* Processing at 1st left half line detection */
//            led_out( 0x1 );
//            handle( 0 );
//            motor( 0 ,0 );
//            pattern = 62;
//            cnt1 = 0;
//            break;
//
//        case 62:
//            /* Read but ignore 2nd time */
//            if( cnt1 > 100 ){
//                pattern = 63;
//                cnt1 = 0;
//            }
//            break;
//
//        case 63:
//            /* Trace, lane change after left half line detection */
//            if( sensor_inp(MASK4_4) == 0x00 ) {
//                handle( -15 );
//                motor( 31 ,40 );
//                pattern = 64;
//                cnt1 = 0;
//                break;
//            }
//            switch( sensor_inp(MASK3_3) ) {
//                case 0x00:
//                    /* Center -> straight */
//                    handle( 0 );
//                    motor( 40 ,40 );
//                    break;
//                case 0x04:
//                case 0x06:
//                case 0x07:
//                case 0x03:
//                    /* Left of center -> turn to right */
//                    handle( 8 );
//                    motor( 40 ,35 );
//                    break;
//                case 0x20:
//                case 0x60:
//                case 0xe0:
//                case 0xc0:
//                    /* Right of center -> turn to left */
//                    handle( -8 );
//                    motor( 35 ,40 );
//                    break;
//                default:
//                    break;
//            }
//            break;
//
//        case 64:
//            /* Left lane change end check */
//            if( sensor_inp( MASK4_4 ) == 0x3c ) {
//                led_out( 0x0 );
//                pattern = 11;
//                cnt1 = 0;
//            }
//            break;

        default:
            /* If neither, return to standby state */
            pattern = 0;
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

		//0b0001 1000
		default: baseSteering = 0; break;
	}
	return baseSteering;
}

void traceTrack() {
	int sensorResult = sensor_inp(MASK4_4);

	int handleAngle = getSteeringAngle(sensorResult);

	int angleFactor = abs(handleAngle) * 100 / 45;

	int fasterSpeed = 100 - angleFactor * 0.5f;
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

unsigned char readSensor(void)
{
    return sensor_inp(MASK4_4);
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
