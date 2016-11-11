/*
 * CarControl.h
 *
 *  Created on: Nov 11, 2016
 *      Author: Stefan
 */

#ifndef CARCONTROL_H_
#define CARCONTROL_H_
#include <machine.h>
#include "iodefine.h"
#include "stdlib.h"


/* Constant settings */
#define PWM_CYCLE       24575           /* Motor PWM period (16ms)     */
#define CAR_1 2284
#define CAR_2 2291
#define CAR_3 2321
#define CAR_4 2370
#define SERVO_CENTER    CAR_1          /* Servo center value          */
#define HANDLE_STEP     13              /* 1 degree value              */

SensorInfo readSensorInfo()
{
	SensorInfo x = { ~PORT4.PORT.BYTE };
	return x;
}
SensorInfo readSensorInfoWithMask(unsigned char mask)
{
	SensorInfo x = { (~PORT4.PORT.BYTE & mask) };
	return x;
}
SensorInfo maskSensorInfo(SensorInfo info, unsigned char mask)
{
	SensorInfo x = { (info.Byte & mask) };
	return x;
}

/***********************************************************************/
/* Sensor state detection                                              */
/* Arguments:       masked values                                      */
/* Return values:   sensor value                                       */
/***********************************************************************/
unsigned char sensor_inp(unsigned char mask)
{
	unsigned char sensor;

	sensor = ~PORT4.PORT.BYTE;

	sensor &= mask;

	return sensor;
}
unsigned char readSensor(){
	return sensor_inp(0xff);
}

/***********************************************************************/
/* Read start bar detection sensor                                     */
/* Return values: Sensor value, ON (bar present):1,                    */
/*                              OFF (no bar present):0                 */
/***********************************************************************/
unsigned char startbar_get(void)
{
	unsigned char b;

	b = ~PORT4.PORT.BIT.B0 & 0x01; /* Read start bar signal       */

	return b;
}

/***********************************************************************/
/* DIP switch value read                                               */
/* Return values: Switch value, 0 to 15                                */
/***********************************************************************/
unsigned char dipsw_get(void)
{
	unsigned char sw, d0, d1, d2, d3;

	d0 = ( PORT6.PORT.BIT.B3 & 0x01); /* P63~P60 read                */
	d1 = ( PORT6.PORT.BIT.B2 & 0x01) << 1;
	d2 = ( PORT6.PORT.BIT.B1 & 0x01) << 2;
	d3 = ( PORT6.PORT.BIT.B0 & 0x01) << 3;
	sw = d0 | d1 | d2 | d3;

	return sw;
}

/***********************************************************************/
/* Push-button in MCU board value read                                 */
/* Return values: Switch value, ON: 1, OFF: 0                          */
/***********************************************************************/
unsigned char buttonsw_get(void)
{
	unsigned char sw;

	sw = ~PORTE.PORT.BIT.B5 & 0x01; /* Read ports with switches    */

	return sw;
}

/***********************************************************************/
/* Push-button in motor drive board value read                         */
/* Return values: Switch value, ON: 1, OFF: 0                          */
/***********************************************************************/
unsigned char pushsw_get(void)
{
	unsigned char sw;

	sw = ~PORT7.PORT.BIT.B0 & 0x01; /* Read ports with switches    */

	return sw;
}

/***********************************************************************/
/* LED control in MCU board                                            */
/* Arguments: Switch value, LED0: bit 0, LED1: bit 1. 0: dark, 1: lit  */
/*                                                                     */
/***********************************************************************/
void led_out_m(unsigned char led)
{
	led = ~led;
	PORTA.DR.BYTE = led & 0x0f;
}

/***********************************************************************/
/* LED control in motor drive board                                    */
/* Arguments: Switch value, LED0: bit 0, LED1: bit 1. 0: dark, 1: lit  */
/* Example: 0x3 -> LED1: ON, LED0: ON, 0x2 -> LED1: ON, LED0: OFF      */
/***********************************************************************/
void led_out(unsigned char led)
{
	led = ~led;
	PORT7.DR.BIT.B6 = led & 0x01;
	PORT1.DR.BIT.B0 = (led >> 1) & 0x01;
}

/***********************************************************************/
/* Motor speed control                                                 */
/* Arguments:   Left motor: -100 to 100, Right motor: -100 to 100      */
/*        Here, 0 is stopped, 100 is forward, and -100 is reverse.     */
/* Return value:    None                                               */
/***********************************************************************/
void motor(int accele_l, int accele_r)
{
	int sw_data;

	sw_data = dipsw_get() + 5;
	accele_l = accele_l * sw_data / 20;
	accele_r = accele_r * sw_data / 20;

	/* Left Motor Control */
	if (accele_l >= 0)
	{
		PORT7.DR.BYTE &= 0xef;
		MTU4.TGRC = (long) ( PWM_CYCLE - 1) * accele_l / 100;
	}
	else
	{
		PORT7.DR.BYTE |= 0x10;
		MTU4.TGRC = (long) ( PWM_CYCLE - 1) * (-accele_l) / 100;
	}

	/* Right Motor Control */
	if (accele_r >= 0)
	{
		PORT7.DR.BYTE &= 0xdf;
		MTU4.TGRD = (long) ( PWM_CYCLE - 1) * accele_r / 100;
	}
	else
	{
		PORT7.DR.BYTE |= 0x20;
		MTU4.TGRD = (long) ( PWM_CYCLE - 1) * (-accele_r) / 100;
	}
}

/***********************************************************************/
/* Servo steering operation                                            */
/* Arguments:   servo operation angle: -90 to 90                       */
/*              -90: 90-degree turn to left, 0: straight,              */
/*               90: 90-degree turn to right                           */
/***********************************************************************/
void handle(int angle)
{
	/* When the servo move from left to right in reverse, replace "-" with "+". */
	MTU3.TGRD = SERVO_CENTER - angle * HANDLE_STEP;
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

	PORT2.DR.BYTE = 0x08;
	PORT2.DDR.BYTE = 0x1b;                  //P24:SDCARD_CLK(o)
											//P23:SDCARD_DI(o)
											//P22:SDCARD_DO(i)
											//CN:P21-P20
	PORT3.DR.BYTE = 0x01;
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
	PORTA.DR.BYTE = 0x0f;                  //CN:PA5-PA4
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

	ICU.IPR[0x04].BYTE = 0x0f;             //CMT0_CMI0 Priority of interrupts
	ICU.IER[0x03].BIT.IEN4 = 1;             //CMT0_CMI0 Permission for interrupt
	CMT.CMSTR0.WORD = 0x0000;           //CMT0,CMT1 Stop counting
	CMT0.CMCR.WORD = 0x00C3;           //PCLK/512
	CMT0.CMCNT = 0;
	CMT0.CMCOR = 96;               //1ms/(1/(49.152MHz/512))
	CMT.CMSTR0.WORD = 0x0003;           //CMT0,CMT1 Start counting

	// MTU3_3 MTU3_4 PWM mode synchronized by RESET
	MSTP_MTU = 0;                //Release module stop state
	MTU.TSTRA.BYTE = 0x00;             //MTU Stop counting

	MTU3.TCR.BYTE = 0x23;                 //ILCK/64(651.04ns)
	MTU3.TCNT = MTU4.TCNT = 0;              //MTU3,MTU4TCNT clear
	MTU3.TGRA = MTU3.TGRC = PWM_CYCLE;      //cycle(16ms)
	MTU3.TGRB = MTU3.TGRD = SERVO_CENTER;   //PWM(servo motor)
	MTU4.TGRA = MTU4.TGRC = 0;              //PWM(left motor)
	MTU4.TGRB = MTU4.TGRD = 0;              //PWM(right motor)
	MTU.TOCR1A.BYTE = 0x40;                 //Selection of output level
	MTU3.TMDR1.BYTE = 0x38;                 //TGRC,TGRD buffer function
											//PWM mode synchronized by RESET
	MTU4.TMDR1.BYTE = 0x00;                 //Set 0 to exclude MTU3 effects
	MTU.TOERA.BYTE = 0xc7;    //MTU3TGRB,MTU4TGRA,MTU4TGRB permission for output

	MTU.TSTRA.BYTE = 0x40;                 //MTU0,MTU3 count function
}


#endif /* CARCONTROL_H_ */
