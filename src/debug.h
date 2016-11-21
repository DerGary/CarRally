/*
 * Debug.h
 *
 *  Created on: 05.11.2016
 *      Author: Markus
 */

#ifndef DEBUG_H_
#define DEBUG_H_

#define ENABLE_DEBUG 0
#define BUFFER_OUTPUT 1

#include <stdio.h>
#include <machine.h>
#include "printf_lib.h"

typedef struct
{
	unsigned char Pattern;
	         char Angle;
	         char SpeedLeft;
	         char SpeedRight;
	unsigned char Sensor;
	unsigned char SensorMask;
	unsigned char MessageByte;
	unsigned char MessageData;
		    short EndOfMessage;
} Message;

void debug_init();
void _debugBreak(int pc);

void dbgMsg(char Pattern, char Angle, char SpeedLeft, char SpeedRight, char Sensor, char SensorMask, char MessageByte, char MessageData);
void dbglog(Message msg);

#if ENABLE_DEBUG
// Prints a string like printf
#define PRINT(_FORMAT, ...)    printf("%s:%d: " _FORMAT "\r\n", __FILE__, __LINE__, __VA_ARGS__)
// Prints the value of a variable
#define PRINT_I(_INT)			 printf("%s:%d: %s = %d\r\n", __FILE__, __LINE__, #_INT, _INT)
#define PRINT_F(_FLOAT)			 printf("%s:%d: %s = %f\r\n", __FILE__, __LINE__, #_FLOAT, _FLOAT)
#define PRINT_S(_STRING)	     printf("%s:%d: %s = %s\r\n", __FILE__, __LINE__, #_STRING, _STRING)

// Prints a literal like 42 or "Hello"
#define PRINT_L(_LITERAL)  printf("%s:%d: %s\r\n",      __FILE__, __LINE__, #_LITERAL)

// Prints the current file, linenumber and function name
#define PRINT_FX()		printf("%s:%d: %s\r\n",      __FILE__, __LINE__, __func__)
#else
#define PRINT(_FORMAT, ...) (void)0
#define PRINT_I(_INT)		(void)0
#define PRINT_F(_FLOAT)		(void)0
#define PRINT_S(_STRING)	(void)0
#define PRINT_L(_LITERAL)   (void)0
#define PRINT_FX()		    (void)0
#endif
// Prints EXPR, waits for user input and then executes EXPR
#define BREAK(__EXPR__) printf("Break at %s:%d: %s\r\n", __FILE__, __LINE__, #__EXPR__); while (get_sci1(NULL)!=1) ; __EXPR__
#define BREAK2() _debugBreak(_read_pc())

#pragma inline_asm _read_r0
static int _read_r0()  { MOV.L  R0,  R1 }
#pragma inline_asm _read_r1
static int _read_r1()  { MOV.L  R1,  R1 }
#pragma inline_asm _read_r2
static int _read_r2()  { MOV.L  R2,  R1 }
#pragma inline_asm _read_r3
static int _read_r3()  { MOV.L  R3,  R1 }
#pragma inline_asm _read_r4
static int _read_r4()  { MOV.L  R4,  R1 }
#pragma inline_asm _read_r5
static int _read_r5()  { MOV.L  R5,  R1 }
#pragma inline_asm _read_r6
static int _read_r6()  { MOV.L  R6,  R1 }
#pragma inline_asm _read_r7
static int _read_r7()  { MOV.L  R7,  R1 }
#pragma inline_asm _read_r8
static int _read_r8()  { MOV.L  R8,  R1 }
#pragma inline_asm _read_r9
static int _read_r9()  { MOV.L  R9,  R1 }
#pragma inline_asm _read_r10
static int _read_r10() { MOV.L  R10, R1 }
#pragma inline_asm _read_r11
static int _read_r11() { MOV.L  R11, R1 }
#pragma inline_asm _read_r12
static int _read_r12() { MOV.L  R12, R1 }
#pragma inline_asm _read_r13
static int _read_r13() { MOV.L  R13, R1 }
#pragma inline_asm _read_r14
static int _read_r14() { MOV.L  R14, R1 }
#pragma inline_asm _read_pc
static int _read_pc()  { MVFC   PC,  R1 }

static int _read_rx(unsigned char r)
{
	switch (r) {
		case  0: return _read_r0();
		case  1: return _read_r1();
		case  2: return _read_r2();
		case  3: return _read_r3();
		case  4: return _read_r4();
		case  5: return _read_r5();
		case  6: return _read_r6();
		case  7: return _read_r7();
		case  8: return _read_r8();
		case  9: return _read_r9();
		case 10: return _read_r10();
		case 11: return _read_r11();
		case 12: return _read_r12();
		case 13: return _read_r13();
		case 14: return _read_r14();
		default: return 0;
	}
}

#endif /* DEBUG_H_ */
