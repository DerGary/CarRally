/*
 * Debug.h
 *
 *  Created on: 05.11.2016
 *      Author: Markus
 */

#ifndef DEBUG_H_
#define DEBUG_H_

#include <stdio.h>
#include <machine.h>
#include "printf_lib.h"

// Bootstrap debug functions. Has to be called before any debug function
// Ibit=1(permit interrupt),IPL=0
// (processor interrupt prior level=0)
#define DBG_INIT() do { init_sci1_printf( SPEED_9600 ); setpsw_i(); } while(0)

// Prints the value of a variable
#define PRINT_I(__X__)	printf("%s:%d: %s = %d\r\n", __FILE__, __LINE__, #__X__, __X__)
#define PRINT_F(__X__)	printf("%s:%d: %s = %f\r\n", __FILE__, __LINE__, #__X__, __X__)
#define PRINT_S(__X__)	printf("%s:%d: %s = %s\r\n", __FILE__, __LINE__, #__X__, __X__)

// Prints a literal like 42 or "Hello"
#define PRINT_L(__X__)  printf("%s:%d: %s\r\n",      __FILE__, __LINE__, #__X__)

// Prints the current file, linenumber and function name
#define PRINT_FX()		printf("%s:%d: %s\r\n",      __FILE__, __LINE__, __func__)

// Prints EXPR, waits for user input and then executes EXPR
#define BREAK(__EXPR__) printf("Break at %s:%d: %s\r\n", __FILE__, __LINE__, #__EXPR__); while (get_sci1(NULL)!=1) ; __EXPR__


#endif /* DEBUG_H_ */
