/*
 * Debug.c
 *
 *  Created on: 07.11.2016
 *      Author: Markus
 */

#include "debug.h"

void _debugBreakPC(int pc);

void debug_init()
{
	// Bootstrap debug functions. Has to be called before any debug function
	// Ibit=1(permit interrupt),IPL=0
	// (processor interrupt prior level=0)
	setbuf(stdout, NULL);
	init_sci1_printf( SPEED_9600 );
	setpsw_i();
}

void _debugBreak(int pc)
{
	printf("b0x%08X", pc);

	char do_continue = 0;
	while (!do_continue)
	{
		int i = 0;
		char cmd = 0;

		while (!i)
			i = get_sci1(&cmd);

		switch (cmd)
		{
			case 'c':
			{
				do_continue = 1;
				break;
			}
			case 'r':
			{
				printf("%X;%X;%X\n", _read_rx(0), _read_rx(1), _read_rx(2));
				break;
			}
			case 'a':
			{
				//PRINT_L("cmd a");
				char s[9];
				for (int i = 0; i < sizeof(s); i++)
				{
					int r = 0;
					while (!r) r = get_sci1(&s[i]);
				}

				//printf("%s\n", s);

				char* p;
				sscanf(s, "%p", &p);

				char data = *p;
				printf("%02X\n", data);

				break;
			}
		}
	}
}
