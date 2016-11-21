/*
 * Debug.c
 *
 *  Created on: 07.11.2016
 *      Author: Markus
 */

#include "debug.h"
#include <string.h>

#define MsgBufferSize 100
static size_t BufferPos = 0;
static Message MsgBuffer[MsgBufferSize];

void _debugBreakPC(int pc);

void debug_init()
{
	// Bootstrap debug functions. Has to be called before any debug function
	// Ibit=1(permit interrupt),IPL=0
	// (processor interrupt prior level=0)
#if !BUFFER_OUTPUT
	setbuf(stdout, NULL);
#endif

	memset(&MsgBuffer, 0, sizeof(Message)*sizeof(MsgBuffer));

	init_sci1_printf( SPEED_9600 );
	setpsw_i();
}

void dbgMsg(char Pattern, char Angle, char SpeedLeft, char SpeedRight, char Sensor, char SensorMask, char MessageByte, char MessageData)
{
	Message msg;

	msg.Pattern	= Pattern;
	msg.Angle	= Angle;
	msg.SpeedLeft = SpeedLeft;
	msg.SpeedRight= SpeedRight;
	msg.Sensor = Sensor;
	msg.SensorMask = SensorMask;
	msg.MessageByte = MessageByte;
	msg.MessageData = MessageData;

	dbglog(msg);
}

void dbglog(Message msg)
{
	Message prevMsg = MsgBuffer[BufferPos];

	char equal =
			   msg.Pattern	== prevMsg.Pattern
			&& msg.Angle	== prevMsg.Angle
			&& msg.SpeedLeft == prevMsg.SpeedLeft
			&& msg.SpeedRight== prevMsg.SpeedRight
			&& msg.Sensor == prevMsg.Sensor
			&& msg.SensorMask == prevMsg.SensorMask
			&& msg.MessageByte == prevMsg.MessageByte
			&& msg.MessageData == prevMsg.MessageData;

	if (!equal)
	{
		BufferPos++;
		if (BufferPos >= MsgBufferSize)
		{
			BufferPos = 0;
		}

		MsgBuffer[BufferPos] = msg;
		//fwrite(&msg, sizeof(Message), 1, stdout);
	}
}

void sendDebugBuffer()
{
	fwrite(&MsgBuffer, sizeof(Message), MsgBufferSize, stdout);
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
