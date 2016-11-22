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

	memset(&MsgBuffer, 0, sizeof(MsgBuffer));

	init_sci1_printf( SPEED_9600 );
	setpsw_i();
}

void dbgMsg(char Pattern, char Angle, char SpeedLeft, char SpeedRight, char Sensor, char SensorMask, char MessageByte, char MessageData)
{
	Message* prevMsg = &MsgBuffer[BufferPos];

	if (prevMsg->Pattern != Pattern
	 || prevMsg->Angle	!= Angle
	 || prevMsg->MotorLeft != SpeedLeft
	 || prevMsg->MotorRight != SpeedRight
	 || prevMsg->Sensor.Byte != Sensor
	 || prevMsg->TraceMask != SensorMask
	 || prevMsg->MessageByte != MessageByte
	 || prevMsg->MessageData != MessageData)
	{
		BufferPos = (BufferPos + 1) % MsgBufferSize;
		Message* nextMsg = &MsgBuffer[BufferPos];

		nextMsg->Pattern	= Pattern;
		nextMsg->Angle	= Angle;
		nextMsg->MotorLeft = SpeedLeft;
		nextMsg->MotorRight= SpeedRight;
		nextMsg->Sensor.Byte = Sensor;
		nextMsg->TraceMask = SensorMask;
		nextMsg->MessageByte = MessageByte;
		nextMsg->MessageData = MessageData;
	}
}

void dbglog(Message* newMsg)
{
	Message* prevMsg = &MsgBuffer[BufferPos];

	if (!memcmp(newMsg, prevMsg, sizeof(Message)))
	{
		BufferPos = (BufferPos + 1) % MsgBufferSize;
		Message* nextMsg = &MsgBuffer[BufferPos];

		memcpy(nextMsg, newMsg, sizeof(Message));
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
