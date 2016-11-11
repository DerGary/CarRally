/*
 * SensorInfo.h
 *
 *  Created on: Nov 11, 2016
 *      Author: Stefan
 */

#ifndef SENSORINFO_H_
#define SENSORINFO_H_

typedef union
{
	char Byte;
	struct
	{
		unsigned char b0 :1;
		unsigned char b1 :1;
		unsigned char b2 :1;
		unsigned char b3 :1;
		unsigned char b4 :1;
		unsigned char b5 :1;
		unsigned char b6 :1;
		unsigned char b7 :1;
	} Bits;
} SensorInfo;

#endif /* SENSORINFO_H_ */
