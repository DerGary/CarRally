/*
 * customMath.h
 *
 *  Created on: Nov 11, 2016
 *      Author: Stefan
 */

#ifndef CUSTOMMATH_H_
#define CUSTOMMATH_H_

int iroundf(float f)
{
	return f < 0 ? (int) (f - 0.5f) : (int) (f + 0.5f);
}

float absf(float f)
{
	return f < 0 ? -f : f;
}

#endif /* CUSTOMMATH_H_ */
