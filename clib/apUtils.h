/*
The MIT License

Copyright (c) 2009 UCSC Autonomous Systems Lab

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/

#ifndef _APUTILS_H_
#define _APUTILS_H_

#ifdef __cplusplus
       extern "C"{
#endif

#include <math.h>
#include <string.h>
#include "apDefinitions.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include "mavlink.h"
#include <uart.h>
// Uncomment if you want to use sensor
// mcu diagnostic  data. Note that you can only
// use diagnostic data from 1 MCU at a time
// 
//#define USE_SENSOR_MCU_DIAG		1

// Scaling functions for GPS data message
#define INT32_1E7_TO_FLOAT(x)  ((float)x * 0.0000001f)
#define FLOAT_TO_INT32_1E7(x) ((int32_t)(x * 10000000.0f))

// Used for eph, epv, vel, and cog
#define UINT16_1E2_TO_FLOAT(x)  ((float)x * 0.01f)
#define FLOAT_TO_UINT16_1E2(x) ((uint16_t)(x * 100.0f))
//
#define INT32_1E3_TO_FLOAT(x)  ((float)x * 0.001f)
#define FLOAT_TO_INT32_1E3(x) ((int32_t)(x * 1000.0f))


#ifndef M_PI_2
#define M_PI_2 (float)(asin(1))
#endif

#ifndef M_PI
#define M_PI (float)(M_PI_2*2.0f)
#endif


// Trig and Math Functions
float myAtan2 (float num, float denom);
float myPow (float x, float toTheN);
float mySqrt (float x);
float myAbs (float x);
float myAtan (float x);
float myAsin (float x);
float mySin (float x);
float myCos (float x);
float myTan (float x);
float myAcos (float x);
float myExp (float x);
uint8_t isFinite (float s);

void hugeDelay (void);
void smallDelay (void);
void tinyDelay(void);

BOOL hasMode(uint8_t field, uint8_t flag);

// Debug utilities
#ifndef __cplusplus
	void printToUart2 (const char *fmt, ...);
#endif

#ifdef __cplusplus
      }
#endif


#endif /* _APUTILS_H_ */
