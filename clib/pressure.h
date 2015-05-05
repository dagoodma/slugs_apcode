/*
The MIT License

Copyright (c) 2015 UCSC Autonomous Systems Lab

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

#ifndef _PRESSURE_H_
#define _PRESSURE_H_
#ifdef __cplusplus
       extern "C"{
#endif

#include <p33fxxxx.h>
#include <i2c.h>
#include <uart.h>

#include "apDefinitions.h"
#include "apUtils.h"
#include "mavlinkSensorMcu.h"

#ifdef NO_MAGNETO

// Pressure sensor addesses
#define PRESSURE_READ		0x51
#define	PRESSURE_WRITE		0x28

//  I2C State Machine Defines

// Reading State Machine
#define	READ_IDLE		1
#define READ_START		2
#define READ_WAIT               3
#define READ_BRDATA_HI          4
#define READ_BRDATA_LO          5
#define READ_TEMPDATA_HI        6
#define READ_TEMPDATA_LO        7
#define READ_DONE		8
#define	READ_STOP		9

void pressureInit (void);
void startPressureRead (void);
void i2c1Start (void);    	
void i2c1Stop (void);
void i2c1Write (unsigned char byte2write);
void dummyDelay (void);
void getPressure (int16_t* pressureVals);

#ifdef __cplusplus
       }
#endif

#endif

#endif /* _PRESSURE_H_ */
