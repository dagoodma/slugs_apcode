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

#ifndef _EEPLOADER_H_
#define _EEPLOADER_H_

#ifdef __cplusplus
       extern "C"{
#endif

#include "apDefinitions.h"
#include "mavlinkControlMcu.h"
#include "apUtils.h"
#include <p33fxxxx.h>
#include <string.h>

#include "DEE.h"


// EEPROM Emulation Values
// =======================
#define PARAM_OFFSET            0
#define WPS_OFFSET              80
#define WP_SIZE_IN_EEPROM 	8


unsigned char EEPInit (void);
void loadEEPData (void);
uint8_t storeWaypointInEeprom (mavlink_mission_item_t* mlSingleWp);
uint8_t storeParameterInEeprom (float parameter, uint8_t pmIndex);
uint8_t storeAllParamsInEeprom (void);
uint8_t readParamsInEeprom (void);

#ifdef __cplusplus
      }
#endif


#endif /* _EEPLOADER_H_ */
