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


// EEPROM Data Macros
// =======================
#define PARAM_OFFSET            0
#define MIDLEVEL_OFFSET         160
#define WPS_OFFSET              172  // = 160 + 4*3 = start + floatsize * #mid-level_cmds
#define WP_SIZE_IN_EEPROM 	8

// EEPROM Errors
// =======================
#define EEPROM_ERROR_PAGE_EXPIRED       1
#define EEPROM_ERROR_MEMORY_CORRUPTED   6

/* Note: With 2 EEPROM banks, we have 510 16-bit EEPROM addresses available.
 *       This means that there is space for roughly 79 parameters, 3 mid-level
 *       commands, and 42 waypoints in EEPROM.
 */

int8_t EEPInit (void);
void loadAllEEPData (void); // TODO add error handling to this function
int8_t eraseWaypointsInEeprom(uint8_t startingWp);
int8_t storeWaypointInEeprom (mavlink_mission_item_t* mlSingleWp);
int8_t storeParameterInEeprom (float parameter, uint8_t pmIndex);
int8_t storeAllParamsInEeprom (void);
int8_t storeMidLevelCommandsInEeprom (void);
int8_t readParamsInEeprom (void);
int8_t readWaypointsInEeprom (void);
int8_t readMidLevelCommandsInEeprom (void);

#ifdef __cplusplus
      }
#endif


#endif /* _EEPLOADER_H_ */
