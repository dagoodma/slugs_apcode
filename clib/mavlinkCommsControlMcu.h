#ifndef _MAVLINKCOMMSCONTROLMCU_H_
#define _MAVLINKCOMMSCONTROLMCU_H

/**
 * @file
 * This library handles mavlink communication for the SLUGS control DSC.
 *
 * The following communication tasks are handled here: interprocess communication
 * between the control and sensor DSCs, sending and receiving MAVLink messages with
 * the groundstation, logging of telemetry data to an external SD logger module.
 * This module also contains the mission and parameter state machines.
 *
 * @note SLUGS uses two MAV_MODEs: MAV_MODE_MANUAL and MAV_MODE_GUIDED.
 *  MAV_MODE_MANUAL is used when the autopilot control is disabled and RC input
 *  is connected directly to the control surfaces. MAV_MODE_GUIDED is used when 
 *  the autopilot is manipulating the control surfaces.
 * 
 * @note SLUGS uses two MAV_STATEs: MAV_STATE_ACTIVE and MAV_STATE_HILSIM.
 *  MAV_STATE_ACTIVE is used when the sensor DSC is reading real data directly
 *  from the on-board sensors. MAV_STATE_HILSIM is used when the sensor DSC is
 *  reading simulated data from the RS232 port used for hardware-in-the-loop.
 */

#ifdef __cplusplus
extern "C" {
#endif

// **** Include libraries ****
// Standard libraries
#include <string.h>    
#include <stdio.h>	

// Microchip libraries
#include <p33fxxxx.h>   

// User libraries
#include "circBuffer.h"
#include "apDefinitions.h"
#include "mavlinkControlMcu.h"
#include "apUtils.h"
#include "eepLoader.h"

// **** Macros or preprocessor directives ****
#define PROTOCOL_TIMEOUT_TICKS 20    ///< maximum time to wait for pending messages until timeout

// Define a timeout (in units of main timesteps of MavlinkReceive()) for transmitting
// MAVLink messages as part of the PARAMETER and MISSION protocols. Messages will be retransmit
// twice before it's considered hopeless.
#define MAVLINK_RESEND_TIMEOUT 300

// Specify how long between transmitting parameters in a parameter transmission stream.
#define INTRA_PARAM_DELAY 1

//#define DEBUG_MISSION_SM  // comment this out to disable mission state machine
                            // debug messages sent to groundstation

//#define ENABLE_FULL_TELEMETRY // uncomment to send full 38,400 bps telemetry over radio

//#define RECORD_TO_LOGGER // comment this out to disable recording messages to logger
                         // TODO send full telemetry stream when this is disabled
                         //      instead of reduced version.

// **** Datatypes and enums ****
// Mission protocol state machine states
enum MISSION_STATE {
    MISSION_STATE_INACTIVE = 0,

    // States handling transmitting the mission count. It needs to be retransmit twice.
    MISSION_STATE_SEND_MISSION_COUNT,
    MISSION_STATE_MISSION_COUNT_TIMEOUT,
    MISSION_STATE_SEND_MISSION_COUNT2,
    MISSION_STATE_MISSION_COUNT_TIMEOUT2,
    MISSION_STATE_SEND_MISSION_COUNT3,
    MISSION_STATE_MISSION_COUNT_TIMEOUT3,

    // States handling transmitting a mission item. It needs to be retransmit twice.
    MISSION_STATE_SEND_MISSION_ITEM,
    MISSION_STATE_MISSION_ITEM_TIMEOUT,
    MISSION_STATE_SEND_MISSION_ITEM2,
    MISSION_STATE_MISSION_ITEM_TIMEOUT2,
    MISSION_STATE_SEND_MISSION_ITEM3,
    MISSION_STATE_MISSION_ITEM_TIMEOUT3,

    // States handling transmission of the current mission
    MISSION_STATE_SEND_CURRENT,
    MISSION_STATE_CURRENT_TIMEOUT,
    MISSION_STATE_SEND_CURRENT2,
    MISSION_STATE_CURRENT_TIMEOUT2,

    // States handling sending a mission request
    MISSION_STATE_SEND_MISSION_REQUEST,
    MISSION_STATE_MISSION_REQUEST_TIMEOUT,
    MISSION_STATE_SEND_MISSION_REQUEST2,
    MISSION_STATE_MISSION_REQUEST_TIMEOUT2,
    MISSION_STATE_SEND_MISSION_REQUEST3,
    MISSION_STATE_MISSION_REQUEST_TIMEOUT3
};

// Parameter state machine states
enum PARAM_STATE {
    PARAM_STATE_INACTIVE = 0,

    PARAM_STATE_SINGLETON_SEND_VALUE,

    PARAM_STATE_STREAM_SEND_VALUE,
    PARAM_STATE_STREAM_DELAY
};

// Parameter event enums for changes in the parameter protocol state machine.
enum PARAM_EVENT {
	PARAM_EVENT_NONE,
	PARAM_EVENT_ENTER_STATE,
	PARAM_EVENT_EXIT_STATE,

	PARAM_EVENT_REQUEST_LIST_RECEIVED,
	PARAM_EVENT_REQUEST_READ_RECEIVED,
	PARAM_EVENT_SET_RECEIVED
};

// Mission event enums for events in the mission protocol state machine.
enum MISSION_EVENT {
    MISSION_EVENT_NONE = 0,
    MISSION_EVENT_ENTER_STATE,
    MISSION_EVENT_EXIT_STATE,

    // Message reception events
    MISSION_EVENT_COUNT_RECEIVED,
    MISSION_EVENT_ACK_RECEIVED,
    MISSION_EVENT_REQUEST_RECEIVED,
    MISSION_EVENT_REQUEST_LIST_RECEIVED,
    MISSION_EVENT_CLEAR_ALL_RECEIVED,
    MISSION_EVENT_SET_CURRENT_RECEIVED,
    MISSION_EVENT_ITEM_RECEIVED
};

// **** Module-level, global, or external variables ****

// **** Public function prototypes ****
void uart1Init(void);
void uart2Init(void);
void send2GS(unsigned char* protData);
void gsRead(unsigned char* gsChunk);
void copyBufferToDMA1(unsigned char size);

void evaluateParameterState(enum PARAM_EVENT event, const void *data);
void evaluateMissionState(enum MISSION_EVENT event, const void *data);

void prepareLoggerMavlink(unsigned char* dataOut);
void prepareTelemetryMavlink(unsigned char* dataOut);
void protDecodeMavlink(uint8_t* dataIn);
char sendQGCDebugMessage(const char * dbgMessage, char severity, unsigned char* bytesToAdd, char positionStart);
void addMessageToSpiOut(mavlink_message_t* msg);

#ifdef __cplusplus
}
#endif


#endif /* _MAVLINKCOMMSCONTROLMCU_H_ */
