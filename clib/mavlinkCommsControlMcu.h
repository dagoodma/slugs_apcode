#ifndef _MAVLINKCOMMSCONTROLMCU_H_
#define _MAVLINKCOMMSCONTROLMCU_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "circBuffer.h"
#include "apDefinitions.h"
#include "mavlinkControlMcu.h"
#include "apUtils.h"
#include "eepLoader.h"

#include <p33fxxxx.h>   
#include <string.h>    
#include <stdio.h>	


#define PROTOCOL_TIMEOUT_TICKS 20    ///< maximum time to wait for pending messages until timeout

// Events that trigger changes in the parameter protocol state machine.
enum PARAM_EVENT {
	PARAM_EVENT_NONE,
	PARAM_EVENT_ENTER_STATE,
	PARAM_EVENT_EXIT_STATE,

	PARAM_EVENT_REQUEST_LIST_RECEIVED,
	PARAM_EVENT_REQUEST_READ_RECEIVED,
	PARAM_EVENT_SET_RECEIVED
};


    void evaluateParameterState(enum PARAM_EVENT event, const void *data);


    void uart2Init(void);
    void send2GS(unsigned char* protData);
    void gsRead(unsigned char* gsChunk);

    void prepareTelemetryMavlink(unsigned char* dataOut);
    void protDecodeMavlink(uint8_t* dataIn);
    void lowRateTelemetryMavlink(unsigned char* dataOut);
    char sendQGCDebugMessage(const char * dbgMessage, char severity, unsigned char* bytesToAdd, char positionStart);
    uint8_t clearWaypointsFrom(uint8_t startingWp);
    void addMessageToSpiOut(mavlink_message_t* msg);

#ifdef __cplusplus
}
#endif


#endif /* _MAVLINKCOMMSCONTROLMCU_H_ */
