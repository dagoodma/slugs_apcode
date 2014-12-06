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

/* ==================================================
This is the definition file for the the UCSC AP code
it creates all the common defines, unions, enumerations
and data types.

Code by: Mariano I. Lizarraga
First Revision: Aug 18 2008 @ 17:42
====================================================*/
#ifndef _APDEFINITIONS_H_
#define _APDEFINITIONS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

    // =========== Global Definitions ==========

    // Boolean types
    // ===================
typedef char BOOL;
#define TRUE ((char)1)
#define FALSE ((char)0)

#define SUCCESS ((char)0)
#define FAILURE ((char)-1)

    // Circular Buffer Size
    // ===================
#define BSIZE			1024

    // GPS Circular Buffers
    // ====================
#define MSIZE			180

    // UAV System ID
    // =============
#define SLUGS_SYSTEMID		101
#define SLUGS_COMPID		1

#define GS_SYSTEMID		127
#define GS_COMPID		0

    // Cube Model Used
    // ===============
#define USE_CUBE_16405	 1
    
    // GPS Model Used
#define USE_NOVATEL_GPS  0

    // GPS Header IDs
    // ==============
#define GGAID			1
#define RMCID			2
#define VTGID			3
#define UNKID			254

#define USE_NMEA 		1

    // DMA Maximum Char Sending
    // ========================

#define MAXSEND					109
#define MAXSPI                   200
/*
#define MAXSEND                 512
#define MAXSPI                  512
 * */

    // Maximun Number of WPs and PIDs
#define MAX_NUM_WPS		17
#define ORIGIN_WP_INDEX		MAX_NUM_WPS - 1

    // Age limit of heartbeat to consider that the UAV has lost comm with GC
#define HEARTBEAT_LIMIT 6000  // 30 seconds
#define HEARTBEAT_WARN_LIMIT 3000  // 30 seconds

    // Define log raw data at 100 hz. Comment out to have
    // XYZ data come at 100 Hz instead. COMMENT not Change to 0 (using #ifdef)
    //#define LOGRAW100	1

    // Define diagnostic data at 100 hz. Comment out to have
    // XYZ data come at 100 Hz instead. COMMENT not Change to 0 (using #ifdef)
    //#define DIAG100		1

    // ERROR MESSAGES
    // ==============

    // GPS Fix Types
    enum GPS_FIX {
        GPS_FIX_NONE = 0,
        GPS_FIX_2D = 2,
        GPS_FIX_3D = 3
    };

    // PID EEPROM Error Messages

    enum EEPROM_STATUS {
        EEP_MEMORY_OK,
        EEP_WRITE_FAIL = 10,
        EEP_PAGE_EXP = 20,
        EEP_MEMORY_CORR = 30
    };

    enum WP_PROTOCOL {
        WP_PROT_IDLE,
        WP_PROT_LIST_REQUESTED,
        WP_PROT_NUM_SENT,
        WP_PROT_TX_WP,
        WP_PROT_RX_WP,
        WP_PROT_SENDING_WP_IDLE,
        WP_PROT_GETTING_WP_IDLE
    };

    // TODO remove this
    enum PARAM_INTERFACE {
        PI_IDLE,
        PI_SEND_ALL_PARAM,
        PI_SEND_ONE_PARAM
    };


    // Standard characters used in the parsing of messages
    // ===================================================
#define DOLLAR	36
#define STAR	42
#define CR	13
#define LF                   10
#define AT		64

    // Standard Units
    // ==============
#define KTS2MPS 		0.514444444
#define PI          3.141592653589793

    // Periphereal Configurations
    // ==========================
#define APFCY			40000000

// TODO correct baudrate equations

#define GPSBAUDF		9600//19200//38400
#define GPSBAUDI		9600//38400
#define SYSCLK APFCY
    //UxBRG = ((sysclk/baudrate)/16)-1
#define UCSCAP_UBRGF 	((SYSCLK/GPSBAUDF)/16)-1 //64 for 38400

#define UCSCAP_UBRGI 	((SYSCLK/GPSBAUDI)/16)-1 //64 for 38400

//#define LOGBAUD		115200//57600
//#define LOG_UBRG		21//43
#define RADIO_BAUDRATE		        115200//57600
#define RADIO_BAUDRATE_GENERATOR    21//43

#define LOGGER_BAUDRATE		        57600
#define LOGGER_BAUDRATE_GENERATOR   43

#define CAMERA_BAUDRATE             9600
#define CAMERA_BAUDRATE_GENERATOR   259


    // ifdef switches for debugging and conditional inclusion
    // ======================================================
#define __IN_DSPIC__ 	1 // switch for use in PC

#if __IN_DSPIC__
#ifdef DEBUG
#undef DEBUG
#endif
#else
#define DEBUG 1
#endif

    // Uncomment if there is no magentometer
#define NO_MAGNETO

    // Uncomment to allow full gyro calibration
    //#define DO_FULL_CAL


    // ============= Unions Used for Data Transmission ====
    //Type definitions for standard unions used in sending
    // and receiving data

    typedef union {
        unsigned char chData[2];
        unsigned short usData;
    } tUnsignedShortToChar;

    typedef union {
        unsigned char chData[2];
        short shData;
    } tShortToChar;

    typedef union {
        unsigned char chData[4];
        int inData;
    } tIntToChar;

    typedef union {
        unsigned char chData[4];
        float flData;
        unsigned short shData[2];
    } tFloatToChar;

    typedef union {
        unsigned char chData[8];
        uint16_t shData[4];
        uint64_t liData;
    } t64IntToChar;
    
    
    
    
    //===== LED Macros =======
    
    #define ON      ((unsigned char) 1)
    #define OFF     ((unsigned char) 0)


    #define LED_CTRL_BUSY_TRIS  (TRISAbits.TRISA14)
    #define LED_CTRL_STATUS_TRIS  (TRISAbits.TRISA15)
    #define LED_CTRL_BUSY       (LATAbits.LATA14)
    #define LED_CTRL_STATUS     (LATAbits.LATA15)

    #define LED_CTRL_BUSY_SET(state)    (LED_CTRL_BUSY = (unsigned char)state)
    #define LED_CTRL_STATUS_SET(state)  (LED_CTRL_STATUS = (unsigned char)state)
    #define LED_CTRL_BUSY_TOGGLE()    (LED_CTRL_BUSY = LED_CTRL_BUSY ^ 1)
    #define LED_CTRL_STATUS_TOGGLE()  (LED_CTRL_STATUS = LED_CTRL_STATUS ^ 1)

    #define LED_CTRL_INIT() do { \
                                    LED_CTRL_BUSY_TRIS = 0; \
                                    LED_CTRL_BUSY_SET(OFF); \
                                    LED_CTRL_STATUS_TRIS = 0; \
                                    LED_CTRL_STATUS_SET(OFF); \
                                } while(0)

#ifdef __cplusplus
}
#endif

#endif /* _APDEFINITIONS_H_ */
