// TODO fix volatility, add interrupts for logger TX, write logger function,
//      wrap everything in ifdefs, hook function to interrupt to write to log
// **** Include libraries ****
// Standard libraries
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

// Microchip libraries

// User libraries
#include "mavlinkCommsControlMcu.h"

// **** Macros or preprocessor directives ****

// **** Module-level, global, or external variables ****
// Round-robin telemetry counters (for radio communication)
static uint8_t periodicCounter, responseCounter;

// UART1 circular buffer for logging mavlink messages
#ifdef RECORD_TO_LOGGER
bool loggerReady = false;
static uint8_t loggerCounter; // round-robin log counter
static struct CircBuffer _uart1BufferOut;
static CBRef uart1BufferOut;
unsigned int dma1Buffer[MAXSEND] __attribute__((space(dma))) = {0};
#endif

// UART2 circular and DMA buffers for radio RX messages from groundstation
static volatile struct CircBuffer _uart2BufferIn;
static volatile CBRef uart2BufferIn;
static struct CircBuffer _uart2BufferOut;
static CBRef uart2BufferOut;
unsigned int dma2Buffer[MAXSEND] __attribute__((space(dma))) = {0};

// Temporary and debug variables
mavlink_message_t mavlinkMessageBuffer;
char sw_debug;
char sw_temp[50];
char sw_intTemp;
float fl_temp1, fl_temp2;

// **** Private function prototypes ****
static void _uart1OutputToDMA1(uint16_t size);
static void _uart2OutputToDMA2(uint16_t size);
static void _logTelemetryMavlink(void);

// -- Parameter and mission Messages --
static void _prepareTransmitParameter(uint16_t id);
static void _prepareTransmitCurrentMission(void);
static void _prepareTransmitMissionAck(uint8_t type);
static void _prepareTransmitMissionCount(void);
static void _prepareTransmitMissionItem(uint8_t currentMissionIndex);
static void _prepareTransmitMissionRequest(uint8_t currentMissionIndex);

//-- Outgoing Mavlink Messages --
static uint8_t _prepareGpsMavlink(unsigned char *buf);
static uint8_t _prepareHeartbeatMavlink(unsigned char *buf);
static uint8_t _prepareGpsTimeMavlink(unsigned char *buf);
static uint8_t _prepareVfrHudMavlink(unsigned char *buf);
static uint8_t _prepareSlugsNavigationMavlink(unsigned char *buf);
static uint8_t _prepareCpuLoadMavlink(unsigned char *buf);
static uint8_t _prepareAttitudeMavlink(unsigned char *buf);
static uint8_t _prepareLocalPositionMavlink(unsigned char *buf);
static uint8_t _prepareSystemStatusMavlink(unsigned char *buf);
static uint8_t _prepareDiagnosticMavlink(unsigned char *buf);
static uint8_t _prepareSensorDiagnosticMavlink(unsigned char *buf);
static uint8_t _prepareDataLogMavlink(unsigned char *buf);
static uint8_t _prepareGpsStatusMavlink(unsigned char *buf);
static uint8_t _prepareServoOutputMavlink(unsigned char *buf);
static uint8_t _prepareSensorBiasMavlink(unsigned char *buf);
static uint8_t _prepareRawPressureMavlink(unsigned char *buf);
static uint8_t _prepareScaledPressureMavlink(unsigned char *buf);
static uint8_t _prepareRcChannelsMavlink(unsigned char *buf);
static uint8_t _prepareRawImuMavlink(unsigned char *buf);
static uint8_t _prepareScaledImuMavlink(unsigned char *buf);
static uint8_t _preparePtzStatusMavlink(unsigned char *buf);
static uint8_t _prepareVoltSensorMavlink(unsigned char *buf);
static uint8_t _preparePassthroughMavlink(unsigned char *buf);
static uint8_t _preparePingMavlink(unsigned char *buf);
static uint8_t _prepareParamValueMavlink(unsigned char *buf);
static uint8_t _prepareCommandLongMavlink(unsigned char *buf);
static uint8_t _prepareGpsOriginMavlink(unsigned char *buf);
static uint8_t _prepareMissionCurrentMavlink(unsigned char *buf);
static uint8_t _prepareMissionAckMavlink(unsigned char *buf);
static uint8_t _prepareMissionCountMavlink(unsigned char *buf);
static uint8_t _prepareMissionItemMavlink(unsigned char *buf);
static uint8_t _prepareMissionRequestMavlink(unsigned char *buf);
static uint8_t _prepareCommandAckMavlink(unsigned char *buf);
static uint8_t _prepareMidLevelCommandsMavlink(unsigned char *buf);
static uint8_t _prepareIsrMavlink(unsigned char *buf);
static uint8_t _prepareStatusTextMavlink(unsigned char *buf);
//-- SPI Messages --
static void _sendSpiSetGpsOriginMavlink(void);
static void _sendSpiServoOutputMavlink(void);

// **** Public functions ****
/**
 * Initializes UART1 and DMA1 for logging data to an SD Card.
 */
void uart1Init(void) {
#ifdef RECORD_TO_LOGGER
    // Initialize the circular buffer
    uart1BufferOut = (struct CircBuffer*) &_uart1BufferOut;
    newCircBuffer(uart1BufferOut);

    // DMA1REQ is channel IRQ select register
    DMA1REQ = 12; // IRQ Number for UART1 Transmission (see table 8-1)

    // DMA1PAD is peripheral address register
    DMA1PAD = (volatile unsigned int) &U1TXREG;

    // DMA1CON is channel control register
    DMA1CONbits.AMODE = 0; // Register Indirect with post-increment
    DMA1CONbits.MODE = 1; // One-shot, No Ping-Pong Mode	
    DMA1CONbits.DIR = 1; // Read from RAM and send to Periphereal
    DMA1CONbits.SIZE = 0; // Word Data Transfer

    // DMA1CNT is transfer count register
    DMA1CNT = MAXSEND; // NOTE should this be 0 to start?

    // DMA1STA is primary start address offset register
    DMA1STA = __builtin_dmaoffset(dma1Buffer);

    // Enable DMA1 TX interrupts
    IFS0bits.DMA1IF = 0; // Clear DMA Interrupt Flag
    IPC3bits.DMA1IP = 6; // interrupt priority to 6
    IEC0bits.DMA1IE = 1; // Enable DMA interrupt

    // Configure and open the port
    // U1MODE is UART settings register
    U1MODEbits.UARTEN = 0; // Disable the port		
    U1MODEbits.USIDL = 0; // Stop on idle
    U1MODEbits.IREN = 0; // No IR decoder
    U1MODEbits.RTSMD = 0; // Ready to send mode (irrelevant)
    U1MODEbits.UEN = 0; // Only RX and TX
    U1MODEbits.WAKE = 1; // Enable at startup
    U1MODEbits.LPBACK = 0; // Disable loopback
    U1MODEbits.ABAUD = 0; // Disable autobaud
    U1MODEbits.URXINV = 0; // Normal operation (high is idle)
    U1MODEbits.PDSEL = 0; // No parity 8 bit
    U1MODEbits.STSEL = 0; // 1 stop bit
    U1MODEbits.BRGH = 0; // Low speed mode

    // U1STA is interrupt status register
    U1STAbits.UTXISEL0 = 0; // generate interrupt on every char
    U1STAbits.UTXISEL1 = 0; // for the DMA	
    U1STAbits.URXISEL = 0; // RX interrupt when a char is in
    U1STAbits.OERR = 0; // clear overun error

    // U1BRG is baudrate generator register
    U1BRG = LOGGER_BAUDRATE_GENERATOR; // Set the baud rate for 57,600

    // Disable UART1 receive interrupts
    //IPC7bits.U1RXIP = 7; // Interrupt priority 7  
    //IFS1bits.U1RXIF = 0; // Clear the interrupt flag
    IEC0bits.U1RXIE = 0; // Disable interrupts

    // Enable the port and interrupts
    U1MODEbits.UARTEN = 1; // Enable the port	
    U1STAbits.UTXEN = 1; // Enable TX

    IEC4bits.U1EIE = 1; // interrupt on UART1 errors

    // Wait for logger
                loggerReady = true;
    /*
    LED_CTRL_BUSY_TOGGLE();
    uint32_t timeoutCounter = 0xffffff;
    uint8_t i = 0;
    uint8_t expect[3] = {'1', '2', '<' };
    char *error[3] = {"UART", "SD card", "system"};

    while  (!loggerReady && timeoutCounter > 0) {
        
        while(U2STAbits.URXDA != 0) {
            uint8_t c = (uint8_t)U2RXREG;
            // If we got the first expected, expect the rest
            if (i > 0) {
                if (expect[i] != c) {
                    mlStatustext.severity = MAV_SEVERITY_ERROR;
                    sprintf(mlStatustext.text, "Failed initializing logger: %s",
                        error[i]);
                    mlPending.statustext++;
                    LED_CTRL_BUSY_TOGGLE();
                    return;
                }
                i++;
            }
            else {
                i += expect[i] == c;
            }

            if (i == 3) {
                loggerReady = true;
            }
        }
        timeoutCounter--;
    }
    if (!loggerReady) {
        prepareTransmitTextMessage("Timed out initializing logger.", MAV_SEVERITY_ERROR);
    }
    LED_CTRL_BUSY_TOGGLE();
*/
#endif
}

/**
 * Initializes UART2 and DMA2 for radio communication.
 */
void uart2Init(void) {

    /* Initialize LEDS */
    // TODO move this to a better init function
    LED_CTRL_INIT();

    sw_debug = 0;

    // Initialize the circular buffers
    uart2BufferIn = (struct CircBuffer*) &_uart2BufferIn;
    newCircBuffer(uart2BufferIn);

    uart2BufferOut = (struct CircBuffer*) &_uart2BufferOut;
    newCircBuffer(uart2BufferOut);


    // DMA2REQ is channel IRQ select register
    DMA2REQ = 31; // IRQ Number for UART2 Transmission (see table 8-1)

    // DMA2PAD is peripheral address register
    DMA2PAD = (volatile unsigned int) &U2TXREG;

    // DMA2CON is channel control register
    DMA2CONbits.AMODE = 0; // Register Indirect with post-increment
    DMA2CONbits.MODE = 1; // One-shot, No Ping-Pong Mode
    DMA2CONbits.DIR = 1; // Read from RAM and send to Periphereal
    DMA2CONbits.SIZE = 0; // Word Data Transfer

    // DMA2CNT is transfer count register
    DMA2CNT = MAXSEND - 1;

    // DMA2STA is primary start address offset register
    DMA2STA = __builtin_dmaoffset(dma2Buffer);

    // Enable DMA2 TX interrupts
    IFS1bits.DMA2IF = 0; // Clear DMA Interrupt Flag
    IPC6bits.DMA2IP = 6; // interrupt priority to 6
    IEC1bits.DMA2IE = 1; // Enable DMA interrupt

    // Configure and open the port
    // U2MODE is UART settings register
    U2MODEbits.UARTEN = 0; // Disable the port		
    U2MODEbits.USIDL = 0; // Stop on idle
    U2MODEbits.IREN = 0; // No IR decoder
    U2MODEbits.RTSMD = 0; // Ready to send mode (irrelevant)
    U2MODEbits.UEN = 0; // Only RX and TX
    U2MODEbits.WAKE = 1; // Enable at startup
    U2MODEbits.LPBACK = 0; // Disable loopback
    U2MODEbits.ABAUD = 0; // Disable autobaud
    U2MODEbits.URXINV = 0; // Normal operation (high is idle)
    U2MODEbits.PDSEL = 0; // No parity 8 bit
    U2MODEbits.STSEL = 0; // 1 stop bit
    U2MODEbits.BRGH = 0; // Low speed mode

    // U2STA is interrupt status register
    U2STAbits.UTXISEL0 = 0; // generate interrupt on every char
    U2STAbits.UTXISEL1 = 0; // for the DMA	
    U2STAbits.URXISEL = 0; // RX interrupt when a char is in
    U2STAbits.OERR = 0; // clear overun error

    // U2BRG is baudrate generator register
    U2BRG = RADIO_BAUDRATE_GENERATOR; // Set the baud rate for 115,200

    // Initialize the Interrupt  
    IPC7bits.U2RXIP = 7; // Interrupt priority 7  
    IFS1bits.U2RXIF = 0; // Clear the interrupt flag
    IEC1bits.U2RXIE = 1; // Enable interrupts

    // Enable the port and interrupts
    U2MODEbits.UARTEN = 1; // Enable the port	
    U2STAbits.UTXEN = 1; // Enable TX

    IEC4bits.U2EIE = 1;
}

/**
 * Send mavlink messages to ground station.
 * @note Called by simulink step function.
 * @param protData is a pointer to the mavlink message buffer
 */
void send2GS(unsigned char* protData) {
    unsigned int bufLen, i;

    // add the data to the circular buffer
    for (i = 1; i <= protData[0]; i += 1) {
        writeBack(uart2BufferOut, protData[i]);
    }

    // get the Length of the logBuffer
    bufLen = getLength(uart2BufferOut);

    // if the interrupt caught up with the circularBuffer
    // and new data was added then turn on the DMA 
    if (!(DMA2CONbits.CHEN) && (bufLen > 0)) {
        // Configure the bytes to send
        DMA2CNT = bufLen <= (MAXSEND - 1) ? bufLen - 1 : MAXSEND - 1;
        // copy the buffer to the DMA channel outgoing buffer
        _uart2OutputToDMA2((unsigned char) DMA2CNT + 1);
        // Enable the DMA
        DMA2CONbits.CHEN = 1;
        // Init the transmission
        DMA2REQbits.FORCE = 1;
    }
}

/**
 * Read radio RX data from groundstation.
 *
 * @param gsChunk Data buffer to read from. First byte is length.
 */
void gsRead(unsigned char* gsChunk) {
    // fix the data length so if the interrupt adds data
    // during execution of this block, it will be read
    // until the next gsRead
    unsigned int tmpLen = getLength(uart2BufferIn), i = 0;

    // if the buffer has more data than the max size, set it to max,
    // otherwise set it to the length
    gsChunk[0] = (tmpLen > MAXSPI) ? MAXSPI : tmpLen;

    // read the data 
    for (i = 1; i <= gsChunk[0]; i += 1) {
        gsChunk[i] = readFront(uart2BufferIn);
    }

    gsChunk[MAXSPI + 1] = 1;
}

/**
 * Send mavlink messages to the logger over UART1.
 * @note Called by _logTelemetryMavlink()
 * @param buf is a pointer to the buffer with messages to send
 * @param bufLen is the length of bytes to send
 */
void send2Logger(unsigned char* buf, uint16_t bufLen) {
    #ifdef RECORD_TO_LOGGER
    if (!loggerReady)
        return;

    unsigned int i;

    // add the data to the circular buffer
    for (i = 0; i < bufLen; i ++) {
        writeBack(uart1BufferOut, buf[i]);
    }

    // get the Length of the logBuffer
    bufLen = getLength(uart1BufferOut);

    // if the interrupt caught up with the circularBuffer
    // and new data was added then turn on the DMA 
    if (!(DMA1CONbits.CHEN) && (bufLen > 0)) {
        // Configure the bytes to send
        DMA1CNT = bufLen <= (MAXSEND) ? bufLen : MAXSEND;
        // copy the buffer to the DMA channel outgoing buffer
        _uart1OutputToDMA1((unsigned char) DMA1CNT);
        // Enable the DMA
        DMA1CONbits.CHEN = 1;
        // Init the transmission
        DMA1REQbits.FORCE = 1;
    }
    #endif
}

/**
 * Prepares to send a command acknowledgement message.
 * @param commandId of the command to acknowledge. see MAV_CMD enum
 * @param result of executing the command. see MAV_RESULT enum
 */
void prepareTransmitCommandAck(uint16_t commandId, uint8_t result) {
    mlPending.commandAck = TRUE;
    mlCommandAck.command = commandId;
    mlCommandAck.result = result;
}

/**
 * Prepares to send an error message.
 * @param message to be sent (must be constant)
 * @param severity see MAV_SEVERITY enum
 * @note Uses mlStatustext. Messages are limited to 50 Hz.
 */
void prepareTransmitTextMessage(const char *message, uint8_t severity) {
    mlStatustext.severity = severity;
    strcpy(mlStatustext.text, message);
    mlPending.statustext++;
}

/**
 * Send a debug message to groundstation using statustext mavlink message.
 * @param dbgMessage is the message to be sent
 * @param severity is the severity of the message (see MAV_SEVERITY enum)
 * @param bytesToAdd is the output stream buffer address
 * @param positionStart is the offset into the bytesToAdd buffer
 */
char sendQGCDebugMessage(const char * dbgMessage, char severity, unsigned char* bytesToAdd, char positionStart) {
    mavlink_message_t msg;
    unsigned char bytes2Send = 0; // size in bytes of the mavlink packed message (return value)

    mavlink_msg_statustext_pack(SLUGS_SYSTEMID,
        SLUGS_COMPID,
        &msg,
        severity,
        dbgMessage);

    bytes2Send = mavlink_msg_to_send_buffer((bytesToAdd + positionStart), &msg);
    return bytes2Send;
}

/**
 * Sends a MAVLink message to the Sensor DSC over SPI.
 * @param msg to send
 */
void addMessageToSpiOut(mavlink_message_t* msg) {
    // mlPending.spiCurrentIndex = 0;
    mlPending.spiTotalData += mavlink_msg_to_send_buffer(
        &(mlPending.spiToSensor[mlPending.spiTotalData]), msg);
}

void prepareTelemetryMavlink(unsigned char* dataOut) {
    // Record to logger first
    #ifdef RECORD_TO_LOGGER
    _logTelemetryMavlink();
    #endif

    // Whether to skip this step of periodic messages (counter not incremented)
    bool skipPeriodicStep = false;

    // Reduced radio telemetry
    /* Reduces frequency of 5 Hz messages to 1 Hz, and only sends QGC essential messages
       instead of the full telemetry. Attitude is reduced from 50 Hz to 2 Hz. */
    #ifdef REDUCE_RADIO_TELEMETRY
    static uint8_t reducedRateCounter = 1;
    skipPeriodicStep = (reducedRateCounter++ > 1);
    reducedRateCounter = (reducedRateCounter > 5)? 1 : reducedRateCounter;
    #endif

    // Contains the total bytes to send via the serial port
    uint8_t bytes2Send = 0;

    // String used to send text messages to QGC console
    char vr_message[50];

    // Periodic messages
    if (!skipPeriodicStep) {
        switch (periodicCounter) {
             case 1: // GPS and Heartbeat
                // == GPS Raw ==
                bytes2Send += _prepareGpsMavlink(dataOut + 1 + bytes2Send);

                // == Heartbeat ==
                bytes2Send += _prepareHeartbeatMavlink(dataOut + 1 + bytes2Send);

                break;

            case 2: // GPS Date Time, (diagnostic, scaled pressure)
                // == GPS Date Time ==
                bytes2Send += _prepareGpsTimeMavlink(dataOut + 1 + bytes2Send);

                #ifndef REDUCE_RADIO_TELEMETRY
                // == Diagnostic ==
                bytes2Send += _prepareDiagnosticMavlink(dataOut + 1 + bytes2Send);

                // == Scaled Pressure ==
                bytes2Send += _prepareScaledPressureMavlink(dataOut + 1 + bytes2Send);

                #endif

                break;

            case 3: // vfr_hud, data log
                // == VFR Hud ==
                bytes2Send += _prepareVfrHudMavlink(dataOut + 1 + bytes2Send);

                #ifndef REDUCE_RADIO_TELEMETRY
                // == Data Log ==
                bytes2Send += _prepareDataLogMavlink(dataOut + 1 + bytes2Send);
                #endif

                break; 

            case 4: // navigation, cpu load, sensor diag
                // == SLUGS Navigation ==
                bytes2Send += _prepareSlugsNavigationMavlink(dataOut + 1 + bytes2Send);

                // == CPU Load ==
                bytes2Send += _prepareCpuLoadMavlink(dataOut + 1 + bytes2Send);

                #ifndef REDUCE_RADIO_TELEMETRY
                // == Sensor Diagnostic ==
                bytes2Send += _prepareSensorDiagnosticMavlink(dataOut + 1 + bytes2Send);
                #endif

                break;

            case 5: // Raw IMU

                #ifndef REDUCE_RADIO_TELEMETRY
                // == Raw IMU ==
                bytes2Send += _prepareRawImuMavlink(dataOut + 1 + bytes2Send);
                #endif

                break;

            case 6: // Local Position, System Status, GPS Status
                // == Local Position (NED) ==
                bytes2Send += _prepareLocalPositionMavlink(dataOut + 1 + bytes2Send);

                // == System Status ==
                bytes2Send += _prepareSystemStatusMavlink(dataOut + 1 + bytes2Send);

                // == GPS Status ==
                bytes2Send += _prepareGpsStatusMavlink(dataOut + 1 + bytes2Send);

                break;

            case 7: // Servo PWM Commands, Biases

                #ifndef REDUCE_RADIO_TELEMETRY
                // == Servo Output ==
                bytes2Send += _prepareServoOutputMavlink(dataOut + 1 + bytes2Send);

                // == Sensor Bias ==
                bytes2Send += _prepareSensorBiasMavlink(dataOut + 1 + bytes2Send);
                #endif

                break;

            case 8:// raw Pressure
                #ifndef REDUCE_RADIO_TELEMETRY
                // == Raw Pressure ==
                bytes2Send += _prepareRawPressureMavlink(dataOut + 1 + bytes2Send);
                #endif

                break;

            case 9: // Manual RC Channels
                #ifndef REDUCE_RADIO_TELEMETRY
                // == RC Channels ==
                bytes2Send += _prepareRcChannelsMavlink(dataOut + 1 + bytes2Send);
                #endif

                break;

            case 10: // Scaled IMU, PTZ data, VI Sensor
                // == Volt Sensor ==
                bytes2Send += _prepareVoltSensorMavlink(dataOut + 1 + bytes2Send);

                #ifndef REDUCE_RADIO_TELEMETRY
                // == Scaled IMU ==
                bytes2Send += _prepareScaledImuMavlink(dataOut + 1 + bytes2Send);
                // == PTZ Status ==
                bytes2Send += _preparePtzStatusMavlink(dataOut + 1 + bytes2Send);
                #endif

                break;

            default:
                break;
        } // switch
        // increment/overflow the periodic counter (for 5 Hz messages)
        periodicCounter = (periodicCounter >= 10) ? 1 : periodicCounter + 1;
    } // if (!skipPerioidicStep)

    // Response messages
    switch (responseCounter) {
        case 1: // Ctrl srfc passthrough
            // == Control Surface Passthrough ==
            if (mlPending.pt == TRUE) {
                bytes2Send += _preparePassthroughMavlink(dataOut + 1 + bytes2Send);
                mlPending.pt = FALSE;
            }
            break;
        case 3: // Ping
            // == Ping ==
            if (mlPending.ping == TRUE) {
                bytes2Send += _preparePingMavlink(dataOut + 1 + bytes2Send);
                mlPending.ping = FALSE;
            }
            break;
        case 5: // Parameter interface
            // = Parameter State Machine =
            evaluateParameterState(PARAM_EVENT_NONE, NULL);

            // == Param Value ==
            /* Set by evaluateParameterState() via _prepareTransmitParameter()
               when we're ready to transmit. */
            if (mlPending.piTransaction == PARAM_TRANSACTION_SEND) {
                bytes2Send += _prepareParamValueMavlink(dataOut + 1 + bytes2Send);

                mlPending.piTransaction = PARAM_TRANSACTION_NONE;
                if (++mlPending.piCurrentParameter >= PAR_PARAM_COUNT)
                    mlPending.piCurrentParameter = 0;
            }
            break;
        case 6: // gps origin
            if (mlPending.miSendOrigin) {
                bytes2Send += _prepareGpsOriginMavlink(dataOut + 1 + bytes2Send);
                mlPending.miSendOrigin = 0;
            }
            break;
        case 7: // servo output, command ack
            // == Servo Output (Raw) ==
            /* In HIL Mode, report PWM commands to Sensor MCU so they are sent
                to the 6DOF model in Simulink. This is not a response per say,
                but we do not want to rate limit it in reduced telemetry mode. */
            if (hasMode(mlHeartbeatLocal.base_mode, MAV_MODE_FLAG_HIL_ENABLED)) {
                _sendSpiServoOutputMavlink();
            }

            // == Command Long ==
            if (mlPending.command != FALSE) {
                bytes2Send += _prepareCommandLongMavlink(dataOut + 1 + bytes2Send);
                mlPending.command = FALSE;
            } 
            break;
        case 8: // mission state machine
            // = Mission State Machine =
            evaluateMissionState(MISSION_EVENT_NONE, NULL);

            if (mlPending.miTransaction != MISSION_TRANSACTION_NONE) {
                // == Mission Current ==
                if (mlPending.miTransaction == MISSION_TRANSACTION_SEND_CURRENT) {
                    bytes2Send += _prepareMissionCurrentMavlink(dataOut + 1 + bytes2Send);
                    mlPending.miTransaction = MISSION_TRANSACTION_NONE;
                }
                // == Mission Ack ==
                else if (mlPending.miTransaction == MISSION_TRANSACTION_SEND_ACK) {
                    bytes2Send += _prepareMissionAckMavlink(dataOut + 1 + bytes2Send);
                    mlPending.miTransaction = MISSION_TRANSACTION_NONE;

                    #ifdef DEBUG_MISSION_SM
                    memset(vr_message,0,sizeof(vr_message));
                    sprintf(vr_message, "Sent ack = %d", mlPending.miAckType);
                    bytes2Send += sendQGCDebugMessage (vr_message, 0, dataOut, bytes2Send+1);
                    #endif
                }
                // == Mission Count ==
                else if (mlPending.miTransaction == MISSION_TRANSACTION_SEND_COUNT) {
                    bytes2Send += _prepareMissionCountMavlink(dataOut + 1 + bytes2Send);
                    mlPending.miTransaction = MISSION_TRANSACTION_NONE;

                    #ifdef DEBUG_MISSION_SM
                    memset(vr_message,0,sizeof(vr_message));
                    sprintf(vr_message, "Sent count = %d", mlPending.miTotalMissions);
                    bytes2Send += sendQGCDebugMessage (vr_message, 0, dataOut, bytes2Send+1);
                    #endif
                }
                // == Mission Item ==
                else if (mlPending.miTransaction == MISSION_TRANSACTION_SEND_ITEM) {
                    bytes2Send += _prepareMissionItemMavlink(dataOut + 1 + bytes2Send);
                    mlPending.miTransaction = MISSION_TRANSACTION_NONE;

                    #ifdef DEBUG_MISSION_SM
                    memset(vr_message,0,sizeof(vr_message));
                    sprintf(vr_message, "Sent item = %d", mlPending.miCurrentMission);
                    bytes2Send += sendQGCDebugMessage (vr_message, 0, dataOut, bytes2Send+1);
                    #endif
                }
                // == Mission Request ==
                else if (mlPending.miTransaction == MISSION_TRANSACTION_SEND_REQUEST
                        && mlPending.miCurrentMission < mlPending.miTotalMissions) {
                    bytes2Send += _prepareMissionRequestMavlink(dataOut + 1 + bytes2Send);
                    mlPending.miTransaction = MISSION_TRANSACTION_NONE;

                    #ifdef DEBUG_MISSION_SM
                    memset(vr_message,0,sizeof(vr_message));
                    sprintf(vr_message, "Sent request = %d", mlPending.miCurrentMission);
                    bytes2Send += sendQGCDebugMessage (vr_message, 0, dataOut, bytes2Send+1);
                    #endif
                }

                periodicCounter = (periodicCounter >= 10) ? 1 : periodicCounter + 1;
            } // if mission SM
            break;
        case 9: // command ack, mid level commands, boot, isr
            // == Command Ack ==
            if (mlPending.commandAck) {
                bytes2Send += _prepareCommandAckMavlink(dataOut + 1 + bytes2Send);
                mlPending.commandAck = FALSE;
            }

            // == Mid Level Commands ==
            if (mlPending.midLvlCmds == TRUE) {
                bytes2Send += _prepareMidLevelCommandsMavlink(dataOut + 1 + bytes2Send);
                mlPending.midLvlCmds = FALSE;
            }
            
            // == Boot ==
            if (mlBoot.version == 1 && !mlPending.statustext) {
                memset(vr_message, 0, sizeof (vr_message));
                sprintf(vr_message, "%s DSC Reboot.", "Control");
                bytes2Send += sendQGCDebugMessage(vr_message, 255, dataOut, bytes2Send + 1);
                mlBoot.version = 0;
            }

            // == ISR Location ==
            if (mlPending.isrLoc == TRUE) {
                bytes2Send += _prepareIsrMavlink(dataOut + 1 + bytes2Send);
                mlPending.isrLoc = FALSE;
            }
            break;
    } // switch(responseCounter)

    // increment/overflow the response counter (for 5 Hz messages)
    responseCounter = (responseCounter >= 10) ? 1 : responseCounter + 1;


    // == GPS Global Origin ==
    // This block goes via SPI to the sensor DSC so it does not count
    // towards the Bandwith budget
    if (mlPending.spiSendOrigin) {
        _sendSpiSetGpsOriginMavlink();
        mlPending.spiSendOrigin = 0;

        memset(vr_message,0,sizeof(vr_message));
        sprintf(vr_message, "Sensor DSC Reboot.");
        bytes2Send += sendQGCDebugMessage(vr_message, 255, dataOut, bytes2Send + 1);

    }

    // == Connection Warning ==
    if (mlPending.connectionWarning && !mlPending.connectionWarningSent) {
         bytes2Send += sendQGCDebugMessage ("Connection failing. Pleaser restart GS radio.",
            0, dataOut, bytes2Send+1);
         mlPending.connectionWarningSent = TRUE;
    }

    // == Status Text ==
    if (mlPending.statustext > 0) {
        bytes2Send += _prepareStatusTextMavlink(dataOut + 1 + bytes2Send);
        mlPending.statustext = 0;
    }

    // == Attitude ==
    /* Reduce rate from 50 Hz to 2 Hz if REDUCE_RADIO_TELEMETRY is set. */
    #ifdef REDUCE_RADIO_TELEMETRY
    if (!skipPeriodicStep && (periodicCounter % 2) == 0) {
    #endif
        bytes2Send += _prepareAttitudeMavlink(dataOut + 1 + bytes2Send);
    #ifdef REDUCE_RADIO_TELEMETRY
    }
    #endif


    // Put the length of the message in the first byte of the outgoing array
    *dataOut = bytes2Send;
}

/**
 * Record mavlink messages to the on-board logger.
 */
void _logTelemetryMavlink(void) {
    #ifdef RECORD_TO_LOGGER
    if (!loggerReady)
        return;

    // Bytes to send
    uint16_t logBytes = 0;
    uint8_t logOut[255];

    switch (loggerCounter) {
         case 1: // GPS and Heartbeat
            // == GPS Raw ==
            logBytes += _prepareGpsMavlink(logOut + logBytes);

            // == Heartbeat ==
            logBytes += _prepareHeartbeatMavlink(logOut + logBytes);

            break;

        case 2: // GPS Date Time, (diagnostic, scaled pressure)
            // == GPS Date Time ==
            logBytes += _prepareGpsTimeMavlink(logOut + logBytes);

            // == Diagnostic ==
            logBytes += _prepareDiagnosticMavlink(logOut + logBytes);

            // == Scaled Pressure ==
            logBytes += _prepareScaledPressureMavlink(logOut + logBytes);

            break;

        case 3: // vfr_hud, data log
            // == VFR Hud ==
            logBytes += _prepareVfrHudMavlink(logOut + logBytes);

            // == Data Log ==
            logBytes += _prepareDataLogMavlink(logOut + logBytes);

            break; 

        case 4: // navigation, cpu load, sensor diag
            // == SLUGS Navigation ==
            logBytes += _prepareSlugsNavigationMavlink(logOut + logBytes);

            // == CPU Load ==
            logBytes += _prepareCpuLoadMavlink(logOut + logBytes);

            // == Sensor Diagnostic ==
            logBytes += _prepareSensorDiagnosticMavlink(logOut + logBytes);

            break;

        case 5: // Raw IMU

            // == Raw IMU ==
            logBytes += _prepareRawImuMavlink(logOut + logBytes);

            break;

        case 6: // Local Position, System Status, GPS Status
            // == Local Position (NED) ==
            logBytes += _prepareLocalPositionMavlink(logOut + logBytes);

            // == System Status ==
            logBytes += _prepareSystemStatusMavlink(logOut + logBytes);

            // == GPS Status ==
            logBytes += _prepareGpsStatusMavlink(logOut + logBytes);

            break;

        case 7: // Servo PWM Commands, Biases

            // == Servo Output ==
            logBytes += _prepareServoOutputMavlink(logOut + logBytes);

            // == Sensor Bias ==
            logBytes += _prepareSensorBiasMavlink(logOut + logBytes);

            break;

        case 8:// raw Pressure

            // == Raw Pressure ==
            logBytes += _prepareRawPressureMavlink(logOut + logBytes);

            break;

        case 9: // Manual RC Channels

            // == RC Channels ==
            logBytes += _prepareRcChannelsMavlink(logOut + logBytes);

            break;

        case 10: // Scaled IMU, PTZ data, VI Sensor

            // == Volt Sensor ==
            logBytes += _prepareVoltSensorMavlink(logOut + logBytes);
            // == Scaled IMU ==
            logBytes += _prepareScaledImuMavlink(logOut + logBytes);
            // == PTZ Status ==
            logBytes += _preparePtzStatusMavlink(logOut + logBytes);

            break;

        default:
            break;
    } // switch

    // == Attitude ==
    logBytes += _prepareAttitudeMavlink(logOut + logBytes);

    // Copy the buffer and start DMA
    send2Logger(logOut, logBytes);

    // increment/overflow the samplePeriod counter
    // configured for 10 Hz in non vital messages
    loggerCounter = (loggerCounter >= 10) ? 1 : loggerCounter + 1;
    #endif
}


/**
 * Decodes incoming MAVLink messages.
 *
 * @note Messages come from either the groundstation or from sensor DSC via IPC
 * @param dataIn buffer with input data
 */ 
void protDecodeMavlink(uint8_t* dataIn) {

    uint8_t i, commChannel = dataIn[MAXSPI + 1];
    int8_t writeResult;
    uint32_t temp;

    // Track if a parameter message was processed in this call. This is used to determine if a
    // NONE_EVENT should be sent to the parameter manager. The manager needs to be called every
    // timestep such that its internal state machine works properly.
    BOOL processedParameterMessage = FALSE;
    BOOL processedMissionMessage = FALSE;

    //static int16_t packet_drops = 0;
    mavlink_message_t msg;
    mavlink_status_t status;

    // increment the age of heartbeat
    mlPending.heartbeatAge++;

    for (i = 1; i <= dataIn[0]; i++) {

        // Try to get a new message
        if (mavlink_parse_char(commChannel, dataIn[i], &msg, &status)) {

            // Handle message
            switch (msg.msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT:
                    mavlink_msg_heartbeat_decode(&msg, &mlHeartbeat);
                    // Reset the heartbeat
                    mlPending.heartbeatAge = 0;
                    mlPending.connectionWarning = 0;
                    mlPending.connectionWarningSent = 0;
                    break;

                case MAVLINK_MSG_ID_GPS_RAW_INT:
                    mavlink_msg_gps_raw_int_decode(&msg, &mlGpsData);
                    //updateGpsPosition(); // scales raw ints to floats
                    break;

                case MAVLINK_MSG_ID_CPU_LOAD:
                    mavlink_msg_cpu_load_decode(&msg, &mlCpuLoadData);
                    // Copy battery voltage to system status message
                    mlSystemStatus.voltage_battery = mlCpuLoadData.batVolt;
                    break;

                case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
                    mavlink_msg_local_position_ned_decode(&msg, &mlLocalPositionData);
                    break;

                case MAVLINK_MSG_ID_SCALED_PRESSURE:
                    mavlink_msg_scaled_pressure_decode(&msg, &mlAirData);
                    break;

                case MAVLINK_MSG_ID_BOOT:
                    temp = mavlink_msg_boot_get_version(&msg);

                    if (temp) {
                        mlPending.spiSendOrigin = 1;
                    }
                    mlPending.sensorDspReady = 1;
                    /*
                    if (temp != 0 && mlBoot.version != 0) {
                        mlBoot.version += temp;
                    } else if (mlBoot.version == 0) {
                        mavlink_msg_boot_decode(&msg, &mlBoot);
                    }
                     */

                    break;

                case MAVLINK_MSG_ID_SENSOR_BIAS:
                    mavlink_msg_sensor_bias_decode(&msg, &mlSensorBiasData);
                    break;

                case MAVLINK_MSG_ID_DIAGNOSTIC:
                    //mavlink_msg_diagnostic_decode(&msg, &mlDiagnosticData);
                    break;

                case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
                    mavlink_msg_rc_channels_raw_decode(&msg, &mlPilotConsoleData);
                    break;

                case MAVLINK_MSG_ID_SCALED_IMU:
                    mavlink_msg_scaled_imu_decode(&msg, &mlFilteredData);
                    break;

                case MAVLINK_MSG_ID_ATTITUDE:
                    mavlink_msg_attitude_decode(&msg, &mlAttitudeData);
                    break;

                case MAVLINK_MSG_ID_RAW_IMU:
                    mavlink_msg_raw_imu_decode(&msg, &mlRawImuData);
                    break;

                case MAVLINK_MSG_ID_RAW_PRESSURE:
                    mavlink_msg_raw_pressure_decode(&msg, &mlRawPressureData);
                    break;

                case MAVLINK_MSG_ID_GPS_DATE_TIME:
                    mavlink_msg_gps_date_time_decode(&msg, &mlGpsDateTime);
                    break;

#if USE_NMEA
                case MAVLINK_MSG_ID_STATUS_GPS:
                    mavlink_msg_status_gps_decode(&msg, &mlGpsStatus);
                    break;
#else
                case MAVLINK_MSG_ID_NOVATEL_DIAG:
                    mavlink_msg_novatel_diag_decode(&msg, &mlNovatelStatus);
                    break;
#endif

                case MAVLINK_MSG_ID_SENSOR_DIAG:
                    mavlink_msg_sensor_diag_decode(&msg, &mlSensorDiag );
                    break;
                    //  End of Sensor MCU exclusive Messages
                    // =====================================

                case MAVLINK_MSG_ID_SET_MODE:
                {
                    mavlink_set_mode_t mlSetMode;
                    mavlink_msg_set_mode_decode(&msg, &mlSetMode );
                    // Can set custom_mode (previously nav_mode) and or base_mode
                    if (mlSetMode.base_mode != 0) {
                        // Turn HIL mode on/off
                        if (!hasMode(mlHeartbeatLocal.base_mode,MAV_MODE_FLAG_HIL_ENABLED)
                            && hasMode(mlSetMode.base_mode, MAV_MODE_FLAG_HIL_ENABLED)) {
                            prepareTransmitTextMessage("Turning on HIL mode.", MAV_SEVERITY_INFO);
                        }
                        else if (hasMode(mlHeartbeatLocal.base_mode, MAV_MODE_FLAG_HIL_ENABLED)
                            && !hasMode(mlSetMode.base_mode,MAV_MODE_FLAG_HIL_ENABLED)) {
                            prepareTransmitTextMessage("Turning off HIL mode.", MAV_SEVERITY_INFO);
                        }
                        mlHeartbeatLocal.base_mode = mlSetMode.base_mode;

                    }
                    if (mlSetMode.custom_mode != SLUGS_MODE_NONE) {
                        BOOL receivedValidMode = TRUE;
                        // Note: unused or unknown modes generate an error message
                        switch (mlSetMode.custom_mode) {
                            case SLUGS_MODE_PASSTHROUGH:
                                mlPending.pt = TRUE; // want passthrough mode
                                break;
                            case SLUGS_MODE_WAYPOINT:
                                break;
                            case SLUGS_MODE_MID_LEVEL:
                                break;
                            case SLUGS_MODE_SELECTIVE_PASSTHROUGH:
                                break;
                            case SLUGS_MODE_ISR:
                                break;
                            /* Unused modes:
                            case SLUGS_MODE_LIFTOFF:
                                break;
                            case SLUGS_MODE_RETURNING:
                                break;
                            case SLUGS_MODE_LANDING:
                                break;
                            case SLUGS_MODE_LOST:
                                break;
                            case SLUGS_MODE_LINE_PATROL:
                                break;
                            */
                            default:
                                receivedValidMode = FALSE;
                                // Send an error message
                                mlPending.statustext++;
                                mlStatustext.severity = MAV_SEVERITY_ERROR;
                                sprintf(mlStatustext.text, "Unknown slugs mode sent: %X.",
                                    (unsigned int)mlSetMode.custom_mode);
                                break;
                        } // switch (mlSetMode.custom_mode)
                        // Set new slugs mode if valid
                        if (receivedValidMode) {
                            mlHeartbeatLocal.custom_mode = mlSetMode.custom_mode;
                        }
                    } // if (mlSetMode.custom_mode != SLUGS_MODE_NONE)
                    break;
                }
                case MAVLINK_MSG_ID_MID_LVL_CMDS:
                {
                    mavlink_msg_mid_lvl_cmds_decode(&msg, &mlMidLevelCommands);
                    // Report the Change
                    prepareTransmitTextMessage( "Mid level flight parameters received.",
                        MAV_SEVERITY_INFO);

                    break;
                }
                // Slugs camera orders not used, but should be moved to command_long
                /*
                case MAVLINK_MSG_ID_SLUGS_CAMERA_ORDER:
                    mavlink_msg_slugs_camera_order_decode(&msg, &mlCameraOrder);

                    // Report the Change
                    mlPending.commandAck++;
                    mlCommandAck.command = MAVLINK_MSG_ID_SLUGS_CAMERA_ORDER;
                    mlCommandAck.result = MAV_RESULT_ACCEPTED;

                    break;

                case MAVLINK_MSG_ID_SLUGS_CONFIGURATION_CAMERA:
                    mavlink_msg_slugs_configuration_camera_decode(&msg, &mlCameraConfig);

                    // Report the Change
                    mlPending.commandAck++;
                    mlCommandAck.command = MAVLINK_MSG_ID_SLUGS_CONFIGURATION_CAMERA;
                    mlCommandAck.result = MAV_RESULT_ACCEPTED;

                    break;
                */
                /*** New mission state machine code. ***/
                // If we are not doing any mission protocol operations, record the size of the incoming mission
                // list and transition into the write missions state machine loop.
                case MAVLINK_MSG_ID_MISSION_COUNT: {
                    uint8_t mavlinkNewMissionListSize = mavlink_msg_mission_count_get_count(&msg);
                    evaluateMissionState(MISSION_EVENT_COUNT_RECEIVED, &mavlinkNewMissionListSize);
                    processedMissionMessage = TRUE;
#ifdef DEBUG_MISSION_SM
                    mlPending.statustext++;
                    mlStatustext.severity = MAV_SEVERITY_INFO;
                    sprintf(mlStatustext.text,"Got mission count %d.", mavlinkNewMissionListSize);
#endif
                    
                } break;
                // Handle receiving a mission.
                case MAVLINK_MSG_ID_MISSION_ITEM: {
                    mavlink_mission_item_t currentMission;
                    mavlink_msg_mission_item_decode(&msg, &currentMission);
                    evaluateMissionState(MISSION_EVENT_ITEM_RECEIVED, &currentMission);
                    processedMissionMessage = TRUE;
#ifdef DEBUG_MISSION_SM
                    mlPending.statustext++;
                    mlStatustext.severity = MAV_SEVERITY_INFO;
                    sprintf(mlStatustext.text,"Got mission item %d.", currentMission.seq);
#endif
                } break;

                /* Responding to a mission request entails moving into the first
                  active state and scheduling a MISSION_COUNT message. Will also
                  schedule a transmission of a GPS_ORIGIN message. This is used
                  for translating global to local coordinates in QGC. 
                 */
                case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: {
                    //MavLinkSendGpsGlobalOrigin();  // TODO determine if we need this
                    mlPending.miSendOrigin = 1;
                    evaluateMissionState(MISSION_EVENT_REQUEST_LIST_RECEIVED, NULL);
                    processedMissionMessage = TRUE;
#ifdef DEBUG_MISSION_SM
                    mlPending.statustext++;
                    mlStatustext.severity = MAV_SEVERITY_INFO;
                    sprintf(mlStatustext.text,"Got mission request list.");
#endif
                } break;

                /* When a mission request message is received, respond with that
                   mission information from the MissionManager.
                 */
                case MAVLINK_MSG_ID_MISSION_REQUEST: {
                    uint8_t receivedMissionIndex = mavlink_msg_mission_request_get_seq(&msg);
                    evaluateMissionState(MISSION_EVENT_REQUEST_RECEIVED, &receivedMissionIndex);
                    processedMissionMessage = TRUE;
#ifdef DEBUG_MISSION_SM
                    mlPending.statustext++;
                    mlStatustext.severity = MAV_SEVERITY_INFO;
                    sprintf(mlStatustext.text,"Got mission request %d.", receivedMissionIndex);
#endif
                } break;

                /* Allow for clearing waypoints. Here we respond simply with an
                   ACK message if we successfully cleared the mission list.
                 */
                case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
                    evaluateMissionState(MISSION_EVENT_CLEAR_ALL_RECEIVED, NULL);
                    processedMissionMessage = TRUE;
                break;

                /* Allow for the groundstation to set the current mission. This
                   requires a WAYPOINT_CURRENT response message agreeing with the
                   received current message index.
                */
                case MAVLINK_MSG_ID_MISSION_SET_CURRENT: {
                    uint8_t newCurrentMission = mavlink_msg_mission_set_current_get_seq(&msg);
                    evaluateMissionState(MISSION_EVENT_SET_CURRENT_RECEIVED, &newCurrentMission);
                    processedMissionMessage = TRUE;
                } break;

                case MAVLINK_MSG_ID_MISSION_ACK: {
                    uint8_t type = mavlink_msg_mission_ack_get_type(&msg);
                    evaluateMissionState(MISSION_EVENT_ACK_RECEIVED, &type);
                    processedMissionMessage = TRUE;
#ifdef DEBUG_MISSION_SM
                    mlPending.statustext++;
                    mlStatustext.severity = MAV_SEVERITY_INFO;
                    sprintf(mlStatustext.text,"Got mission ack=%d", type);
#endif
                } break;
                /*** End of mission state machine ***/
                // Got a new home location
                case MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN:
                    writeResult = SUCCESS;

                    // Forward origin to sensor DSC
                    memset(&mlSingleWp, 0, sizeof (mavlink_mission_item_t));
                    mavlink_msg_set_gps_global_origin_decode(&msg, &mlGSLocation);
                    addMessageToSpiOut(&msg);   

                    // Save origin and record in ROM
                    writeResult = setMissionOrigin(
                            INT32_1E7_TO_FLOAT(mlGSLocation.latitude),
                            INT32_1E7_TO_FLOAT(mlGSLocation.longitude),
                            INT32_1E3_TO_FLOAT(mlGSLocation.altitude));

                    // Respond with origin
                    mlPending.miSendOrigin = 1;

                    // Send a message with result
                    if (writeResult != SUCCESS) {
                        prepareTransmitTextMessage("Failed to write origin to EEPROM.",
                                MAV_SEVERITY_ERROR);
                    }
                    else {
                        prepareTransmitTextMessage("Wrote origin to EEPROM.",
                                MAV_SEVERITY_INFO);
                    }
                    break;

                case MAVLINK_MSG_ID_CTRL_SRFC_PT:
                    mavlink_msg_ctrl_srfc_pt_decode(&msg, &mlPassthrough);
                    mlPending.statustext++;
                    mlStatustext.severity = MAV_SEVERITY_INFO;
                    sprintf(mlStatustext.text, "Control surface passthrough: 0x%X", mlPassthrough.bitfieldPt);
                    break;

                case MAVLINK_MSG_ID_PING:
                    mavlink_msg_ping_decode(&msg, &mlPing);
                    mlPending.ping = 1;
                    break;

                case MAVLINK_MSG_ID_STATUSTEXT:
                    // Received status message from the sensor MCU -- forward to GS
                    mavlink_msg_statustext_decode(&msg, &mlStatustext);
                    mlPending.statustext++;
                    break;
                // Recieved command acknowledgement from sensor DSC
                case MAVLINK_MSG_ID_COMMAND_ACK:
                    mavlink_msg_command_ack_decode(&msg, &mlCommandAck);
                    switch (mlCommandAck.command) {
                        case MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN:
                            prepareTransmitTextMessage("Sensor DSC origin set.",
                                MAV_SEVERITY_INFO);
                            break;
                        default:
                            break;
                    }
                    break;

                case MAVLINK_MSG_ID_COMMAND_LONG:
                    mavlink_msg_command_long_decode(&msg, &mlCommand);
                    switch (mlCommand.command) {
                        // TODO: Only handle this in PREFLIGHT mode
                        case MAV_CMD_PREFLIGHT_STORAGE:
                            writeResult = FAILURE;
                            // Parameter storage
                            if (mlCommand.param1 == 0.0f) { // read
                                memset(&(mlParamInterface.param[0]), 0, sizeof (float) *PAR_PARAM_COUNT);
                                writeResult = readParamsInEeprom();
                            }
                            else if (mlCommand.param1 == 1.0f) { // write
                                writeResult = storeAllParamsInEeprom();
                            }
                            // Waypoint storage (only handle either params or waypoints)
                            // TODO look into implementing this (again?)
                            else if (mlCommand.param2 == 0.0f) { // read
                            }
                            else if (mlCommand.param2 == 1.0f) { // write
                            }
                            
                            prepareTransmitCommandAck(MAV_CMD_PREFLIGHT_STORAGE,
                                ((writeResult == SUCCESS)? MAV_RESULT_ACCEPTED : MAV_RESULT_FAILED)
                            );

                        // Store or read mid-level commands to/from eeprom
                        case MAV_CMD_MIDLEVEL_STORAGE:
                            writeResult = FAILURE;
                            if (mlCommand.param1 == 0.0f) { // read
                                mlMidLevelCommands.hCommand = 0.0f;
                                mlMidLevelCommands.rCommand = 0.0f;
                                mlMidLevelCommands.uCommand = 0.0f;
                                writeResult = readMidLevelCommandsInEeprom();
                            }
                            else if (mlCommand.param1 == 1.0f) { // write
                                writeResult = storeMidLevelCommandsInEeprom();
                                if (writeResult == SUCCESS) {
                                    prepareTransmitTextMessage("Wrote mid-level commands to EEPROM.",
                                        MAV_SEVERITY_INFO);
                                }
                                else {
                                    prepareTransmitTextMessage("Failed to write mid-level commands to EEPROM.",
                                        MAV_SEVERITY_ERROR);
                                }
                            }

                            prepareTransmitCommandAck(MAV_CMD_MIDLEVEL_STORAGE,
                                ((writeResult == SUCCESS)? MAV_RESULT_ACCEPTED : MAV_RESULT_FAILED));
                            break;

                        case MAV_CMD_RETURN_TO_BASE:
                            mlRTB.rtb = TRUE;
                            mlRTB.track_mobile = mlCommand.param1;

                            prepareTransmitCommandAck(MAV_CMD_RETURN_TO_BASE, MAV_RESULT_ACCEPTED);
                            break;

                        case MAV_CMD_TURN_LIGHT:
                            mlLights.state = mlCommand.param2;
                            mlLights.type = mlCommand.param1;

                            prepareTransmitCommandAck(MAV_CMD_TURN_LIGHT, MAV_RESULT_ACCEPTED);
                            break;
                        case MAV_CMD_GET_MID_LEVEL_COMMANDS:
                            mlPending.midLvlCmds = 1;

                            prepareTransmitCommandAck(MAV_CMD_GET_MID_LEVEL_COMMANDS, MAV_RESULT_ACCEPTED);
                            break;

                    } // switch COMMAND_LONG
                    break;

                case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
                    evaluateParameterState(PARAM_EVENT_REQUEST_LIST_RECEIVED, NULL);
                    processedParameterMessage = TRUE;
                } break;

                case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {
                    uint16_t currentParameter = mavlink_msg_param_request_read_get_param_index(&msg);
                    evaluateParameterState(PARAM_EVENT_REQUEST_READ_RECEIVED, &currentParameter);
                    processedParameterMessage = TRUE;
                } break;

                case MAVLINK_MSG_ID_PARAM_SET: {
                    mavlink_param_set_t p;
                    mavlink_msg_param_set_decode(&msg, &p);
                    evaluateParameterState(PARAM_EVENT_SET_RECEIVED, &p);
                    processedParameterMessage = TRUE;
                } break;
                   
                case MAVLINK_MSG_ID_SLUGS_MOBILE_LOCATION:
                    mavlink_msg_slugs_mobile_location_decode(&msg, &mlMobileLocation);
                    break;

                case MAVLINK_MSG_ID_ISR_LOCATION:
                    mavlink_msg_isr_location_decode(&msg, &mlISR);
                    mlPending.isrLoc = 1;

                    prepareTransmitTextMessage("ISR position recieved.", MAV_SEVERITY_INFO);
                    break;
     
            } // switch	
        } // if mavlink message decoded

        // Update global packet drops counter
        if (commChannel == 1) {
            mlSystemStatus.errors_comm += status.packet_rx_drop_count;
        }

    }// for


    // Now if no mission messages were received, trigger the Mission Manager anyways with a NONE
    // event.
    if (!processedMissionMessage) {
        evaluateMissionState(MISSION_EVENT_NONE, NULL);
    }

    // Now if no parameter messages were received, trigger the Parameter Manager anyways with a NONE
    // event.
    if (!processedParameterMessage) {
        evaluateParameterState(PARAM_EVENT_NONE, NULL);
    }
}

/**
 * @param event An event from MISSION_EVENT.
 * @param data A pointer to data, its meaning depends on the current state of the mission protocol.
 * @remark This function implements the mission protocol state machine for the MAVLink protocol.
 * events can be passed as the first argument, or NO_EVENT if desired. data is a pointer
 * to data if there is any to be passed to the state logic. data is not guaranteed to persist
 * beyond the single call to this function.
 *
 * @note Adapted from bwmairs mission state machine code for autoboat:
 *  https://github.com/Susurrus/Autoboat/blob/master/Code/Primary_node/MavlinkGlue.c
 */
void evaluateMissionState(enum MISSION_EVENT event, const void *data) {
    // Internal counter variable for use with the COUNTDOWN state
    static uint16_t counter = 0;

    // Keep track of the expected length of the incoming mission list
    static uint16_t mavlinkNewMissionListSize;

    // Track a mission index for some multi-state loops.
    //static uint8_t currentMissionIndex;

    // Track the state
    static uint8_t state = MISSION_STATE_INACTIVE;

    // Keep track of the next state to transition into
    uint8_t nextState = state;

    // Then check the mission protocol state
    switch (state) {
        case MISSION_STATE_INACTIVE:
            // If a REQUEST_LIST is received, reset the current mission and move into the receive
            // missions mode.
            if (event == MISSION_EVENT_REQUEST_LIST_RECEIVED) {
                sprintf(mlStatustext.text, "WPCount = %d, MiTotal= %d", mlWpValues.wpCount, mlPending.miTotalMissions);
                mlPending.statustext++;
                
                mlPending.miCurrentMission = 0;
                nextState = MISSION_STATE_SEND_MISSION_COUNT;
            }                // Otherwise if a mission count was received, prepare to receive new missions.
            else if (event == MISSION_EVENT_COUNT_RECEIVED) {
                // Don't allow for writing of new missions if we're in autonomous mode.
                if (hasMode(mlHeartbeatLocal.base_mode, MAV_MODE_FLAG_AUTO_ENABLED)
                        && mlHeartbeatLocal.custom_mode == SLUGS_MODE_WAYPOINT) {
                    _prepareTransmitMissionAck(MAV_MISSION_ERROR);
                    nextState = MISSION_STATE_INACTIVE;
                    prepareTransmitTextMessage("Cannot add waypoints in autonomous mode.",
                        MAV_SEVERITY_WARNING);
                    break; // stop handling count received (boat code missing this)
                }
                uint8_t newListSize = *(uint8_t *) data;

                // If we received a 0-length mission list, just respond with a MISSION_ACK error.
                if (newListSize == 0) {
                    _prepareTransmitMissionAck(MAV_MISSION_ERROR);
                    nextState = MISSION_STATE_INACTIVE;
                }                    // If there isn't enough room, respond with a MISSION_ACK error.
                else if (newListSize >= ORIGIN_WP_INDEX) {
                    _prepareTransmitMissionAck(MAV_MISSION_NO_SPACE);
                    nextState = MISSION_STATE_INACTIVE;
                }                    // Otherwise we're set to start retrieving a new mission list so we request the first mission.
                else {
                    // Update the size of the mission list to the new list size.
                    mavlinkNewMissionListSize = newListSize;

                    // Clear all the old waypoints.
                    int8_t result = clearMissionList();
                    if (result != SUCCESS) {
                        mlPending.miEepromError = 1;
                        //prepareTransmitTextMessage("Failed to clear mission list in EEPROM.",
                        //        MAV_SEVERITY_ERROR);
                    }

                    // TODO determine if we need this
                    // Update the starting point to the vehicle's current location
                    //SetStartingPointToCurrentLocation();

                    // And wait for info on the first mission.
                    mlPending.miTotalMissions = newListSize; // must set this or ignores missions
                    mlPending.miCurrentMission = 0;

                    // And finally trigger the proper response.
                    nextState = MISSION_STATE_SEND_MISSION_REQUEST;
                }
            } else if (event == MISSION_EVENT_CLEAR_ALL_RECEIVED) {
                // If we're in autonomous mode, don't allow for clearing the mission list
                if (hasMode(mlHeartbeatLocal.base_mode, MAV_MODE_FLAG_AUTO_ENABLED)
                        && mlHeartbeatLocal.custom_mode == SLUGS_MODE_WAYPOINT) {
                    _prepareTransmitMissionAck(MAV_MISSION_ERROR);
                    nextState = MISSION_STATE_INACTIVE;
                        prepareTransmitTextMessage("Cannot clear waypoints in autonomous mode.",
                                MAV_SEVERITY_WARNING);
                }                    // But if we're in manual mode, go ahead and clear everything.
                else {
                    // Clear the old list
                    int8_t result = clearMissionList();
                    if (result != SUCCESS) {
                        prepareTransmitTextMessage("Failed to clear mission list in EEPROM.",
                                MAV_SEVERITY_ERROR);
                        _prepareTransmitMissionAck(MAV_MISSION_ERROR);
                    }
                    else {
                        // TODO determine if we need this
                        // Update the starting point to the vehicle's current location
                        //SetStartingPointToCurrentLocation();

                        // And then send our acknowledgement.
                        _prepareTransmitMissionAck(MAV_MISSION_ACCEPTED);
                    }
                    nextState = MISSION_STATE_INACTIVE;
                }
            } else if (event == MISSION_EVENT_SET_CURRENT_RECEIVED) {
                setCurrentMission(*(uint8_t*)data);
                _prepareTransmitCurrentMission();
                nextState = MISSION_STATE_INACTIVE;
            }
            break;

        case MISSION_STATE_SEND_MISSION_COUNT:
            if (event == MISSION_EVENT_NONE) {
                _prepareTransmitMissionCount();
                nextState = MISSION_STATE_MISSION_COUNT_TIMEOUT;
            }
            break;

        case MISSION_STATE_MISSION_COUNT_TIMEOUT:
            if (event == MISSION_EVENT_ENTER_STATE) {
                counter = 0;
            } else if (event == MISSION_EVENT_NONE) {
                if (counter++ > MAVLINK_RESEND_TIMEOUT) {
                    nextState = MISSION_STATE_SEND_MISSION_COUNT2;
                }
            } else if (event == MISSION_EVENT_REQUEST_LIST_RECEIVED) {
                nextState = MISSION_STATE_SEND_MISSION_COUNT;
            } else if (event == MISSION_EVENT_REQUEST_RECEIVED) {
                // If the current mission is requested, send it.
                if (data && *(uint8_t *) data == mlPending.miCurrentMission) {
                    _prepareTransmitMissionItem(mlPending.miCurrentMission);
                    nextState = MISSION_STATE_MISSION_ITEM_TIMEOUT;
                } else {
                    _prepareTransmitMissionAck(MAV_MISSION_INVALID_SEQUENCE);
                    nextState = MISSION_STATE_INACTIVE;
                }
            }
            break;

        case MISSION_STATE_SEND_MISSION_COUNT2:
            if (event == MISSION_EVENT_NONE) {
                _prepareTransmitMissionCount();
                nextState = MISSION_STATE_MISSION_COUNT_TIMEOUT2;
            }
            break;

        case MISSION_STATE_MISSION_COUNT_TIMEOUT2:
            if (event == MISSION_EVENT_ENTER_STATE) {
                counter = 0;
            } else if (event == MISSION_EVENT_NONE) {
                if (counter++ > MAVLINK_RESEND_TIMEOUT) {
                    nextState = MISSION_STATE_SEND_MISSION_COUNT3;
                }
            } else if (event == MISSION_EVENT_REQUEST_LIST_RECEIVED) {
                nextState = MISSION_STATE_SEND_MISSION_COUNT;
            } else if (event == MISSION_EVENT_REQUEST_RECEIVED) {
                // If the current mission is requested, send it.
                if (data && *(uint8_t *) data == mlPending.miCurrentMission) {
                    _prepareTransmitMissionItem(mlPending.miCurrentMission);
                    nextState = MISSION_STATE_MISSION_ITEM_TIMEOUT;
                } else {
                    _prepareTransmitMissionAck(MAV_MISSION_INVALID_SEQUENCE);
                    nextState = MISSION_STATE_INACTIVE;
                }
            }
            break;

        case MISSION_STATE_SEND_MISSION_COUNT3:
            if (event == MISSION_EVENT_NONE) {
                _prepareTransmitMissionCount();
                nextState = MISSION_STATE_MISSION_COUNT_TIMEOUT3;
            }
            break;

        case MISSION_STATE_MISSION_COUNT_TIMEOUT3:
            if (event == MISSION_EVENT_ENTER_STATE) {
                counter = 0;
            } else if (event == MISSION_EVENT_NONE) {
                if (counter++ > MAVLINK_RESEND_TIMEOUT) {
                    nextState = MISSION_STATE_INACTIVE;
                }
            } else if (event == MISSION_EVENT_REQUEST_LIST_RECEIVED) {
                nextState = MISSION_STATE_SEND_MISSION_COUNT;
            } else if (event == MISSION_EVENT_REQUEST_RECEIVED) {
                // If the current mission is requested, send it.
                if (data && *(uint8_t *) data == mlPending.miCurrentMission) {
                    _prepareTransmitMissionItem(mlPending.miCurrentMission);
                    nextState = MISSION_STATE_MISSION_ITEM_TIMEOUT;
                } else {
                    _prepareTransmitMissionAck(MAV_MISSION_INVALID_SEQUENCE);
                    nextState = MISSION_STATE_INACTIVE;
                }
            }
            break;

        case MISSION_STATE_SEND_MISSION_ITEM:
        {
            if (event == MISSION_EVENT_NONE) {
                _prepareTransmitMissionItem(mlPending.miCurrentMission);
                nextState = MISSION_STATE_MISSION_ITEM_TIMEOUT2;
            }
        }
        break;

        case MISSION_STATE_MISSION_ITEM_TIMEOUT:
            if (event == MISSION_EVENT_ENTER_STATE) {
                counter = 0;
            } else if (event == MISSION_EVENT_NONE) {
                // Keep track of how long it's taking for a request to be received so we can timeout
                // if necessary.
                if (counter++ > MAVLINK_RESEND_TIMEOUT) {
                    nextState = MISSION_STATE_SEND_MISSION_ITEM2;
                }
            } else if (event == MISSION_EVENT_REQUEST_RECEIVED) {
                // If the mission that was already sent was requested again, retransmit it.
                if (*(uint8_t *) data == mlPending.miCurrentMission) {
                    nextState = MISSION_STATE_SEND_MISSION_ITEM;
                }                    // Otherwise if the next mission was requested, move on to sending that one.
                else if (*(uint8_t *) data == mlPending.miCurrentMission + 1) {
                    ++(mlPending.miCurrentMission);
                    nextState = MISSION_STATE_SEND_MISSION_ITEM;
                } else {
                    _prepareTransmitMissionAck(MAV_MISSION_INVALID_SEQUENCE);
                    nextState = MISSION_STATE_INACTIVE;
                }
            } else if (event == MISSION_EVENT_ACK_RECEIVED) {
                nextState = MISSION_STATE_INACTIVE;
            }
            break;

        case MISSION_STATE_SEND_MISSION_ITEM2:
        {
            if (event == MISSION_EVENT_NONE) {
                _prepareTransmitMissionItem(mlPending.miCurrentMission);
                nextState = MISSION_STATE_MISSION_ITEM_TIMEOUT2;
            }
        }
        break;

        case MISSION_STATE_MISSION_ITEM_TIMEOUT2:
            if (event == MISSION_EVENT_ENTER_STATE) {
                counter = 0;
            } else if (event == MISSION_EVENT_NONE) {
                // Keep track of how long it's taking for a request to be received so we can timeout
                // if necessary.
                if (counter++ > MAVLINK_RESEND_TIMEOUT) {
                    nextState = MISSION_STATE_SEND_MISSION_ITEM3;
                }
            } else if (event == MISSION_EVENT_REQUEST_RECEIVED) {
                // If the mission that was already sent was requested again, retransmit it.
                if (*(uint8_t *) data == mlPending.miCurrentMission) {
                    nextState = MISSION_STATE_SEND_MISSION_ITEM;
                }                    // Otherwise if the next mission was requested, move on to sending that one.
                else if (*(uint8_t *) data == mlPending.miCurrentMission + 1) {
                    ++(mlPending.miCurrentMission);
                    nextState = MISSION_STATE_SEND_MISSION_ITEM;
                } else {
                    _prepareTransmitMissionAck(MAV_MISSION_INVALID_SEQUENCE);
                    nextState = MISSION_STATE_INACTIVE;
                }
            } else if (event == MISSION_EVENT_ACK_RECEIVED) {
                nextState = MISSION_STATE_INACTIVE;
            }
            break;

        case MISSION_STATE_SEND_MISSION_ITEM3:
        {
            if (event == MISSION_EVENT_NONE) {
                _prepareTransmitMissionItem(mlPending.miCurrentMission);
                nextState = MISSION_STATE_MISSION_ITEM_TIMEOUT3;
            }
        }
        break;

        case MISSION_STATE_MISSION_ITEM_TIMEOUT3:
            if (event == MISSION_EVENT_ENTER_STATE) {
                counter = 0;
            } else if (event == MISSION_EVENT_NONE) {
                // Keep track of how long it's taking for a request to be received so we can timeout
                // if necessary.
                if (counter++ > MAVLINK_RESEND_TIMEOUT) {
                    nextState = MISSION_STATE_INACTIVE;
                }
            } else if (event == MISSION_EVENT_REQUEST_RECEIVED) {
                // If the mission that was already sent was requested again, retransmit it.
                if (*(uint8_t *) data == mlPending.miCurrentMission) {
                    nextState = MISSION_STATE_SEND_MISSION_ITEM;
                }                    // Otherwise if the next mission was requested, move on to sending that one.
                else if (*(uint8_t *) data == mlPending.miCurrentMission + 1) {
                    ++(mlPending.miCurrentMission);
                    nextState = MISSION_STATE_SEND_MISSION_ITEM;
                } else {
                    _prepareTransmitMissionAck(MAV_MISSION_INVALID_SEQUENCE);
                    nextState = MISSION_STATE_INACTIVE;
                }
            } else if (event == MISSION_EVENT_ACK_RECEIVED) {
                nextState = MISSION_STATE_INACTIVE;
            }
            break;

        case MISSION_STATE_SEND_MISSION_REQUEST:
        {
            if (event == MISSION_EVENT_NONE) {
                _prepareTransmitMissionRequest(mlPending.miCurrentMission);
                nextState = MISSION_STATE_MISSION_REQUEST_TIMEOUT;
            }
        }
        break;

            // Implement the countdown timer for receiving a mission item
        case MISSION_STATE_MISSION_REQUEST_TIMEOUT:
            if (event == MISSION_EVENT_ENTER_STATE) {
                counter = 0;
            } else if (event == MISSION_EVENT_NONE) {
                // Keep track of how long it's taking for a request to be received so we can timeout
                // if necessary.
                if (counter++ > MAVLINK_RESEND_TIMEOUT) {
                    nextState = MISSION_STATE_SEND_MISSION_REQUEST2;
                }
            }                // If an ACK was received, we just stop. One shouldn't have been received, so just stop
                // for now.
            else if (event == MISSION_EVENT_ACK_RECEIVED) {
                nextState = MISSION_STATE_INACTIVE;
            }                //
            else if (event == MISSION_EVENT_ITEM_RECEIVED) {
                mavlink_mission_item_t *incomingMission = (mavlink_mission_item_t *) data;

                // Make sure that they're coming in in the right order, and if they don't return an error in
                // the acknowledgment response.

                if (mlPending.miCurrentMission == incomingMission->seq) {
                    int8_t missionAddStatus = addMission(incomingMission);
                    // Report error, but keep saving waypoints to RAM
                    if (missionAddStatus != SUCCESS) {
                        mlPending.miEepromError = 1;
                        //prepareTransmitTextMessage("Failed to add mission in EEPROM.",
                        //        MAV_SEVERITY_ERROR);
                    }
                    // If this is going to be the new current mission, then we should set it as such.
                    if (incomingMission->current) {
                        // TODO should this be commented out?
                        //setCurrentMission(incomingMission->seq);
                    }

                    // If this was the last mission we were expecting, respond with an ACK
                    // confirming that we've successfully received the entire mission list.
                    if (mlPending.miCurrentMission == mavlinkNewMissionListSize - 1) {
                        if (mlPending.miEepromError) {
                            mlPending.miEepromError = 0;
                            prepareTransmitTextMessage("Failed to add  mission(s) in EEPROM.",
                                    MAV_SEVERITY_ERROR);
                        }
                        _prepareTransmitMissionAck(MAV_MISSION_ACCEPTED);
                        nextState = MISSION_STATE_INACTIVE;
                    }
                    // Otherwise we just increment and request the next mission.
                    else {
                        mlPending.miCurrentMission++;
                        _prepareTransmitMissionRequest(mlPending.miCurrentMission);
                        nextState = MISSION_STATE_MISSION_REQUEST_TIMEOUT;
                    }
                    //} // If we've run out of space before the last message, respond saying so.
                    /*
                     else {
                        prepareTransmitTextMessage("Failed to add mission in EEPROM.",
                                MAV_SEVERITY_ERROR);
                        _prepareTransmitMissionAck(MAV_MISSION_NO_SPACE);
                        nextState = MISSION_STATE_INACTIVE;
                    }
                    */
                }
            }
            break;

        case MISSION_STATE_SEND_MISSION_REQUEST2:
        {
            if (event == MISSION_EVENT_NONE) {
                _prepareTransmitMissionRequest(mlPending.miCurrentMission);
                nextState = MISSION_STATE_MISSION_REQUEST_TIMEOUT2;
            }
        }
        break;

            // Implement the countdown timer for receiving a mission item
        case MISSION_STATE_MISSION_REQUEST_TIMEOUT2:
            if (event == MISSION_EVENT_ENTER_STATE) {
                counter = 0;
            } else if (event == MISSION_EVENT_NONE) {
                // Keep track of how long it's taking for a request to be received so we can timeout
                // if necessary.
                if (counter++ > MAVLINK_RESEND_TIMEOUT) {
                    nextState = MISSION_STATE_SEND_MISSION_REQUEST3;
                }
            }                // If an ACK was received, we just stop. One shouldn't have been received, so just stop
                // for now.
            else if (event == MISSION_EVENT_ACK_RECEIVED) {
                nextState = MISSION_STATE_INACTIVE;
            }                //
            else if (event == MISSION_EVENT_ITEM_RECEIVED) {
                mavlink_mission_item_t *incomingMission = (mavlink_mission_item_t *) data;

                // Make sure that they're coming in in the right order, and if they don't return an error in
                // the acknowledgment response.
                if (mlPending.miCurrentMission == incomingMission->seq) {
                    int8_t missionAddStatus = addMission(incomingMission);
                    if (missionAddStatus == SUCCESS) {
                        // If this is going to be the new current mission, then we should set it as such.
                        if (incomingMission->current) {
                            // TODO should this be commented out
                            //setCurrentMission(incomingMission->seq);
                        }

                        // If this was the last mission we were expecting, respond with an ACK
                        // confirming that we've successfully received the entire mission list.
                        if (mlPending.miCurrentMission == mavlinkNewMissionListSize - 1) {
                            if (mlPending.miEepromError) {
                                mlPending.miEepromError = 0;
                                prepareTransmitTextMessage("Failed to add  mission(s) in EEPROM.",
                                        MAV_SEVERITY_ERROR);
                            }
                            _prepareTransmitMissionAck(MAV_MISSION_ACCEPTED);
                            nextState = MISSION_STATE_INACTIVE;
                        }                            // Otherwise we just increment and request the next mission.
                        else {
                            ++(mlPending.miCurrentMission);
                            _prepareTransmitMissionRequest(mlPending.miCurrentMission);
                            nextState = MISSION_STATE_MISSION_REQUEST_TIMEOUT;
                        }
                    }
                    // If we've run out of space before the last message, respond saying so.
                    else {
                        _prepareTransmitMissionAck(MAV_MISSION_NO_SPACE);
                        nextState = MISSION_STATE_INACTIVE;
                    }
                }
            }
            break;

        case MISSION_STATE_SEND_MISSION_REQUEST3:
        {
            if (event == MISSION_EVENT_NONE) {
                _prepareTransmitMissionRequest(mlPending.miCurrentMission);
                nextState = MISSION_STATE_MISSION_REQUEST_TIMEOUT3;
            }
        }
        break;

            // Implement the countdown timer for receiving a mission item
        case MISSION_STATE_MISSION_REQUEST_TIMEOUT3:
            if (event == MISSION_EVENT_ENTER_STATE) {
                counter = 0;
            } else if (event == MISSION_EVENT_NONE) {
                // Keep track of how long it's taking for a request to be received so we can timeout
                // if necessary.
                if (counter++ > MAVLINK_RESEND_TIMEOUT) {
                    nextState = MISSION_STATE_INACTIVE;
                }
            }                // If an ACK was received, we just stop. One shouldn't have been received, so just stop
                // for now.
            else if (event == MISSION_EVENT_ACK_RECEIVED) {
                nextState = MISSION_STATE_INACTIVE;
            }                //
            else if (event == MISSION_EVENT_ITEM_RECEIVED) {
                mavlink_mission_item_t *incomingMission = (mavlink_mission_item_t *) data;

                // Make sure that they're coming in in the right order, and if they don't return an error in
                // the acknowledgment response.
                if (mlPending.miCurrentMission == incomingMission->seq) {
                    int8_t missionAddStatus = addMission(incomingMission);
                    if (missionAddStatus == SUCCESS) {
                        // If this is going to be the new current mission, then we should set it as such.
                        if (incomingMission->current) {
                            // TODO should this be commented out?
                            //setCurrentMission(incomingMission->seq);
                        }

                        // If this was the last mission we were expecting, respond with an ACK
                        // confirming that we've successfully received the entire mission list.
                        if (mlPending.miCurrentMission  == mavlinkNewMissionListSize - 1) {
                            if (mlPending.miEepromError) {
                                mlPending.miEepromError = 0;
                                prepareTransmitTextMessage("Failed to add  mission(s) in EEPROM.",
                                        MAV_SEVERITY_ERROR);
                            }
                            _prepareTransmitMissionAck(MAV_MISSION_ACCEPTED);
                            nextState = MISSION_STATE_INACTIVE;
                        }                            // Otherwise we just increment and request the next mission.
                        else {
                            ++(mlPending.miCurrentMission);
                            _prepareTransmitMissionRequest(mlPending.miCurrentMission);
                            nextState = MISSION_STATE_MISSION_REQUEST_TIMEOUT;
                        }
                    }                        // If we've run out of space before the last message, respond saying so.
                    else {
                        _prepareTransmitMissionAck(MAV_MISSION_NO_SPACE);
                        nextState = MISSION_STATE_INACTIVE;
                    }
                }
            }
            break;
    }

    // Here is when we actually transition between states, calling init/exit code as necessary
    if (nextState != state) {
        evaluateMissionState(MISSION_EVENT_EXIT_STATE, NULL);
        state = nextState;
        evaluateMissionState(MISSION_EVENT_ENTER_STATE, NULL);
    }
}

/**
 * @param event An event from PARAM_EVENT.
 * @param data A pointer to data, its meaning depends on the current state of the parameter protocol.
 * @note Adapted from bwmairs param state machine code for autoboat:
 *  https://github.com/Susurrus/Autoboat/blob/master/Code/Primary_node/MavlinkGlue.c
 */
void evaluateParameterState(enum PARAM_EVENT event, const void *data)
{
    // Track the parameter protocol state
    static uint8_t state = PARAM_STATE_INACTIVE;

    // Keep a record of the current parameter being used
    //static uint16_t currentParameter;

    // Used for the delaying parameter transmission
    static uint8_t delayCountdown = 0;

    // Store the state to change into
    uint8_t nextState = state;

    // First check the parameter protocol state
    switch (state) {
        case PARAM_STATE_INACTIVE:
            if (event == PARAM_EVENT_REQUEST_LIST_RECEIVED) {
                mlPending.piCurrentParameter = 0;
                nextState = PARAM_STATE_STREAM_SEND_VALUE;
            }
            else if (event == PARAM_EVENT_SET_RECEIVED) {
                mavlink_param_set_t x = *(mavlink_param_set_t*)data;
                mlPending.piCurrentParameter = setParameterByName(x.param_id, x.param_value);
                // If there was an error, just reset.
                if (mlPending.piCurrentParameter == FAILURE) {
                    mlPending.piCurrentParameter = 0;
                }
                nextState = PARAM_STATE_SINGLETON_SEND_VALUE;
            }
            else if (event == PARAM_EVENT_REQUEST_READ_RECEIVED) {
                mlPending.piCurrentParameter = *(uint16_t*)data;
                nextState = PARAM_STATE_SINGLETON_SEND_VALUE;
            }
            break;

        case PARAM_STATE_SINGLETON_SEND_VALUE: {
            if (event == PARAM_EVENT_NONE) {
                _prepareTransmitParameter(mlPending.piCurrentParameter);
                nextState = PARAM_STATE_INACTIVE;
            }
        } break;

        case PARAM_STATE_STREAM_SEND_VALUE: {
            if (event == PARAM_EVENT_NONE) {
                _prepareTransmitParameter(mlPending.piCurrentParameter);

                // And increment the current parameter index for the next iteration and
                // we finish if we've hit the limit of parameters.
                if ((mlPending.piCurrentParameter) == (PAR_PARAM_COUNT - 1)) {
                    nextState = PARAM_STATE_INACTIVE;
                }
                else {
                    nextState = PARAM_STATE_STREAM_DELAY;
                }
            }
        } break;

            // Add a delay of INTRA_PARAM_DELAY timesteps before attempting to schedule another one
        case PARAM_STATE_STREAM_DELAY: {
            if (event == PARAM_EVENT_ENTER_STATE) {
                delayCountdown = INTRA_PARAM_DELAY;
            }
            else if (event == PARAM_EVENT_NONE) {
                if (delayCountdown-- == 0) {
                    nextState = PARAM_STATE_STREAM_SEND_VALUE;
                }
            }
        } break;

        default: break;
    }

    // Here is when we actually transition between states, calling init/exit code as necessary
    if (nextState != state) {
        evaluateParameterState(PARAM_EVENT_EXIT_STATE, NULL);
        state = nextState;
        evaluateParameterState(PARAM_EVENT_ENTER_STATE, NULL);
    }
}

// **** Private functions ****

/**
 * Queues a parameter to be transmitted.
 * @note This is an internal helper function for the parameter state machine.
 * @param id The ID of this parameter.
 */
static void _prepareTransmitParameter(uint16_t id)
{
    if (id < PAR_PARAM_COUNT) {
        mlPending.piCurrentParameter = id;
        mlPending.piTransaction = PARAM_TRANSACTION_SEND;
    }
    else {
        mlPending.piTransaction = PARAM_TRANSACTION_NONE;
    }
}

/**
 * Queues the current mission number to be sent.
 * @note This is an internal helper function for the mission state machine.
 */
static void _prepareTransmitCurrentMission(void)
{
    mlPending.miTransaction = MISSION_TRANSACTION_SEND_CURRENT;
}

/**
* Transmit a mission acknowledgement message.
* @param type is the acknowledgement type to send (see MAV_MISSIONRESULT).
*/
static void _prepareTransmitMissionAck(uint8_t type)
{
    mlPending.miTransaction = MISSION_TRANSACTION_SEND_ACK;
    mlPending.miAckType = type;
}

/**
* Transmit the mission count.
*/
static void _prepareTransmitMissionCount(void) {
    mlPending.miTransaction = MISSION_TRANSACTION_SEND_COUNT;
}

/**
 * Transmit a mission item.
 * @param currentMissionIndex is the index of the mission item to transmit
 */
static void _prepareTransmitMissionItem(uint8_t currentMissionIndex)
{
    if (currentMissionIndex < mlPending.miTotalMissions) {
        mlPending.miTransaction = MISSION_TRANSACTION_SEND_ITEM;
        mlPending.miCurrentMission = currentMissionIndex;
    }
}

/**
 * Transmit a request for a mission item.
 * @param currentMissionIndex is the index of the desired mission item
 */
static void _prepareTransmitMissionRequest(uint8_t currentMissionIndex) {
    if (currentMissionIndex < mlPending.miTotalMissions) {
        mlPending.miTransaction = MISSION_TRANSACTION_SEND_REQUEST;
        mlPending.miCurrentMission = currentMissionIndex;
    }
}

//-- Uart Messages --
static uint8_t _prepareGpsMavlink(unsigned char *buf) {
    memset(&mavlinkMessageBuffer, 0, sizeof (mavlink_message_t));
    mavlink_msg_gps_raw_int_pack(SLUGS_SYSTEMID,
        SLUGS_COMPID,
        &mavlinkMessageBuffer,
        mlGpsData.time_usec,
        mlGpsData.fix_type,
        mlGpsData.lat,
        mlGpsData.lon,
        mlGpsData.alt,
        mlGpsData.eph,
        0.0, // ephv
        mlGpsData.vel,
        mlGpsData.cog,
        0 // sattelites_visible
        );
    return mavlink_msg_to_send_buffer(buf, &mavlinkMessageBuffer);
}

static uint8_t _prepareHeartbeatMavlink(unsigned char *buf) {
    memset(&mavlinkMessageBuffer, 0, sizeof (mavlink_message_t));
    mavlink_msg_heartbeat_pack(SLUGS_SYSTEMID,
        SLUGS_COMPID,
        &mavlinkMessageBuffer,
        MAV_TYPE_FIXED_WING,
        MAV_AUTOPILOT_SLUGS, //MAV_AUTOPILOT_GENERIC,
        mlHeartbeatLocal.base_mode,
        mlHeartbeatLocal.custom_mode,
        mlHeartbeatLocal.system_status
        );
    return mavlink_msg_to_send_buffer(buf, &mavlinkMessageBuffer);
}

static uint8_t _prepareGpsTimeMavlink(unsigned char *buf) {
    memset(&mavlinkMessageBuffer, 0, sizeof (mavlink_message_t));
    mavlink_msg_gps_date_time_encode(SLUGS_SYSTEMID,
        SLUGS_COMPID,
        &mavlinkMessageBuffer,
        &mlGpsDateTime);
    return mavlink_msg_to_send_buffer(buf, &mavlinkMessageBuffer);
}

static uint8_t _prepareVfrHudMavlink(unsigned char *buf) {
    memset(&mavlinkMessageBuffer, 0, sizeof (mavlink_message_t));
    mavlink_msg_vfr_hud_pack(SLUGS_SYSTEMID,
        SLUGS_COMPID,
        &mavlinkMessageBuffer,
        mlNavigation.u_m,               // air speed (m/s)
        (float)mlGpsData.vel * 0.01f,   // ground speed (m/s)
        0.0f,
        0,                              // throttle from 0 to 100 (percent)
        mlLocalPositionData.z,          // altitude (m)
        mlLocalPositionData.vz         // climb rate (m/s)
        );
    return mavlink_msg_to_send_buffer(buf, &mavlinkMessageBuffer);
}

static uint8_t _prepareSlugsNavigationMavlink(unsigned char *buf) {
    memset(&mavlinkMessageBuffer, 0, sizeof (mavlink_message_t));
    mavlink_msg_slugs_navigation_encode(SLUGS_SYSTEMID,
        SLUGS_COMPID,
        &mavlinkMessageBuffer,
        &mlNavigation);
    return mavlink_msg_to_send_buffer(buf, &mavlinkMessageBuffer);
}

static uint8_t _prepareCpuLoadMavlink(unsigned char *buf) {
    memset(&mavlinkMessageBuffer, 0, sizeof (mavlink_message_t));
    mavlink_msg_cpu_load_encode(SLUGS_SYSTEMID,
        SLUGS_COMPID,
        &mavlinkMessageBuffer,
        &mlCpuLoadData);
    return mavlink_msg_to_send_buffer(buf, &mavlinkMessageBuffer);
}

static uint8_t _prepareAttitudeMavlink(unsigned char *buf) {
    memset(&mavlinkMessageBuffer, 0, sizeof (mavlink_message_t));
    mavlink_msg_attitude_encode(SLUGS_SYSTEMID,
        SLUGS_COMPID,
        &mavlinkMessageBuffer,
        &mlAttitudeRotated);
    return mavlink_msg_to_send_buffer(buf, &mavlinkMessageBuffer);
}

static uint8_t _prepareLocalPositionMavlink(unsigned char *buf) {
    memset(&mavlinkMessageBuffer, 0, sizeof (mavlink_message_t));
    mavlink_msg_local_position_ned_encode(SLUGS_SYSTEMID,
        SLUGS_COMPID,
        &mavlinkMessageBuffer,
        &mlLocalPositionData);
    return mavlink_msg_to_send_buffer(buf, &mavlinkMessageBuffer);
}

static uint8_t _prepareSystemStatusMavlink(unsigned char *buf) {
    memset(&mavlinkMessageBuffer, 0, sizeof (mavlink_message_t));
    mavlink_msg_sys_status_encode(SLUGS_SYSTEMID,
        SLUGS_COMPID,
        &mavlinkMessageBuffer,
        &mlSystemStatus);
    return mavlink_msg_to_send_buffer(buf, &mavlinkMessageBuffer);
}

static uint8_t _prepareDiagnosticMavlink(unsigned char *buf) {
    memset(&mavlinkMessageBuffer, 0, sizeof (mavlink_message_t));
    mavlink_msg_diagnostic_encode(SLUGS_SYSTEMID,
        SLUGS_COMPID,
        &mavlinkMessageBuffer,
        &mlDiagnosticData);
    return mavlink_msg_to_send_buffer(buf, &mavlinkMessageBuffer);
}

static uint8_t _prepareSensorDiagnosticMavlink(unsigned char *buf) {
    memset(&mavlinkMessageBuffer, 0, sizeof (mavlink_message_t));
    mavlink_msg_sensor_diag_encode(SLUGS_SYSTEMID,
        SLUGS_COMPID,
        &mavlinkMessageBuffer,
        &mlSensorDiag);
    return mavlink_msg_to_send_buffer(buf, &mavlinkMessageBuffer);
}

static uint8_t _prepareDataLogMavlink(unsigned char *buf) {
    memset(&mavlinkMessageBuffer, 0, sizeof (mavlink_message_t));
    mavlink_msg_data_log_encode(SLUGS_SYSTEMID,
        SLUGS_COMPID,
        &mavlinkMessageBuffer,
        &mlDataLog);
    return mavlink_msg_to_send_buffer(buf, &mavlinkMessageBuffer);
}

static uint8_t _prepareGpsStatusMavlink(unsigned char *buf) {
    memset(&mavlinkMessageBuffer, 0, sizeof (mavlink_message_t));
#if USE_NMEA
    mavlink_msg_status_gps_encode(SLUGS_SYSTEMID,
        SLUGS_COMPID,
        &mavlinkMessageBuffer,
        &mlGpsStatus);
#else
    mavlink_msg_novatel_diag_encode(SLUGS_SYSTEMID,
        SLUGS_COMPID,
        &mavlinkMessageBuffer,
        &mlNovatelStatus);
#endif
    return mavlink_msg_to_send_buffer(buf, &mavlinkMessageBuffer);
}

static uint8_t _prepareServoOutputMavlink(unsigned char *buf) {
    memset(&mavlinkMessageBuffer, 0, sizeof (mavlink_message_t));
    mavlink_msg_servo_output_raw_encode(SLUGS_SYSTEMID,
        SLUGS_COMPID,
        &mavlinkMessageBuffer,
        &mlPwmCommands);
    return mavlink_msg_to_send_buffer(buf, &mavlinkMessageBuffer);
}

static uint8_t _prepareSensorBiasMavlink(unsigned char *buf) {
    memset(&mavlinkMessageBuffer, 0, sizeof (mavlink_message_t));
    mavlink_msg_sensor_bias_encode(SLUGS_SYSTEMID,
        SLUGS_COMPID,
        &mavlinkMessageBuffer,
        &mlSensorBiasData);
    return mavlink_msg_to_send_buffer(buf, &mavlinkMessageBuffer);
}

static uint8_t _prepareRawPressureMavlink(unsigned char *buf) {
    memset(&mavlinkMessageBuffer, 0, sizeof (mavlink_message_t));
    mavlink_msg_raw_pressure_encode(SLUGS_SYSTEMID,
        SLUGS_COMPID,
        &mavlinkMessageBuffer,
        &mlRawPressureData);
    return mavlink_msg_to_send_buffer(buf, &mavlinkMessageBuffer);
}

static uint8_t _prepareScaledPressureMavlink(unsigned char *buf) {
    memset(&mavlinkMessageBuffer, 0, sizeof (mavlink_message_t));
    mavlink_msg_scaled_pressure_encode(SLUGS_SYSTEMID,
        SLUGS_COMPID,
        &mavlinkMessageBuffer,
        &mlAirData);
    return mavlink_msg_to_send_buffer(buf, &mavlinkMessageBuffer);
}

static uint8_t _prepareRcChannelsMavlink(unsigned char *buf) {
    memset(&mavlinkMessageBuffer, 0, sizeof (mavlink_message_t));
    mavlink_msg_rc_channels_raw_encode(SLUGS_SYSTEMID,
        SLUGS_COMPID,
        &mavlinkMessageBuffer,
        &mlPilotConsoleData);
    return mavlink_msg_to_send_buffer(buf, &mavlinkMessageBuffer);
}

static uint8_t _prepareRawImuMavlink(unsigned char *buf) {
    memset(&mavlinkMessageBuffer, 0, sizeof (mavlink_message_t));
    mavlink_msg_raw_imu_encode(SLUGS_SYSTEMID,
        SLUGS_COMPID,
        &mavlinkMessageBuffer,
        &mlRawImuData);
    return mavlink_msg_to_send_buffer(buf, &mavlinkMessageBuffer);
}

static uint8_t _prepareScaledImuMavlink(unsigned char *buf) {
    memset(&mavlinkMessageBuffer, 0, sizeof (mavlink_message_t));
    mavlink_msg_scaled_imu_encode(SLUGS_SYSTEMID,
        SLUGS_COMPID,
        &mavlinkMessageBuffer,
        &mlFilteredData);
    return mavlink_msg_to_send_buffer(buf, &mavlinkMessageBuffer);
}

static uint8_t _preparePtzStatusMavlink(unsigned char *buf) {
    memset(&mavlinkMessageBuffer, 0, sizeof (mavlink_message_t));
    mavlink_msg_ptz_status_encode(SLUGS_SYSTEMID,
        SLUGS_COMPID,
        &mavlinkMessageBuffer,
        &mlPtzStatus);
    return mavlink_msg_to_send_buffer(buf, &mavlinkMessageBuffer);
}

static uint8_t _prepareVoltSensorMavlink(unsigned char *buf) {
    memset(&mavlinkMessageBuffer, 0, sizeof (mavlink_message_t));
    mavlink_msg_volt_sensor_encode(SLUGS_SYSTEMID,
        SLUGS_COMPID,
        &mavlinkMessageBuffer,
        &mlVISensor);
    return mavlink_msg_to_send_buffer(buf, &mavlinkMessageBuffer);
}

static uint8_t _preparePassthroughMavlink(unsigned char *buf) {
    memset(&mavlinkMessageBuffer, 0, sizeof (mavlink_message_t));
    mavlink_msg_ctrl_srfc_pt_pack(SLUGS_SYSTEMID,
        SLUGS_COMPID,
        &mavlinkMessageBuffer,
        GS_SYSTEMID,
        mlPassthrough.bitfieldPt);
    return mavlink_msg_to_send_buffer(buf, &mavlinkMessageBuffer);
}

static uint8_t _preparePingMavlink(unsigned char *buf) {
    memset(&mavlinkMessageBuffer, 0, sizeof (mavlink_message_t));
    mavlink_msg_ping_pack(SLUGS_SYSTEMID,
        SLUGS_COMPID,
        &mavlinkMessageBuffer,
        mlPing.time_usec, // respond with sent time
        mlPing.seq,
        SLUGS_SYSTEMID,
        SLUGS_COMPID
        );
        //mlRawImuData.time_usec); // this is for a request
    return mavlink_msg_to_send_buffer(buf, &mavlinkMessageBuffer);
}

static uint8_t _prepareParamValueMavlink(unsigned char *buf) {
    memset(&mavlinkMessageBuffer, 0, sizeof (mavlink_message_t));
    mavlink_msg_param_value_pack(SLUGS_SYSTEMID, SLUGS_COMPID,
         &mavlinkMessageBuffer,
        mlParamInterface.param_name[mlPending.piCurrentParameter],
        mlParamInterface.param[mlPending.piCurrentParameter],
        MAV_PARAM_TYPE_REAL32, // NOTE we only use floats for now
        PAR_PARAM_COUNT,
        mlPending.piCurrentParameter);
    return mavlink_msg_to_send_buffer(buf, &mavlinkMessageBuffer);
}

static uint8_t _prepareCommandLongMavlink(unsigned char *buf) {
    memset(&mavlinkMessageBuffer, 0, sizeof (mavlink_message_t));
    mavlink_msg_command_long_encode(SLUGS_SYSTEMID,
        SLUGS_COMPID,
        &mavlinkMessageBuffer,
        &mlCommand);
    return mavlink_msg_to_send_buffer(buf, &mavlinkMessageBuffer);
}

static uint8_t _prepareGpsOriginMavlink(unsigned char *buf) {
    memset(&mavlinkMessageBuffer, 0, sizeof (mavlink_message_t));
    mavlink_msg_gps_global_origin_pack(SLUGS_SYSTEMID,
        SLUGS_COMPID,
        &mavlinkMessageBuffer,
        FLOAT_TO_INT32_1E7(mlWpValues.lat[ORIGIN_WP_INDEX]),
        FLOAT_TO_INT32_1E7(mlWpValues.lon[ORIGIN_WP_INDEX]),
        FLOAT_TO_INT32_1E3(mlWpValues.alt[ORIGIN_WP_INDEX])
        );
    return mavlink_msg_to_send_buffer(buf, &mavlinkMessageBuffer);
}

static uint8_t _prepareMissionCurrentMavlink(unsigned char *buf) {
    memset(&mavlinkMessageBuffer, 0, sizeof (mavlink_message_t));
    mavlink_msg_mission_current_pack(SLUGS_SYSTEMID,
        SLUGS_COMPID,
        &mavlinkMessageBuffer,
        (uint16_t)mlNavigation.toWP - 1
        );
    return mavlink_msg_to_send_buffer(buf, &mavlinkMessageBuffer);
}

static uint8_t _prepareMissionAckMavlink(unsigned char *buf) {
    memset(&mavlinkMessageBuffer, 0, sizeof (mavlink_message_t));
    mavlink_msg_mission_ack_pack(SLUGS_SYSTEMID,
        MAV_COMP_ID_MISSIONPLANNER,
        &mavlinkMessageBuffer,
        GS_SYSTEMID,
        GS_COMPID,
        mlPending.miAckType
        );
    return mavlink_msg_to_send_buffer(buf, &mavlinkMessageBuffer);
}

static uint8_t _prepareMissionCountMavlink(unsigned char *buf) {
    memset(&mavlinkMessageBuffer, 0, sizeof (mavlink_message_t));
    mavlink_msg_mission_count_pack(SLUGS_SYSTEMID,
        SLUGS_COMPID,
        &mavlinkMessageBuffer,
        GS_SYSTEMID,
        GS_COMPID,
        mlPending.miTotalMissions
        );
    return mavlink_msg_to_send_buffer(buf, &mavlinkMessageBuffer);
}

static uint8_t _prepareMissionItemMavlink(unsigned char *buf) {
    memset(&mavlinkMessageBuffer, 0, sizeof (mavlink_message_t));
    mavlink_msg_mission_item_pack(SLUGS_SYSTEMID,
        MAV_COMP_ID_MISSIONPLANNER,
        &mavlinkMessageBuffer,
        GS_SYSTEMID,
        GS_COMPID,
        mlPending.miCurrentMission,
        MAV_FRAME_GLOBAL,
        mlWpValues.type[mlPending.miCurrentMission],
        0, // not current
        1, // always autocontinue
        0.0, // Param 1 not used
        0.0, // Param 2 not used
        (float) mlWpValues.orbit[mlPending.miCurrentMission],
        0.0, // Param 4 not used
        mlWpValues.lat[mlPending.miCurrentMission],
        mlWpValues.lon[mlPending.miCurrentMission],
        mlWpValues.alt[mlPending.miCurrentMission]
        );
    return mavlink_msg_to_send_buffer(buf, &mavlinkMessageBuffer);
}

static uint8_t _prepareMissionRequestMavlink(unsigned char *buf) {
    memset(&mavlinkMessageBuffer, 0, sizeof (mavlink_message_t));
    mavlink_msg_mission_request_pack(SLUGS_SYSTEMID,
        MAV_COMP_ID_MISSIONPLANNER,
        &mavlinkMessageBuffer,
        GS_SYSTEMID,
        GS_COMPID,
        mlPending.miCurrentMission
        );
    return mavlink_msg_to_send_buffer(buf, &mavlinkMessageBuffer);
}

static uint8_t _prepareCommandAckMavlink(unsigned char *buf) {
    memset(&mavlinkMessageBuffer, 0, sizeof (mavlink_message_t));
    mavlink_msg_command_ack_encode(SLUGS_SYSTEMID,
        SLUGS_COMPID,
        &mavlinkMessageBuffer,
        &mlCommandAck
    );
    return mavlink_msg_to_send_buffer(buf, &mavlinkMessageBuffer);
}

static uint8_t _prepareMidLevelCommandsMavlink(unsigned char *buf) {
    memset(&mavlinkMessageBuffer, 0, sizeof (mavlink_message_t));
    mavlink_msg_mid_lvl_cmds_pack(SLUGS_SYSTEMID,
        SLUGS_COMPID,
        &mavlinkMessageBuffer,
        GS_SYSTEMID,
        mlMidLevelCommands.hCommand,
        mlMidLevelCommands.uCommand,
        mlMidLevelCommands.rCommand
    );
    return mavlink_msg_to_send_buffer(buf, &mavlinkMessageBuffer);
}

static uint8_t _prepareIsrMavlink(unsigned char *buf) {
    memset(&mavlinkMessageBuffer, 0, sizeof (mavlink_message_t));
    mavlink_msg_isr_location_encode(SLUGS_SYSTEMID,
        SLUGS_COMPID,
        &mavlinkMessageBuffer,
        &mlISR
    );
    return mavlink_msg_to_send_buffer(buf, &mavlinkMessageBuffer);
}

static uint8_t _prepareStatusTextMavlink(unsigned char *buf) {
    memset(&mavlinkMessageBuffer, 0, sizeof (mavlink_message_t));
    mavlink_msg_statustext_pack(SLUGS_SYSTEMID,
        SLUGS_COMPID,
        &mavlinkMessageBuffer,
        mlStatustext.severity,
        mlStatustext.text
    );
    return mavlink_msg_to_send_buffer(buf, &mavlinkMessageBuffer);
}

//-- SPI Messages --
static void _sendSpiSetGpsOriginMavlink(void) {
    memset(&mavlinkMessageBuffer, 0, sizeof (mavlink_message_t));
    mavlink_msg_set_gps_global_origin_pack(SLUGS_SYSTEMID,
        SLUGS_COMPID,
        &mavlinkMessageBuffer,
        SLUGS_SYSTEMID, // target system ID
        FLOAT_TO_INT32_1E7(mlWpValues.lat[ORIGIN_WP_INDEX]),
        FLOAT_TO_INT32_1E7(mlWpValues.lon[ORIGIN_WP_INDEX]),
        FLOAT_TO_INT32_1E3(mlWpValues.alt[ORIGIN_WP_INDEX])
        );
    addMessageToSpiOut(&mavlinkMessageBuffer);
}


static void _sendSpiServoOutputMavlink(void) {
    memset(&mavlinkMessageBuffer, 0, sizeof (mavlink_message_t));
    mavlink_msg_servo_output_raw_encode(SLUGS_SYSTEMID,
        SLUGS_COMPID,
        &mavlinkMessageBuffer,
        &mlPwmCommands
        );
    addMessageToSpiOut(&mavlinkMessageBuffer);
}

/**
 * Copies the UART1 output buffer to the DMA1 buffer for sending.
 * @param size in bytes to copy to the DMA1 buffer.
 */
static void _uart1OutputToDMA1(uint16_t size) {
#ifdef RECORD_TO_LOGGER
    uint16_t i;
    size = (size > 0x400)? 0x400 : size; // 10-bit max
    for (i = 0; i < size; i++) {
        dma1Buffer[i] = (unsigned int) readFront(uart1BufferOut);
    }
#endif
}

/**
 * Copies the UART2 output buffer to the DMA1 buffer for sending.
 * @param size in bytes to copy to the DMA1 buffer.
 */
static void _uart2OutputToDMA2(uint16_t size) {
    uint16_t i;
    size = (size > 0x400)? 0x400 : size; // 10-bit max
    for (i = 0; i < size; i += 1) {
        dma2Buffer[i] = (unsigned int) readFront(uart2BufferOut);
    }
}

// **** Interrupt service routines ****
void __attribute__((interrupt, no_auto_psv)) _DMA1Interrupt(void) {
    // Clear the DMA1 Interrupt Flag;
    IFS0bits.DMA1IF = 0;
}

void __attribute__((interrupt, no_auto_psv)) _DMA2Interrupt(void) {
    // Clear the DMA1 Interrupt Flag;
    IFS1bits.DMA2IF = 0;
}

void __attribute__((__interrupt__, no_auto_psv)) _U2RXInterrupt(void) {
    LED_CTRL_STATUS_TOGGLE();
    // Read the buffer while it has data
    // and add it to the circular buffer
    while (U2STAbits.URXDA == 1) {
        writeBack(uart2BufferIn, (unsigned char) U2RXREG);
    }

    // If there was an overun error clear it and continue
    if (U2STAbits.OERR == 1) {
        U2STAbits.OERR = 0;
    }

    // clear the interrupt
    IFS1bits.U2RXIF = 0;
}

void __attribute__((interrupt, no_auto_psv)) _U1ErrInterrupt(void) {

    // If there was an overun error clear it and continue
    if (U1STAbits.OERR == 1) {
        U1STAbits.OERR = 0;
    }

    // If there was an overun error clear it and continue
    if (IFS4bits.U1EIF == 1) {
        IFS4bits.U1EIF = 0; // Clear the UART1 Error Interrupt Flag
    }
}

void __attribute__((interrupt, no_auto_psv)) _U2ErrInterrupt(void) {

    // If there was an overun error clear it and continue
    if (U2STAbits.OERR == 1) {
        LED_CTRL_BUSY = ON;
        U2STAbits.OERR = 0;
    }

    // If there was an overun error clear it and continue
    if (IFS4bits.U2EIF == 1) {
        LED_CTRL_BUSY = ON;
        IFS4bits.U2EIF = 0; // Clear the UART2 Error Interrupt Flag
    }
}


