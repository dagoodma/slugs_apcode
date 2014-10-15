// TODO fix volatility, add interrupts for logger TX, write logger function,
//      wrap everything in ifdefs, hook function to interrupt to write to log
// **** Include libraries ****
// Standard libraries
#include <math.h>
#include <stdint.h>

// Microchip libraries

// User libraries
#include "mavlinkCommsControlMcu.h"

// **** Macros or preprocessor directives ****


// **** Module-level, global, or external variables ****
// UART1 circular buffer for logging mavlink messages
#ifdef RECORD_TO_LOGGER
static volatile struct CircBuffer _uart1BufferOut;
static volatile CBRef uart1BufferOut;
unsigned volatile int dma1Buffer[MAXSEND] __attribute__((space(dma))) = {0};
static uint16_t uart1BytesToSend;
#endif

// UART2 circular and DMA buffers for radio RX messages from groundstation
static struct CircBuffer _uart2BufferIn;
static CBRef uart2BufferIn;
static struct CircBuffer _uart2BufferOut;
static CBRef uart2BufferOut;
unsigned int dma2Buffer[MAXSEND] __attribute__((space(dma))) = {0};

// Temporary and debug variables
char sw_debug;
char sw_temp[50];
char sw_intTemp;
float fl_temp1, fl_temp2;

// **** Private function prototypes ****
static void _uart1OutputToDMA1(uint16_t size);
//void uart2CopyOutputToDMA2(unsigned char size);
static void _logTelemetryMavlink(void);

static void _prepareTransmitParameter(uint16_t id);
static void _prepareTransmitCurrentMission(void);
static void _prepareTransmitMissionAck(uint8_t type);
static void _prepareTransmitMissionCount(void);
static void _prepareTransmitMissionItem(uint8_t currentMissionIndex);
static void _prepareTransmitMissionRequest(uint8_t currentMissionIndex);

// **** Public functions ****
/**
 * Initializes UART1 and DMA1 for logging data to an SD Card.
 */
void uart1Init(void) {
#ifdef RECORD_TO_LOGGER
    // Initialize the circular buffer
    uart1BufferOut = (struct CircBuffer*) &_uart1BufferOut;
    newCircBuffer(uart1BufferOut);
    uart1BytesToSend = 0;

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
    DMA1CNT = MAXSEND - 1; // NOTE should this be 0 to start?

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
#endif
}

/**
 * Initializes UART2 and DMA2 for radio communication.
 */
void uart2Init(void) {
    sw_debug = 0;

    // Initialize the circular buffers
    uart2BufferIn = (struct CircBuffer*) &_uart2BufferIn;
    newCircBuffer(uart2BufferIn);

    uart2BufferOut = (struct CircBuffer*) &_uart2BufferOut;
    newCircBuffer(uart2BufferOut);


    // DMA1REQ is channel IRQ select register
    DMA1REQ = 31; // IRQ Number for UART2 Transmission (see table 8-1)

    // DMA1PAD is peripheral address register
    DMA1PAD = (volatile unsigned int) &U2TXREG;

    // DMA1CON is channel control register
    DMA1CONbits.AMODE = 0; // Register Indirect with post-increment
    DMA1CONbits.MODE = 1; // One-shot, No Ping-Pong Mode
    DMA1CONbits.DIR = 1; // Read from RAM and send to Periphereal
    DMA1CONbits.SIZE = 0; // Word Data Transfer

    // DMA1CNT is transfer count register
    DMA1CNT = MAXSEND - 1;

    // DMA1STA is primary start address offset register
    DMA1STA = __builtin_dmaoffset(dma2Buffer);

    // Enable DMA1 TX interrupts
    IFS0bits.DMA1IF = 0; // Clear DMA Interrupt Flag
    IPC3bits.DMA1IP = 6; // interrupt priority to 6
    IEC0bits.DMA1IE = 1; // Enable DMA interrupt

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
    if (!(DMA1CONbits.CHEN) && (bufLen > 0)) {
        // Configure the bytes to send
        DMA1CNT = bufLen <= (MAXSEND - 1) ? bufLen - 1 : MAXSEND - 1;
        // copy the buffer to the DMA channel outgoing buffer
        copyBufferToDMA1((unsigned char) DMA1CNT + 1);
        // Enable the DMA
        DMA1CONbits.CHEN = 1;
        // Init the transmission
        DMA1REQbits.FORCE = 1;
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

/* Called at 50 Hz by Simulink. */
/* Full telemetry version. */
#ifdef ENABLE_FULL_TELEMETRY
void prepareTelemetryMavlink(unsigned char* dataOut) {
    // Record to logger first
    #ifdef RECORD_TO_LOGGER
    _logTelemetryMavlink();
    #endif

    // Generic message container used to pack the messages
    mavlink_message_t msg;

    // Generic buffer used to hold data to be streamed via serial port
    //uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Cycles from 1 to 10 to decide which
    // message's turn is to be sent
    static uint8_t sampleTelemetry = 1;

    // Contains the total bytes to send via the serial port
    uint8_t bytes2Send = 0;//, paramDelay = 0;

    // String used to send text messages to QGC console
    char vr_message[50];

    memset(&msg, 0, sizeof (mavlink_message_t));

    switch (sampleTelemetry) {
        case 1: // GPS, Heartbeat and Passthrough if necessary
            // Pack the GPS message
            mavlink_msg_gps_raw_int_pack(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
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

            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

            // clear the msg to pack a new variable
            memset(&msg, 0, sizeof (mavlink_message_t));

            // Pack the Heartbeat message
            mavlink_msg_heartbeat_pack(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                MAV_TYPE_FIXED_WING,
                MAV_AUTOPILOT_SLUGS,
                //MAV_AUTOPILOT_GENERIC,
                mlHeartbeatLocal.base_mode,
                mlHeartbeatLocal.custom_mode,
                mlHeartbeatLocal.system_status//,
                //mlHeartbeatLocal.mavlink_version
                );

            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

            // If passthrough was requested (never used)
            if (mlPending.pt == TRUE) {
                // clear the message
                memset(&msg, 0, sizeof (mavlink_message_t));

                mavlink_msg_ctrl_srfc_pt_pack(SLUGS_SYSTEMID,
                    SLUGS_COMPID,
                    &msg,
                    GS_SYSTEMID,
                    mlPassthrough.bitfieldPt);
                // Copy the message to the send buffer
                bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

                // Clear the flag
                mlPending.pt = FALSE;
            }

            break; //case 1

        case 2: // GPS Date Time, diagnostic, air data,
            mavlink_msg_gps_date_time_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlGpsDateTime);

            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

            // clear the msg
            memset(&msg, 0, sizeof (mavlink_message_t));

            mavlink_msg_diagnostic_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlDiagnosticData);
            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

            // clear the msg
            memset(&msg, 0, sizeof (mavlink_message_t));

            mavlink_msg_scaled_pressure_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlAirData);
            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

            break; // case 2

        case 3: // data log, ping, vfr_hud
            mavlink_msg_data_log_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlDataLog);

            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

            if (mlPending.ping == 1) {
                // clear the msg
                memset(&msg, 0, sizeof (mavlink_message_t));
                mlPending.ping = 0;

                mavlink_msg_ping_pack(SLUGS_SYSTEMID,
                    SLUGS_COMPID,
                    &msg,
                    mlPing.time_usec, // respond with sent time
                    mlPing.seq,
                    SLUGS_SYSTEMID,
                    SLUGS_COMPID
                    );
                    //mlRawImuData.time_usec); // this is for a request

                // Copy the message to the send buffer
                bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);
            }


            memset(&msg, 0, sizeof (mavlink_message_t));
            // Hud data for primary flight display
            mavlink_msg_vfr_hud_pack(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                mlNavigation.u_m,               // air speed (m/s)
                (float)mlGpsData.vel * 0.01f,   // ground speed (m/s)
                //(float)(mlAttitudeData.yaw + M_PI)*(180.0f/M_PI), // heading from 0 to 360 (deg)
                0.0f,
                0,                              // throttle from 0 to 100 (percent)
                mlLocalPositionData.z,          // altitude (m)
                mlLocalPositionData.vz         // climb rate (m/s)
                );

            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);


            break; // case 3

        case 4: // navigation, cpu load, sensor diag

            mavlink_msg_slugs_navigation_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlNavigation);
            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

            // clear the msg
            memset(&msg, 0, sizeof (mavlink_message_t));

            mavlink_msg_cpu_load_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlCpuLoadData);
            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

             // clear the msg
            memset(&msg, 0, sizeof (mavlink_message_t));

            mavlink_msg_sensor_diag_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlSensorDiag);
            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

            break; // case 4

        case 5: // Raw IMU, Parameter Interface
            mavlink_msg_raw_imu_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlRawImuData);
            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);


            evaluateParameterState(PARAM_EVENT_NONE, NULL);

            /* Set by evaluateParameterState() via _prepareTransmitParameter()
               when we're ready to transmit. */
            //if (mlPending.sensorDspReady) { // TODO check param request on startup
            if (mlPending.piTransaction == PARAM_TRANSACTION_SEND) {
                mavlink_msg_param_value_pack(SLUGS_SYSTEMID, SLUGS_COMPID, &msg,
                    mlParamInterface.param_name[mlPending.piCurrentParameter],
                    mlParamInterface.param[mlPending.piCurrentParameter],
                    MAV_PARAM_TYPE_REAL32, // NOTE we only use floats for now
                    PAR_PARAM_COUNT,
                    mlPending.piCurrentParameter);

                mlPending.piTransaction = PARAM_TRANSACTION_NONE;
                if (++mlPending.piCurrentParameter >= PAR_PARAM_COUNT)
                    mlPending.piCurrentParameter = 0;


                // Copy the message to the send buffer
                bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);
/*
                memset(vr_message,0,sizeof(vr_message));
             	sprintf(vr_message, "P = %d, V=%f ", mlPending.piCurrentParameter,
                    (float)mlParamInterface.param[mlPending.piCurrentParameter]);
             	bytes2Send += sendQGCDebugMessage (vr_message, 0, dataOut, bytes2Send+1);
   */
            }
            break; // case 5

        case 6: // Local Position, System Status, GPS Status

            mavlink_msg_local_position_ned_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlLocalPositionData);
            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);


            memset(&msg, 0, sizeof (mavlink_message_t));

            mavlink_msg_sys_status_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlSystemStatus);
            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

            memset(&msg, 0, sizeof (mavlink_message_t));
#if USE_NMEA
            mavlink_msg_status_gps_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlGpsStatus);
#else
            mavlink_msg_novatel_diag_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlNovatelStatus);
#endif
            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

            break; // case 6

        case 7: // PWM Commands, Biases, slugs action

            mavlink_msg_servo_output_raw_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlPwmCommands);
            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

            // if in HIL Mode, report PWM commands to Sensor MCU so they
            // are sent to the 6DOF model in Simulink
            if (hasMode(mlHeartbeatLocal.base_mode, MAV_MODE_FLAG_HIL_ENABLED)) {
                addMessageToSpiOut(&msg);
            }

            // clear the msg
            memset(&msg, 0, sizeof (mavlink_message_t));

            mavlink_msg_sensor_bias_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlSensorBiasData);
            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

            //
            if (mlPending.command != FALSE) {
                // clear the msg
                memset(&msg, 0, sizeof (mavlink_message_t));

                mavlink_msg_command_long_encode(SLUGS_SYSTEMID,
                    SLUGS_COMPID,
                    &msg,
                    &mlCommand);

                // Copy the message to the send buffer
                bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

                mlPending.command = 0;
            }

            break; // case 7

        case 8:// mission Protocol state machine, raw Pressure
            // This block goes via SPI to the sensor DSC so it does not count
            // towards the Bandwith budget
            if (mlPending.spiSendGSLocation) {
                mavlink_msg_set_gps_global_origin_pack(SLUGS_SYSTEMID,
                    SLUGS_COMPID,
                    &msg,
                    //GS_COMPID,
                    GS_SYSTEMID,
                    mlWpValues.lat[MAX_NUM_WPS - 1],
                    mlWpValues.lon[MAX_NUM_WPS - 1],
                    mlWpValues.alt[MAX_NUM_WPS - 1]);

                addMessageToSpiOut(&msg);

                memset(vr_message, 0, sizeof (vr_message));
                sprintf(vr_message, "Sensor DSC Reboot.");
                bytes2Send += sendQGCDebugMessage(vr_message, 255, dataOut, bytes2Send + 1);

                mlPending.spiSendGSLocation = 0;
            }

            // end of SPI block

            if (sw_debug == 1) {
                memset(vr_message, 0, sizeof (vr_message));
                sprintf(vr_message, "hla = %2.4f, hlo =%2.4f  hh = %2.2f", (double)mlWpValues.lat[MAX_NUM_WPS - 1], (double)mlWpValues.lon[MAX_NUM_WPS - 1], (double)mlWpValues.alt[MAX_NUM_WPS - 1]);
                bytes2Send += sendQGCDebugMessage(vr_message, 0, dataOut, bytes2Send + 1);
                sw_debug = 0;
            }
            //
            if (sw_debug == 4) {
                memset(vr_message, 0, sizeof (vr_message));
                sprintf(vr_message, "%d %d %d %d %d %d %d %d %d %d", sw_temp[0], sw_temp[1], sw_temp[2], sw_temp[3], sw_temp[4], sw_temp[5], sw_temp[6], sw_temp[7], sw_temp[8], sw_temp[9]);
                bytes2Send += sendQGCDebugMessage(vr_message, 0, dataOut, bytes2Send + 1);
                sw_debug = 0;

            }
            //
            // 			if (sw_debug == 3){
            // 				memset(vr_message,0,sizeof(vr_message));
            // 				sprintf(vr_message, "Mode = %d", mlSystemStatus.mode);
            // 				bytes2Send += sendQGCDebugMessage (vr_message, 0, dataOut, bytes2Send+1);
            // 				sw_debug = 0;
            // 			}

             // Raw pressure
            memset(&msg, 0, sizeof (mavlink_message_t));
            mavlink_msg_raw_pressure_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlRawPressureData);
            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);


            /****  New mission state machine code ******/

            evaluateMissionState(MISSION_EVENT_NONE, NULL);

            // TODO determine if we need this
            // clear the msg
            if (mlPending.miTransaction != MISSION_TRANSACTION_NONE)
                memset(&msg, 0, sizeof (mavlink_message_t));

            /* Set by evaluateMissionState() when we're ready to transmit. */
            // Send current mission item number
            if (mlPending.miTransaction == MISSION_TRANSACTION_SEND_CURRENT) {
                mavlink_msg_mission_current_pack(SLUGS_SYSTEMID, SLUGS_COMPID, &msg,
                         ((uint16_t)mlNavigation.toWP) - 1);

                mlPending.miTransaction = MISSION_TRANSACTION_NONE;

                // Copy the message to the send buffer
                bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);
            }
            // Send mission acknowledgement
            else if (mlPending.miTransaction == MISSION_TRANSACTION_SEND_ACK) {
                mavlink_msg_mission_ack_pack(SLUGS_SYSTEMID,
                        MAV_COMP_ID_MISSIONPLANNER,
                        &msg,
                        GS_SYSTEMID,
                        GS_COMPID,
                        mlPending.miAckType);

                mlPending.miTransaction = MISSION_TRANSACTION_NONE;

                // Copy the message to the send buffer
                bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

#ifdef DEBUG_MISSION_SM
                memset(vr_message,0,sizeof(vr_message));
             	sprintf(vr_message, "Sent ack = %d", mlPending.miAckType);
             	bytes2Send += sendQGCDebugMessage (vr_message, 0, dataOut, bytes2Send+1);
#endif
            }
            // Send mission count
            else if (mlPending.miTransaction == MISSION_TRANSACTION_SEND_COUNT) {
                mavlink_msg_mission_count_pack(SLUGS_SYSTEMID, SLUGS_COMPID, &msg,
                    GS_SYSTEMID, GS_COMPID, mlPending.miTotalMissions);

                mlPending.miTransaction = MISSION_TRANSACTION_NONE;

                // Copy the message to the send buffer
                bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

#ifdef DEBUG_MISSION_SM
                memset(vr_message,0,sizeof(vr_message));
             	sprintf(vr_message, "Sent count = %d", mlPending.miTotalMissions);
             	bytes2Send += sendQGCDebugMessage (vr_message, 0, dataOut, bytes2Send+1);
#endif
            }
            // Send mission item
            else if (mlPending.miTransaction == MISSION_TRANSACTION_SEND_ITEM) {
                mavlink_msg_mission_item_pack(SLUGS_SYSTEMID,
                    MAV_COMP_ID_MISSIONPLANNER,
                    &msg,
                    GS_SYSTEMID,
                    GS_COMPID,
                    mlPending.miCurrentMission,
                    MAV_FRAME_GLOBAL,
                    mlWpValues.type[mlPending.miCurrentMission],
                    0, // not current
                    1, // autocontinue
                    0.0, // Param 1 not used
                    0.0, // Param 2 not used
                    (float) mlWpValues.orbit[mlPending.miCurrentMission],
                    0.0, // Param 4 not used
                    mlWpValues.lat[mlPending.miCurrentMission],
                    mlWpValues.lon[mlPending.miCurrentMission],
                    mlWpValues.alt[mlPending.miCurrentMission]); // always autocontinue

                mlPending.miTransaction = MISSION_TRANSACTION_NONE;

                // Copy the message to the send buffer
                bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

#ifdef DEBUG_MISSION_SM
                memset(vr_message,0,sizeof(vr_message));
             	sprintf(vr_message, "Sent item = %d", mlPending.miCurrentMission);
             	bytes2Send += sendQGCDebugMessage (vr_message, 0, dataOut, bytes2Send+1);
#endif
            }
             // Send mission request
            else if (mlPending.miTransaction == MISSION_TRANSACTION_SEND_REQUEST
                    && mlPending.miCurrentMission < mlPending.miTotalMissions) {

                mavlink_msg_mission_request_pack(SLUGS_SYSTEMID,
                    MAV_COMP_ID_MISSIONPLANNER,
                    &msg,
                    GS_SYSTEMID,
                    GS_COMPID,
                    mlPending.miCurrentMission);

                mlPending.miTransaction = MISSION_TRANSACTION_NONE;
                // Increment pending
                // TODO remove this or not
                /*
                if (++mlPending.miCurrentMission >= mlPending.miTotalMissions)
                    mlPending.miCurrentMission = 0;
                 */
                // Copy the message to the send buffer
                bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

#ifdef DEBUG_MISSION_SM
                memset(vr_message,0,sizeof(vr_message));
             	sprintf(vr_message, "Sent request = %d", mlPending.miCurrentMission);
             	bytes2Send += sendQGCDebugMessage (vr_message, 0, dataOut, bytes2Send+1);
#endif

            }

            break; // case 8

        case 9: // Action Ack, Pilot Console, Mid Level Commands, boot

            mavlink_msg_rc_channels_raw_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlPilotConsoleData);
            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

            if (mlPending.commandAck) {
                mavlink_msg_command_ack_encode(SLUGS_SYSTEMID,
                    SLUGS_COMPID,
                    &msg,
                    &mlCommandAck);

                // Copy the message to the send buffer
                bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

                mlPending.commandAck = FALSE;
            }


            // if there is a pending request for the Mid Level Commands
            if (mlPending.midLvlCmds == 1) {
                // clear the msg
                memset(&msg, 0, sizeof (mavlink_message_t));

                mavlink_msg_mid_lvl_cmds_pack(SLUGS_SYSTEMID,
                    SLUGS_COMPID,
                    &msg,
                    GS_SYSTEMID,
                    mlMidLevelCommands.hCommand,
                    mlMidLevelCommands.uCommand,
                    mlMidLevelCommands.rCommand);

                // Copy the message to the send buffer
                bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

                // clear the flag
                mlPending.midLvlCmds = 0;
            }

            if (mlPending.isrLoc == 1) {
                // clear the msg
                memset(&msg, 0, sizeof (mavlink_message_t));

                mavlink_msg_isr_location_encode(SLUGS_SYSTEMID,
                    SLUGS_COMPID,
                    &msg,
                    &mlISR);

                // Copy the message to the send buffer
                bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

                mlPending.isrLoc = 0;
            }

            // if the boot message is set then transmit it urgently
            if (mlBoot.version == 1) {
                memset(vr_message, 0, sizeof (vr_message));
                sprintf(vr_message, "%s DSC Reboot.", "Control");
                bytes2Send += sendQGCDebugMessage(vr_message, 255, dataOut, bytes2Send + 1);
                mlBoot.version = 0;
            }

            break; // case 9

        case 10: // Filtered data, PTZ data, VI Sensor
            mavlink_msg_scaled_imu_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlFilteredData);

            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

            // clear the msg
            memset(&msg, 0, sizeof (mavlink_message_t));

            mavlink_msg_ptz_status_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlPtzStatus);

            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

            // clear the msg
            memset(&msg, 0, sizeof (mavlink_message_t));

            mavlink_msg_volt_sensor_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlVISensor);

            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

            break; // case 10

    } // Switch

    // Send additional statustext messages
    if (mlPending.statustext > 0) {
        memset(&msg, 0, sizeof (mavlink_message_t));


        mavlink_msg_statustext_pack(SLUGS_SYSTEMID,
        SLUGS_COMPID,
        &msg,
        mlStatustext.severity,
        mlStatustext.text);

        bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

        mlPending.statustext = 0;
    }



    memset(&msg, 0, sizeof (mavlink_message_t));

    mavlink_msg_attitude_encode(SLUGS_SYSTEMID,
        SLUGS_COMPID,
        &msg,
        &mlAttitudeRotated);
    // Copy the message to the send buffer
    bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);


    // Put the length of the message in the first byte of the outgoing array
    *dataOut = bytes2Send;

    // increment/overflow the samplePeriod counter
    // configured for 10 Hz in non vital messages
    sampleTelemetry = (sampleTelemetry >= 10) ? 1 : sampleTelemetry + 1;

}

#else

// Stripped down version of prepareTelemetryMavlink()

void prepareTelemetryMavlink(unsigned char* dataOut) {
    // Record to logger first
    #ifdef RECORD_TO_LOGGER
    _logTelemetryMavlink();
    #endif

    // Generic message container used to pack the messages
    mavlink_message_t msg;

    // Generic buffer used to hold data to be streamed via serial port
    //uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Cycles from 1 to 10 to decide which
    // message's turn is to be sent
    static uint8_t sampleTelemetry = 1;

    // Contains the total bytes to send via the serial port
    uint8_t bytes2Send = 0;//, paramDelay = 0;

    // String used to send text messages to QGC console
    char vr_message[50];

    memset(&msg, 0, sizeof (mavlink_message_t));

    switch (sampleTelemetry) {
         case 1: // GPS, Heartbeat and Passthrough if necessary
            // Pack the GPS message
            mavlink_msg_gps_raw_int_pack(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
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

            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

            // clear the msg to pack a new variable
            memset(&msg, 0, sizeof (mavlink_message_t));

            // Pack the Heartbeat message
            mavlink_msg_heartbeat_pack(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                MAV_TYPE_FIXED_WING,
                MAV_AUTOPILOT_SLUGS,
                //MAV_AUTOPILOT_GENERIC,
                mlHeartbeatLocal.base_mode,
                mlHeartbeatLocal.custom_mode,
                mlHeartbeatLocal.system_status//,
                //mlHeartbeatLocal.mavlink_version
                );

            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

            break; //case 1

        case 5: // GPS Date Time, diagnostic, air data,
            mavlink_msg_gps_date_time_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlGpsDateTime);

            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

            /*
            // clear the msg
            memset(&msg, 0, sizeof (mavlink_message_t));

            mavlink_msg_diagnostic_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlDiagnosticData);
            // Copy the message to the send buffer
            // TODO LOG this message
            */

            /*
            // clear the msg
            memset(&msg, 0, sizeof (mavlink_message_t));

            mavlink_msg_scaled_pressure_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlAirData);

            // TODO LOG this message
            */

            break; // case 2

        case 10: // data log, ping, vfr_hud
            /*
            mavlink_msg_data_log_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlDataLog);

            // TODO LOG this message
            */




            memset(&msg, 0, sizeof (mavlink_message_t));
            // Hud data for primary flight display
            mavlink_msg_vfr_hud_pack(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                mlNavigation.u_m,               // air speed (m/s)
                (float)mlGpsData.vel * 0.01f,   // ground speed (m/s)
                //(float)(mlAttitudeData.yaw + M_PI)*(180.0f/M_PI), // heading from 0 to 360 (deg)
                0.0f,
                0,                              // throttle from 0 to 100 (percent)
                mlLocalPositionData.z,          // altitude (m)
                mlLocalPositionData.vz         // climb rate (m/s)
                );

            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);


            break; // case 10

        case 15: // navigation, cpu load, sensor diag

            mavlink_msg_slugs_navigation_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlNavigation);
            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

            // clear the msg
            memset(&msg, 0, sizeof (mavlink_message_t));

            mavlink_msg_cpu_load_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlCpuLoadData);
            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

            /*
             // clear the msg
            memset(&msg, 0, sizeof (mavlink_message_t));

            mavlink_msg_sensor_diag_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlSensorDiag);

            // TODO LOG this message
            */
            break; // case 15

        case 20: // Raw IMU, Parameter Interface
            /*
            mavlink_msg_raw_imu_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlRawImuData);

            // TODO LOG this message
            */
            memset(&msg, 0, sizeof (mavlink_message_t));

            mavlink_msg_attitude_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlAttitudeRotated);


            break; // case 20

        case 25: // Local Position, System Status, GPS Status

            mavlink_msg_local_position_ned_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlLocalPositionData);
            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);


            memset(&msg, 0, sizeof (mavlink_message_t));

            mavlink_msg_sys_status_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlSystemStatus);
            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

            memset(&msg, 0, sizeof (mavlink_message_t));
#if USE_NMEA
            mavlink_msg_status_gps_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlGpsStatus);
#else
            mavlink_msg_novatel_diag_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlNovatelStatus);
#endif
            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

            break; // case 25

        case 30: // PWM Commands, Biases, slugs action

            /*
            mavlink_msg_servo_output_raw_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlPwmCommands);
            // TODO LOG this message
            */

            // TODO fix rate of this back to 5 Hz

            // if in HIL Mode, report PWM commands to Sensor MCU so they
            // are sent to the 6DOF model in Simulink
            if (hasMode(mlHeartbeatLocal.base_mode, MAV_MODE_FLAG_HIL_ENABLED)) {
                addMessageToSpiOut(&msg);
            }

            /*
            // clear the msg
            memset(&msg, 0, sizeof (mavlink_message_t));

            mavlink_msg_sensor_bias_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlSensorBiasData);
            // TODO LOG this message
            */

            //
            break; // case 30

        case 35:// mission Protocol state machine, raw Pressure

            //
            // 			if (sw_debug == 3){
            // 				memset(vr_message,0,sizeof(vr_message));
            // 				sprintf(vr_message, "Mode = %d", mlSystemStatus.mode);
            // 				bytes2Send += sendQGCDebugMessage (vr_message, 0, dataOut, bytes2Send+1);
            // 				sw_debug = 0;
            // 			}

             // Raw pressure
            /*
            memset(&msg, 0, sizeof (mavlink_message_t));
            mavlink_msg_raw_pressure_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlRawPressureData);

            // TODO LOG this message
            */


            break; // case 35

        case 40: // Action Ack, Pilot Console, Mid Level Commands, boot

            /*
            mavlink_msg_rc_channels_raw_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlPilotConsoleData);
            // TODO LOG this message
            */


            break; // case 40

        case 50: // Filtered data, PTZ data, VI Sensor
            /*
            mavlink_msg_scaled_imu_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlFilteredData);
            // TODO LOG this message


            // clear the msg
            memset(&msg, 0, sizeof (mavlink_message_t));

            mavlink_msg_ptz_status_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlPtzStatus);
            // TODO LOG this message
             */

            // clear the msg
            memset(&msg, 0, sizeof (mavlink_message_t));

            mavlink_msg_volt_sensor_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlVISensor);

            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

            break; // case 10

        default:
            break;

    } // Switch

    // If passthrough was requested (never used)
    if (mlPending.pt == TRUE) {
        // clear the message
        memset(&msg, 0, sizeof (mavlink_message_t));

        mavlink_msg_ctrl_srfc_pt_pack(SLUGS_SYSTEMID,
            SLUGS_COMPID,
            &msg,
            GS_SYSTEMID,
            mlPassthrough.bitfieldPt);
        // Copy the message to the send buffer
        bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

        // Clear the flag
        mlPending.pt = FALSE;
    }

     if (mlPending.ping == 1) {
        // clear the msg
        memset(&msg, 0, sizeof (mavlink_message_t));
        mlPending.ping = 0;

        mavlink_msg_ping_pack(SLUGS_SYSTEMID,
            SLUGS_COMPID,
            &msg,
            mlPing.time_usec, // respond with sent time
            mlPing.seq,
            SLUGS_SYSTEMID,
            SLUGS_COMPID
            );
            //mlRawImuData.time_usec); // this is for a request

        // Copy the message to the send buffer
        bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);
    }

    // ***** Parameter State Machine ******
    evaluateParameterState(PARAM_EVENT_NONE, NULL);

    /* Set by evaluateParameterState() via _prepareTransmitParameter()
       when we're ready to transmit. */
    //if (mlPending.sensorDspReady) { // TODO check param request on startup
    if (mlPending.piTransaction == PARAM_TRANSACTION_SEND) {
        mavlink_msg_param_value_pack(SLUGS_SYSTEMID, SLUGS_COMPID, &msg,
            mlParamInterface.param_name[mlPending.piCurrentParameter],
            mlParamInterface.param[mlPending.piCurrentParameter],
            MAV_PARAM_TYPE_REAL32, // NOTE we only use floats for now
            PAR_PARAM_COUNT,
            mlPending.piCurrentParameter);

        mlPending.piTransaction = PARAM_TRANSACTION_NONE;
        if (++mlPending.piCurrentParameter >= PAR_PARAM_COUNT)
            mlPending.piCurrentParameter = 0;


        // Copy the message to the send buffer
        bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);
/*
        memset(vr_message,0,sizeof(vr_message));
        sprintf(vr_message, "P = %d, V=%f ", mlPending.piCurrentParameter,
            (float)mlParamInterface.param[mlPending.piCurrentParameter]);
        bytes2Send += sendQGCDebugMessage (vr_message, 0, dataOut, bytes2Send+1);
*/
    }

    // Command to send
    if (mlPending.command != FALSE) {
        // clear the msg
        memset(&msg, 0, sizeof (mavlink_message_t));

        mavlink_msg_command_long_encode(SLUGS_SYSTEMID,
            SLUGS_COMPID,
            &msg,
            &mlCommand);

        // Copy the message to the send buffer
        bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

        mlPending.command = 0;
    }

    // This block goes via SPI to the sensor DSC so it does not count
    // towards the Bandwith budget
    if (mlPending.spiSendGSLocation) {
        mavlink_msg_set_gps_global_origin_pack(SLUGS_SYSTEMID,
            SLUGS_COMPID,
            &msg,
            //GS_COMPID,
            GS_SYSTEMID,
            mlWpValues.lat[MAX_NUM_WPS - 1],
            mlWpValues.lon[MAX_NUM_WPS - 1],
            mlWpValues.alt[MAX_NUM_WPS - 1]);

        addMessageToSpiOut(&msg);

        memset(vr_message, 0, sizeof (vr_message));
        sprintf(vr_message, "Sensor DSC Reboot.");
        bytes2Send += sendQGCDebugMessage(vr_message, 255, dataOut, bytes2Send + 1);

        mlPending.spiSendGSLocation = 0;
    }

    // end of SPI block

    if (sw_debug == 1) {
        memset(vr_message, 0, sizeof (vr_message));
        sprintf(vr_message, "hla = %2.4f, hlo =%2.4f  hh = %2.2f", (double)mlWpValues.lat[MAX_NUM_WPS - 1], (double)mlWpValues.lon[MAX_NUM_WPS - 1], (double)mlWpValues.alt[MAX_NUM_WPS - 1]);
        bytes2Send += sendQGCDebugMessage(vr_message, 0, dataOut, bytes2Send + 1);
        sw_debug = 0;
    }
    //
    if (sw_debug == 4) {
        memset(vr_message, 0, sizeof (vr_message));
        sprintf(vr_message, "%d %d %d %d %d %d %d %d %d %d", sw_temp[0], sw_temp[1], sw_temp[2], sw_temp[3], sw_temp[4], sw_temp[5], sw_temp[6], sw_temp[7], sw_temp[8], sw_temp[9]);
        bytes2Send += sendQGCDebugMessage(vr_message, 0, dataOut, bytes2Send + 1);
        sw_debug = 0;

    }


    /****  New mission state machine code ******/

    evaluateMissionState(MISSION_EVENT_NONE, NULL);

    // TODO determine if we need this
    // clear the msg
    if (mlPending.miTransaction != MISSION_TRANSACTION_NONE) {
        memset(&msg, 0, sizeof (mavlink_message_t));

        /* Set by evaluateMissionState() when we're ready to transmit. */
        // Send current mission item number
        if (mlPending.miTransaction == MISSION_TRANSACTION_SEND_CURRENT) {
            mavlink_msg_mission_current_pack(SLUGS_SYSTEMID, SLUGS_COMPID, &msg,
                     ((uint16_t)mlNavigation.toWP) - 1);

            mlPending.miTransaction = MISSION_TRANSACTION_NONE;

            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);
        }
        // Send mission acknowledgement
        else if (mlPending.miTransaction == MISSION_TRANSACTION_SEND_ACK) {
            mavlink_msg_mission_ack_pack(SLUGS_SYSTEMID,
                    MAV_COMP_ID_MISSIONPLANNER,
                    &msg,
                    GS_SYSTEMID,
                    GS_COMPID,
                    mlPending.miAckType);

            mlPending.miTransaction = MISSION_TRANSACTION_NONE;

            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

    #ifdef DEBUG_MISSION_SM
            memset(vr_message,0,sizeof(vr_message));
            sprintf(vr_message, "Sent ack = %d", mlPending.miAckType);
            bytes2Send += sendQGCDebugMessage (vr_message, 0, dataOut, bytes2Send+1);
    #endif
        }
        // Send mission count
        else if (mlPending.miTransaction == MISSION_TRANSACTION_SEND_COUNT) {
            mavlink_msg_mission_count_pack(SLUGS_SYSTEMID, SLUGS_COMPID, &msg,
                GS_SYSTEMID, GS_COMPID, mlPending.miTotalMissions);

            mlPending.miTransaction = MISSION_TRANSACTION_NONE;

            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

    #ifdef DEBUG_MISSION_SM
            memset(vr_message,0,sizeof(vr_message));
            sprintf(vr_message, "Sent count = %d", mlPending.miTotalMissions);
            bytes2Send += sendQGCDebugMessage (vr_message, 0, dataOut, bytes2Send+1);
    #endif
        }
        // Send mission item
        else if (mlPending.miTransaction == MISSION_TRANSACTION_SEND_ITEM) {
            mavlink_msg_mission_item_pack(SLUGS_SYSTEMID,
                MAV_COMP_ID_MISSIONPLANNER,
                &msg,
                GS_SYSTEMID,
                GS_COMPID,
                mlPending.miCurrentMission,
                MAV_FRAME_GLOBAL,
                mlWpValues.type[mlPending.miCurrentMission],
                0, // not current
                1, // autocontinue
                0.0, // Param 1 not used
                0.0, // Param 2 not used
                (float) mlWpValues.orbit[mlPending.miCurrentMission],
                0.0, // Param 4 not used
                mlWpValues.lat[mlPending.miCurrentMission],
                mlWpValues.lon[mlPending.miCurrentMission],
                mlWpValues.alt[mlPending.miCurrentMission]); // always autocontinue

            mlPending.miTransaction = MISSION_TRANSACTION_NONE;

            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

    #ifdef DEBUG_MISSION_SM
            memset(vr_message,0,sizeof(vr_message));
            sprintf(vr_message, "Sent item = %d", mlPending.miCurrentMission);
            bytes2Send += sendQGCDebugMessage (vr_message, 0, dataOut, bytes2Send+1);
    #endif
        }
         // Send mission request
        else if (mlPending.miTransaction == MISSION_TRANSACTION_SEND_REQUEST
                && mlPending.miCurrentMission < mlPending.miTotalMissions) {

            mavlink_msg_mission_request_pack(SLUGS_SYSTEMID,
                MAV_COMP_ID_MISSIONPLANNER,
                &msg,
                GS_SYSTEMID,
                GS_COMPID,
                mlPending.miCurrentMission);

            mlPending.miTransaction = MISSION_TRANSACTION_NONE;
            // Increment pending
            // TODO remove this or not
            /*
            if (++mlPending.miCurrentMission >= mlPending.miTotalMissions)
                mlPending.miCurrentMission = 0;
             */
            // Copy the message to the send buffer
            bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

    #ifdef DEBUG_MISSION_SM
            memset(vr_message,0,sizeof(vr_message));
            sprintf(vr_message, "Sent request = %d", mlPending.miCurrentMission);
            bytes2Send += sendQGCDebugMessage (vr_message, 0, dataOut, bytes2Send+1);
    #endif
        }
    } // if mission SM

    if (mlPending.commandAck) {
        mavlink_msg_command_ack_encode(SLUGS_SYSTEMID,
            SLUGS_COMPID,
            &msg,
            &mlCommandAck);

        // Copy the message to the send buffer
        bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

        mlPending.commandAck = FALSE;
    }


    // if there is a pending request for the Mid Level Commands
    if (mlPending.midLvlCmds == 1) {
        // clear the msg
        memset(&msg, 0, sizeof (mavlink_message_t));

        mavlink_msg_mid_lvl_cmds_pack(SLUGS_SYSTEMID,
            SLUGS_COMPID,
            &msg,
            GS_SYSTEMID,
            mlMidLevelCommands.hCommand,
            mlMidLevelCommands.uCommand,
            mlMidLevelCommands.rCommand);

        // Copy the message to the send buffer
        bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

        // clear the flag
        mlPending.midLvlCmds = 0;
    }

    if (mlPending.isrLoc == 1) {
        // clear the msg
        memset(&msg, 0, sizeof (mavlink_message_t));

        mavlink_msg_isr_location_encode(SLUGS_SYSTEMID,
            SLUGS_COMPID,
            &msg,
            &mlISR);

        // Copy the message to the send buffer
        bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

        mlPending.isrLoc = 0;
    }

    // if the boot message is set then transmit it urgently
    if (mlBoot.version == 1) {
        memset(vr_message, 0, sizeof (vr_message));
        sprintf(vr_message, "%s DSC Reboot.", "Control");
        bytes2Send += sendQGCDebugMessage(vr_message, 255, dataOut, bytes2Send + 1);
        mlBoot.version = 0;
    }

    if (mlPending.connectionWarning && !mlPending.connectionWarningSent) {
         bytes2Send += sendQGCDebugMessage ("Connection failing. Pleaser restart GS radio.",
            0, dataOut, bytes2Send+1);
         mlPending.connectionWarningSent = 1;
    }

    // Send additional statustext messages
    if (mlPending.statustext > 0) {
        memset(&msg, 0, sizeof (mavlink_message_t));


        mavlink_msg_statustext_pack(SLUGS_SYSTEMID,
        SLUGS_COMPID,
        &msg,
        mlStatustext.severity,
        mlStatustext.text);

        bytes2Send += mavlink_msg_to_send_buffer((dataOut + 1 + bytes2Send), &msg);

        mlPending.statustext = 0;
    }


    /*
    memset(&msg, 0, sizeof (mavlink_message_t));

    mavlink_msg_attitude_encode(SLUGS_SYSTEMID,
        SLUGS_COMPID,
        &msg,
        &mlAttitudeRotated);
    // Copy the message to the send buffer
    // TODO  Log this instead
     * */


    // Put the length of the message in the first byte of the outgoing array
    *dataOut = bytes2Send;

    // increment/overflow the samplePeriod counter
    // configured for 10 Hz in non vital messages
    sampleTelemetry = (sampleTelemetry >= 50) ? 1 : sampleTelemetry + 1;
}
#endif

/**
 * Record mavlink messages to the on-board logger.
 */
static void _logTelemetryMavlink(void) {
#if defined(RECORD_TO_LOGGER)
    // Generic message container used to pack the messages
    mavlink_message_t msg;

    // Cycles from 1 to 10 
    static uint8_t sampleT elemetry = 1;

    switch (sampleTelemetry) {
        case 1: // GPS and Heartbeat
            // == GPS Raw ==
            memset(&msg, 0, sizeof (mavlink_message_t));
            mavlink_msg_gps_raw_int_pack(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
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
            uart1BytesToSend += mavlink_msg_to_send_buffer((uart1BufferOut + uart1BytesToSend), &msg);
            

            // == Heartbeat ==
            memset(&msg, 0, sizeof (mavlink_message_t));
            mavlink_msg_heartbeat_pack(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                MAV_TYPE_FIXED_WING,
                MAV_AUTOPILOT_SLUGS,//MAV_AUTOPILOT_GENERIC,
                mlHeartbeatLocal.base_mode,
                mlHeartbeatLocal.custom_mode,
                mlHeartbeatLocal.system_status
                );
            uart1BytesToSend += mavlink_msg_to_send_buffer((uart1BufferOut + uart1BytesToSend), &msg);

            break; //case 1

        case 2: // GPS Date Time, diagnostic, air data,
            // == GPS Date Time ==
            memset(&msg, 0, sizeof (mavlink_message_t));
            mavlink_msg_gps_date_time_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlGpsDateTime);
            uart1BytesToSend += mavlink_msg_to_send_buffer((uart1BufferOut + uart1BytesToSend), &msg);

            // == Diagnotic ==
            memset(&msg, 0, sizeof (mavlink_message_t));
            mavlink_msg_diagnostic_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlDiagnosticData);
            uart1BytesToSend += mavlink_msg_to_send_buffer((uart1BufferOut + uart1BytesToSend), &msg);

            // == Scaled Pressure (Air Data) ==
            memset(&msg, 0, sizeof (mavlink_message_t));
            mavlink_msg_scaled_pressure_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlAirData);
            uart1BytesToSend += mavlink_msg_to_send_buffer((uart1BufferOut + uart1BytesToSend), &msg);

            break; // case 2

        case 3: // data log, ping, vfr_hud
            // == Data Log (more diagnostics) ==
            memset(&msg, 0, sizeof (mavlink_message_t));
            mavlink_msg_data_log_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlDataLog);
            uart1BytesToSend += mavlink_msg_to_send_buffer((uart1BufferOut + uart1BytesToSend), &msg);

            // == VFR Hud ==
            // TODO determine if this is needed in logs
            memset(&msg, 0, sizeof (mavlink_message_t));
            mavlink_msg_vfr_hud_pack(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                mlNavigation.u_m,               // air speed (m/s)
                (float)mlGpsData.vel * 0.01f,   // ground speed (m/s)
                0.0f,
                0,                              // throttle from 0 to 100 (percent)
                mlLocalPositionData.z,          // altitude (m)
                mlLocalPositionData.vz         // climb rate (m/s)
                );
            uart1BytesToSend += mavlink_msg_to_send_buffer((uart1BufferOut + uart1BytesToSend), &msg);

            break; // case 3

        case 4: // navigation, cpu load, sensor diag
            // == SLUGS Navigation ==
            memset(&msg, 0, sizeof (mavlink_message_t));
            mavlink_msg_slugs_navigation_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlNavigation);
            uart1BytesToSend += mavlink_msg_to_send_buffer((uart1BufferOut + uart1BytesToSend), &msg);

            // == CPU Load ==
            memset(&msg, 0, sizeof (mavlink_message_t));
            mavlink_msg_cpu_load_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlCpuLoadData);
            uart1BytesToSend += mavlink_msg_to_send_buffer((uart1BufferOut + uart1BytesToSend), &msg);

             // == Sensor Diagnostics ==
            memset(&msg, 0, sizeof (mavlink_message_t));
            mavlink_msg_sensor_diag_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlSensorDiag);
            uart1BytesToSend += mavlink_msg_to_send_buffer((uart1BufferOut + uart1BytesToSend), &msg);

            break; // case 4

        case 5: // Raw IMU
            // == Raw IMU ==
            memset(&msg, 0, sizeof (mavlink_message_t));
            mavlink_msg_raw_imu_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlRawImuData);
            uart1BytesToSend += mavlink_msg_to_send_buffer((uart1BufferOut + uart1BytesToSend), &msg);

            break; // case 5

        case 6: // Local Position, System Status, GPS Status
            // == Local Position NED ==
            memset(&msg, 0, sizeof (mavlink_message_t));
            mavlink_msg_local_position_ned_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlLocalPositionData);
            uart1BytesToSend += mavlink_msg_to_send_buffer((uart1BufferOut + uart1BytesToSend), &msg);

            // == System Status ==
            memset(&msg, 0, sizeof (mavlink_message_t));
            mavlink_msg_sys_status_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlSystemStatus);
            uart1BytesToSend += mavlink_msg_to_send_buffer((uart1BufferOut + uart1BytesToSend), &msg);

            // == GPS Status or Novatel Diagnostic ==
            memset(&msg, 0, sizeof (mavlink_message_t));
#if USE_NMEA
            mavlink_msg_status_gps_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlGpsStatus);
#else
            mavlink_msg_novatel_diag_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlNovatelStatus);
#endif
            uart1BytesToSend += mavlink_msg_to_send_buffer((uart1BufferOut + uart1BytesToSend), &msg);

            break; // case 6

        case 7: // PWM Commands, Biases, slugs action
            // == Servo Output Raw ==
            memset(&msg, 0, sizeof (mavlink_message_t));
            mavlink_msg_servo_output_raw_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlPwmCommands);
            uart1BytesToSend += mavlink_msg_to_send_buffer((uart1BufferOut + uart1BytesToSend), &msg);

            // == Sensor Bias ==
            memset(&msg, 0, sizeof (mavlink_message_t));
            mavlink_msg_sensor_bias_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlSensorBiasData);
            uart1BytesToSend += mavlink_msg_to_send_buffer((uart1BufferOut + uart1BytesToSend), &msg);

            break; // case 7

        case 8:// raw pressure
             // == Raw Pressure ==
            memset(&msg, 0, sizeof (mavlink_message_t));
            mavlink_msg_raw_pressure_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlRawPressureData);
            uart1BytesToSend += mavlink_msg_to_send_buffer((uart1BufferOut + uart1BytesToSend), &msg);

            break; // case 8

        case 9: // Action Ack, Pilot Console, Mid Level Commands, boot
            // == RC Channels Raw ==
            memset(&msg, 0, sizeof (mavlink_message_t));
            mavlink_msg_rc_channels_raw_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlPilotConsoleData);
            uart1BytesToSend += mavlink_msg_to_send_buffer((uart1BufferOut + uart1BytesToSend), &msg);

            break; // case 9

        case 10: // Filtered data, PTZ data, VI Sensor
            // == Scaled IMU ==
            memset(&msg, 0, sizeof (mavlink_message_t));
            mavlink_msg_scaled_imu_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlFilteredData);
            uart1BytesToSend += mavlink_msg_to_send_buffer((uart1BufferOut + uart1BytesToSend), &msg);

            // == PTZ Status (from camera) ==
            /*
            memset(&msg, 0, sizeof (mavlink_message_t));

            mavlink_msg_ptz_status_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlPtzStatus);
            uart1BytesToSend += mavlink_msg_to_send_buffer((uart1BufferOut + uart1BytesToSend), &msg);
            */

            // == Voltage Sensor ==
            memset(&msg, 0, sizeof (mavlink_message_t));
            mavlink_msg_volt_sensor_encode(SLUGS_SYSTEMID,
                SLUGS_COMPID,
                &msg,
                &mlVISensor);
            uart1BytesToSend += mavlink_msg_to_send_buffer((uart1BufferOut + uart1BytesToSend), &msg);

            break; // case 10

    } // Switch

    // == Attitude (50 Hz) ==
    memset(&msg, 0, sizeof (mavlink_message_t));
    mavlink_msg_attitude_encode(SLUGS_SYSTEMID,
        SLUGS_COMPID,
        &msg,
        &mlAttitudeRotated);
    uart1BytesToSend += mavlink_msg_to_send_buffer((uart1BufferOut + uart1BytesToSend), &msg);

    // increment/overflow the samplePeriod counter
    sampleTelemetry = (sampleTelemetry >= 10) ? 1 : sampleTelemetry + 1;

    // Send the data
    _uart1OutputToDMA1(uart1BytesToSend);

#endif
} // _logTelemetryMavlink()

/**
 * Decodes incoming MAVLink messages.
 *
 * @note Messages come from either the groundstation or from sensor DSC via IPC
 * @param dataIn buffer with input data
 */ 
void protDecodeMavlink(uint8_t* dataIn) {

    uint8_t i, indx, writeSuccess, commChannel = dataIn[MAXSPI + 1];
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

             uint8_t id = msg.msgid;
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
                        mlPending.spiSendGSLocation = 1;
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

                        if (!hasMode(mlHeartbeatLocal.base_mode,MAV_MODE_FLAG_HIL_ENABLED)
                            && hasMode(mlSetMode.base_mode, MAV_MODE_FLAG_HIL_ENABLED)) {
                            // Turned HIL on
                            mlPending.statustext++;

                            mlStatustext.severity = MAV_SEVERITY_INFO;
                            strncpy(mlStatustext.text, "Turning on HIL mode.", 49);
                        }
                        else if (hasMode(mlHeartbeatLocal.base_mode, MAV_MODE_FLAG_HIL_ENABLED)
                            && !hasMode(mlSetMode.base_mode,MAV_MODE_FLAG_HIL_ENABLED)) {
                            // Turned HIL off
                            mlPending.statustext++;

                            mlStatustext.severity = MAV_SEVERITY_INFO;
                            strncpy(mlStatustext.text, "Turning off HIL mode.", 49);
                        }
                        mlHeartbeatLocal.base_mode = mlSetMode.base_mode;

                    }
                    if (mlSetMode.custom_mode != SLUGS_MODE_NONE) {
                        BOOL badMode = FALSE;
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
                                badMode = TRUE;
                                // Send an error message
                                mlPending.statustext++;
                                mlStatustext.severity = MAV_SEVERITY_ERROR;
                                sprintf(mlStatustext.text, "Unknown slugs mode sent: %X.", mlSetMode.custom_mode);
                                break;
                        } // switch (mlSetMode.custom_mode)
                        // Set new slugs mode if valid
                        if (!badMode) {
                            mlHeartbeatLocal.custom_mode = mlSetMode.custom_mode;
                        }
                    } // if (mlSetMode.custom_mode != SLUGS_MODE_NONE)
                    break;
                }
                case MAVLINK_MSG_ID_MID_LVL_CMDS:
                {
                    mavlink_msg_mid_lvl_cmds_decode(&msg, &mlMidLevelCommands);

                    // Report the Change
                    mlPending.statustext++;
                    mlStatustext.severity = MAV_SEVERITY_INFO;
                    strncpy(mlStatustext.text, "Mid level flight parameters received.", 49);
                    mlPending.midLvlCmds = 1;

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
                // Got a new home location, forward it to sensor DSC
                case MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN:
                    writeSuccess = SUCCESS;

                    memset(&mlSingleWp, 0, sizeof (mavlink_mission_item_t));
                    mavlink_msg_set_gps_global_origin_decode(&msg, &mlGSLocation);
                    addMessageToSpiOut(&msg);   

                    mlSingleWp.x = (float) (mlGSLocation.latitude);
                    mlSingleWp.y = (float) (mlGSLocation.longitude);
                    mlSingleWp.z = (float) (mlGSLocation.altitude);

                    indx = (uint8_t) MAX_NUM_WPS - 1;

                    mlWpValues.lat[indx] = mlSingleWp.x;
                    mlWpValues.lon[indx] = mlSingleWp.y;
                    mlWpValues.alt[indx] = mlSingleWp.z;
                    mlWpValues.type[indx] = MAV_CMD_NAV_LAND;
                    mlWpValues.orbit[indx] = 0;

                    // Record the data in EEPROM
                    writeSuccess = storeWaypointInEeprom(&mlSingleWp);

                    // Send a message with result
                    if (writeSuccess != SUCCESS) {
                        mlPending.statustext++;
                        mlStatustext.severity = MAV_SEVERITY_ERROR;
                        strncpy(mlStatustext.text, "Failed to write origin to EEPROM.", 49);
                    }
                    else {
                        mlPending.statustext++;
                        mlStatustext.severity = MAV_SEVERITY_INFO;
                        strncpy(mlStatustext.text, "Control DSC origin saved to EEPROM.", 49);
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
                            mlPending.statustext++;
                            mlStatustext.severity = MAV_SEVERITY_INFO;
                            strncpy(mlStatustext.text, "Sensor DSC origin set.", 49);
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
                            writeSuccess = FAILURE;
                            // Parameter storage
                            if (mlCommand.param1 == 0.0f) { // read
                                memset(&(mlParamInterface.param[0]), 0, sizeof (float) *PAR_PARAM_COUNT);
                                writeSuccess = readParamsInEeprom();
                            }
                            else if (mlCommand.param1 == 1.0f) { // write
                                writeSuccess = storeAllParamsInEeprom();
                            }
                            // Waypoint storage (only handle either params or waypoints)
                            // TODO look into implementing this (again?)
                            else if (mlCommand.param2 == 0.0f) { // read
                            }
                            else if (mlCommand.param2 == 1.0f) { // write
                            }

                            mlPending.commandAck = TRUE;
                            mlCommandAck.command = MAV_CMD_PREFLIGHT_STORAGE;
                            mlCommandAck.result = (writeSuccess == SUCCESS)?
                                MAV_RESULT_ACCEPTED : MAV_RESULT_FAILED;
                            break;

                        // Store or read mid-level commands to/from eeprom
                        case MAV_CMD_MIDLEVEL_STORAGE:
                            writeSuccess = FAILURE;
                            if (mlCommand.param1 == 0.0f) { // read
                                mlMidLevelCommands.hCommand = 0.0f;
                                mlMidLevelCommands.rCommand = 0.0f;
                                mlMidLevelCommands.uCommand = 0.0f;
                                writeSuccess = readMidLevelCommandsInEeprom();
                            }
                            else if (mlCommand.param1 == 1.0f) { // write
                                writeSuccess = storeMidLevelCommandsInEeprom();
                                mlPending.statustext++;

                                mlStatustext.severity = MAV_SEVERITY_INFO;
                                strncpy(mlStatustext.text, "Wrote mid-level commands to EEPROM.", 49);
                            }

                            mlPending.commandAck = TRUE;
                            mlCommandAck.command = MAV_CMD_MIDLEVEL_STORAGE;
                            mlCommandAck.result = (writeSuccess == SUCCESS)?
                                MAV_RESULT_ACCEPTED : MAV_RESULT_FAILED;

                            break;

                        case MAV_CMD_RETURN_TO_BASE:
                            mlRTB.rtb = TRUE;
                            mlRTB.track_mobile = mlCommand.param1;
                            mlPending.commandAck = TRUE;
                            mlCommandAck.command = MAV_CMD_RETURN_TO_BASE;
                            mlCommandAck.result = MAV_RESULT_ACCEPTED;
                            break;

                        case MAV_CMD_TURN_LIGHT:
                            mlLights.state = mlCommand.param2;
                            mlLights.type = mlCommand.param1;

                            mlPending.commandAck = TRUE;
                            mlCommandAck.command = MAV_CMD_TURN_LIGHT;
                            mlCommandAck.result = MAV_RESULT_ACCEPTED;
                            break;
                        case MAV_CMD_GET_MID_LEVEL_COMMANDS:
                            mlPending.midLvlCmds = 1;

                            mlPending.commandAck = TRUE;
                            mlCommandAck.command = MAV_CMD_GET_MID_LEVEL_COMMANDS;
                            mlCommandAck.result = MAV_RESULT_ACCEPTED;
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

                    mlPending.statustext++;
                    mlStatustext.severity = MAV_SEVERITY_INFO;
                    strncpy(mlStatustext.text, "ISR position recieved.", 49);

                    mlPending.isrLoc = 1;
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
                mlPending.miCurrentMission = 0;
                nextState = MISSION_STATE_SEND_MISSION_COUNT;
            }                // Otherwise if a mission count was received, prepare to receive new missions.
            else if (event == MISSION_EVENT_COUNT_RECEIVED) {
                // Don't allow for writing of new missions if we're in autonomous mode.
                // TODO determine if we need this (safety feature)
                if (hasMode(mlHeartbeatLocal.base_mode, MAV_MODE_FLAG_AUTO_ENABLED)) {
                    _prepareTransmitMissionAck(MAV_MISSION_ERROR);
                    nextState = MISSION_STATE_INACTIVE;
                    // break; // stop handling count received (boat code missing this)
                }
                uint8_t newListSize = *(uint8_t *) data;

                // If we received a 0-length mission list, just respond with a MISSION_ACK error.
                if (newListSize == 0) {
                    _prepareTransmitMissionAck(MAV_MISSION_ERROR);
                    nextState = MISSION_STATE_INACTIVE;
                }                    // If there isn't enough room, respond with a MISSION_ACK error.
                else if (newListSize > MAX_NUM_WPS) {
                    _prepareTransmitMissionAck(MAV_MISSION_NO_SPACE);
                    nextState = MISSION_STATE_INACTIVE;
                }                    // Otherwise we're set to start retrieving a new mission list so we request the first mission.
                else {
                    // Update the size of the mission list to the new list size.
                    mavlinkNewMissionListSize = newListSize;

                    // Clear all the old waypoints.
                    clearMissionList();


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
                if (hasMode(mlHeartbeatLocal.base_mode, MAV_MODE_FLAG_AUTO_ENABLED)) {
                    _prepareTransmitMissionAck(MAV_MISSION_ERROR);
                    nextState = MISSION_STATE_INACTIVE;
                }                    // But if we're in manual mode, go ahead and clear everything.
                else {
                    // Clear the old list
                    clearMissionList();

                    // TODO determine if we need this
                    // Update the starting point to the vehicle's current location
                    //SetStartingPointToCurrentLocation();

                    // And then send our acknowledgement.
                    _prepareTransmitMissionAck(MAV_MISSION_ACCEPTED);
                    nextState = MISSION_STATE_INACTIVE;
                }
            } else if (event == MISSION_EVENT_SET_CURRENT_RECEIVED) {
                // TODO implement this feature
                /*
                    SetCurrentMission(*(uint8_t*)data);
                    _prepareTransmitCurrentMission();
                 **/
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
                    int missionAddStatus = addMission(incomingMission);
                    if (missionAddStatus == SUCCESS) {
                        // TODO decide whether to implement this
                        // If this is going to be the new current mission, then we should set it as such.
                        /*
                        if (incomingMission->current) {
                            SetCurrentMission(incomingMission->seq);
                        }
                        */

                        // If this was the last mission we were expecting, respond with an ACK
                        // confirming that we've successfully received the entire mission list.
                        if (mlPending.miCurrentMission == mavlinkNewMissionListSize - 1) {
                            _prepareTransmitMissionAck(MAV_MISSION_ACCEPTED);
                            nextState = MISSION_STATE_INACTIVE;
                        }
                        // Otherwise we just increment and request the next mission.
                        else {
                            mlPending.miCurrentMission++;
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
                    int missionAddStatus = addMission(incomingMission);
                    if (missionAddStatus != -1) {

                        // TODO decide whether to implement this
                        // If this is going to be the new current mission, then we should set it as such.
                        /*
                        if (incomingMission->current) {
                            SetCurrentMission(incomingMission->seq);
                        }
                        */

                        // If this was the last mission we were expecting, respond with an ACK
                        // confirming that we've successfully received the entire mission list.
                        if (mlPending.miCurrentMission == mavlinkNewMissionListSize - 1) {
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
                    int missionAddStatus = addMission(incomingMission);
                    if (missionAddStatus != -1) {

                        // TODO decide whether to implement this
                        // If this is going to be the new current mission, then we should set it as such.
                        /*
                        if (incomingMission->current) {
                            SetCurrentMission(incomingMission->seq);
                        }
                        */

                        // If this was the last mission we were expecting, respond with an ACK
                        // confirming that we've successfully received the entire mission list.
                        if (mlPending.miCurrentMission  == mavlinkNewMissionListSize - 1) {
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

/**
 * Copies the UART1 output buffer to the DMA1 buffer for sending.
 * @param size in bytes to copy to the DMA1 buffer.
 */
static void _uart1OutputToDMA1(uint16_t size) {
#ifdef RECORD_TO_LOGGER
    uint16_t i;
    for (i = 0; i < size; i++) {
        dma1Buffer[i] = (unsigned int) readFront(uart1BufferOut); //[i];
    }
#endif
}

/**
 * Copies the UART2 output buffer to the DMA1 buffer for sending.
 * @param size in bytes to copy to the DMA1 buffer.
 */
void copyBufferToDMA1(unsigned char size) {
    unsigned char i;
    for (i = 0; i < size; i += 1) {
        dma2Buffer[i] = (unsigned int) readFront(uart2BufferOut);
    }
}

// **** Interrupt service routines ****
void __attribute__((interrupt, no_auto_psv)) _DMA1Interrupt(void) {
    // Clear the DMA1 Interrupt Flag;
    IFS0bits.DMA1IF = 0;
}
/*
void __attribute__((interrupt, no_auto_psv)) _DMA1Interrupt(void) {
    // Clear the DMA1 Interrupt Flag;
    IFS1bits.DMA1IF = 0;
}
 * */

void __attribute__((__interrupt__, no_auto_psv)) _U2RXInterrupt(void) {
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

void __attribute__((interrupt, no_auto_psv)) _U2ErrInterrupt(void) {

    // If there was an overun error clear it and continue
    if (U2STAbits.OERR == 1) {
        U2STAbits.OERR = 0;
    }

    // If there was an overun error clear it and continue
    if (IFS4bits.U2EIF == 1) {
        IFS4bits.U2EIF = 0; // Clear the UART2 Error Interrupt Flag
    }
}


