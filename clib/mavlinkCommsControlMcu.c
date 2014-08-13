
#include <math.h>
#include "mavlinkCommsControlMcu.h"

//#define DEBUG_MISSION_SM  // comment this out to disable mission SM debugging messages

/*

MAVLink supports the following UAV Modes:

enum MAV_MODE
{
    MAV_MODE_UNINIT = 0,     ///< System is in undefined state
    MAV_MODE_LOCKED = 1,     ///< Motors are blocked, system is safe
    MAV_MODE_MANUAL = 2,     ///< System is allowed to be active, under manual (RC) control
    MAV_MODE_GUIDED = 3,     ///< System is allowed to be active, under autonomous control, manual setpoint
    MAV_MODE_AUTO =   4,     ///< System is allowed to be active, under autonomous control and navigation
    MAV_MODE_TEST1 =  5,     ///< Generic test mode, for custom use
    MAV_MODE_TEST2 =  6,     ///< Generic test mode, for custom use
    MAV_MODE_TEST3 =  7,     ///< Generic test mode, for custom use
    MAV_MODE_READY =  8,     ///< System is ready, motors are unblocked, but controllers are inactive
    MAV_MODE_RC_TRAINING = 9 ///< System is blocked, only RC valued are read and reported back
};

SLUGS uses two of these, MANUAL and GUIDED. The former, renders the navigation
modes (see navSupport.c) irrelevant which is when the safety pilot controls
the aircraft from the ground. In the other (GUIDED) it is the autopilot
who is in control.

MAVLink also supports a set of states:

enum MAV_STATE
{
    MAV_STATE_UNINIT = 0,
    MAV_STATE_BOOT,
    MAV_STATE_CALIBRATING,
    MAV_STATE_STANDBY,
    MAV_STATE_ACTIVE,
    MAV_STATE_CRITICAL,
    MAV_STATE_EMERGENCY,
    MAV_STATE_HILSIM,
    MAV_STATE_PILOT,
    MAV_STATE_POWEROFF
};

SLUGS uses two of these as follows:

MAV_STATE_ACTIVE: The autopilot is reading data from the real sensors onboard

MAV_STATE_HILSIM: The autopilot is in HIL mode and thus reading data from the
                                    HIL sim serial port in the sensor MCU.

 */
// Set up state machine variables for the mission protocol
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


// Define a timeout (in units of main timesteps of MavlinkReceive()) for transmitting
// MAVLink messages as part of the PARAMETER and MISSION protocols. Messages will be retransmit
// twice before it's considered hopeless.
#define MAVLINK_RESEND_TIMEOUT 300

// Specify how long between transmitting parameters in a parameter transmission stream.
#define INTRA_PARAM_DELAY 1

void _prepareTransmitParameter(uint16_t id);
void _prepareTransmitCurrentMission(void);
void _prepareTransmitMissionAck(uint8_t type);
void _prepareTransmitMissionCount(void);
void _prepareTransmitMissionItem(uint8_t currentMissionIndex);
void _prepareTransmitMissionRequest(uint8_t currentMissionIndex);

struct CircBuffer com2BufferIn;
CBRef uartBufferIn;


struct CircBuffer com1BufferOut;
CBRef uartBufferOut;

unsigned int BufferB[MAXSEND] __attribute__((space(dma))) = {0};

char sw_debug;
char sw_temp[50];
char sw_intTemp;
float fl_temp1, fl_temp2;

// UART, DMA and Buffer initialization

void uart2Init(void) {
    sw_debug = 0;

    // initialize the circular buffers
    uartBufferIn = (struct CircBuffer*) &com2BufferIn;
    newCircBuffer(uartBufferIn);

    uartBufferOut = (struct CircBuffer*) &com1BufferOut;
    newCircBuffer(uartBufferOut);


    // DMA1REQ Register
    // ================
    DMA1REQ = 0x001F;

    // DMA1PAD Register
    // ================
    DMA1PAD = (volatile unsigned int) &U2TXREG;

    // DMA1CON Register
    // ================
    DMA1CONbits.AMODE = 0; // Register Indirect with post-increment
    DMA1CONbits.MODE = 1; // One-shot, No Ping-Pong Mode	
    DMA1CONbits.DIR = 1; // Read from RAM and send to Periphereal
    DMA1CONbits.SIZE = 0; // Word Data Transfer

    // DMA1CNT Register
    // ==============
    DMA1CNT = MAXSEND - 1;

    // DMA1STA Register
    // ================
    DMA1STA = __builtin_dmaoffset(BufferB);

    // Enable DMA1 TX interrupts
    IFS0bits.DMA1IF = 0; // Clear DMA Interrupt Flag
    IPC3bits.DMA1IP = 6; // interrupt priority to 6
    IEC0bits.DMA1IE = 1; // Enable DMA interrupt

    // Configure and open the port;
    // U2MODE Register
    // ==============
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

    // U1STA Register
    // ==============
    U2STAbits.UTXISEL0 = 0; // generate interrupt on every char
    U2STAbits.UTXISEL1 = 0; // for the DMA	
    U2STAbits.URXISEL = 0; // RX interrupt when a char is in
    U2STAbits.OERR = 0; // clear overun error

    // U1BRG Register
    // ==============
    U2BRG = LOG_UBRG; // Set the baud rate for 115,200

    // Initialize the Interrupt  
    // ========================
    IPC7bits.U2RXIP = 7; // Interrupt priority 7  
    IFS1bits.U2RXIF = 0; // Clear the interrupt flag
    IEC1bits.U2RXIE = 1; // Enable interrupts

    // Enable the port;
    U2MODEbits.UARTEN = 1; // Enable the port	
    U2STAbits.UTXEN = 1; // Enable TX

    IEC4bits.U2EIE = 1;

}

void gsRead(unsigned char* gsChunk) {
    // fix the data length so if the interrupt adds data
    // during execution of this block, it will be read
    // until the next gsRead
    unsigned int tmpLen = getLength(uartBufferIn), i = 0;

    // if the buffer has more data than the max size, set it to max,
    // otherwise set it to the length
    gsChunk[0] = (tmpLen > MAXSPI) ? MAXSPI : tmpLen;

    // read the data 
    for (i = 1; i <= gsChunk[0]; i += 1) {
        gsChunk[i] = readFront(uartBufferIn);
    }

    gsChunk[MAXSPI + 1] = 1;
}

/* Called at 50 Hz by Simulink. */
/* Full telemetry version. */
#ifdef ENABLE_FULL_TELEMETRY
void prepareTelemetryMavlink(unsigned char* dataOut) {

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

void protDecodeMavlink(uint8_t* dataIn) {

    uint8_t i, indx, writeSuccess, commChannel = dataIn[MAXSPI + 1];
    uint32_t temp;
    //mavlink_param_set_t set;
    //mavlink_set_nav_mode_t mode;


    // TODO implement new mission state machine
    // Track if a mission message was processed in this call. This is used to determine if a
    // NONE_EVENT should be sent to the mission manager. The manager needs to be called every
    // timestep such that its internal state machine works properly.
    //BOOL processedMissionMessage = FALSE;

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
                        /*
                        BOOL badMode = FALSE;
                        // Note: some modes are unused. consider pruning
                        switch (mlSetMode.custom_mode) {
                            case SLUGS_MODE_LIFTOFF:
                                break;
                            case SLUGS_MODE_PASSTHROUGH:
                                mlPending.pt = TRUE; // want passthrough mode
                                break;
                            case SLUGS_MODE_WAYPOINT:
                                break;
                            case SLUGS_MODE_MID_LEVEL:
                                break;
                            case SLUGS_MODE_RETURNING:
                                break;
                            case SLUGS_MODE_LANDING:
                                break;
                            case SLUGS_MODE_LOST:
                                break;
                            case SLUGS_MODE_SELECTIVE_PASSTHROUGH:
                                break;
                            case SLUGS_MODE_ISR:
                                break;
                            case SLUGS_MODE_LINE_PATROL:
                                break;
                            default:
                                badMode = TRUE;
                                // Send an error message
                                mlPending.statustext++;

                                mlStatustext.severity = MAV_SEVERITY_ERROR;
                                sprintf(mlStatustext.text, "Unknown slugs mode sent: %X.", mlSetMode.custom_mode)
                                strncpy(mlStatustext.text,"Failed to clear waypoints from EEPROM.", 49);

                        }
                        // Set new slugs mode if valid
                        if (!badMode)
                        */
                        //lastNavigationMode = mlHeartbeatLocal.custom_mode;
                        mlHeartbeatLocal.custom_mode = mlSetMode.custom_mode;
                    }

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
                /********** New mission state machine code. ***********/

                // If we are not doing any mission protocol operations, record the size of the incoming mission
                // list and transition into the write missions state machine loop.
                case MAVLINK_MSG_ID_MISSION_COUNT: {
                    uint8_t mavlinkNewMissionListSize = mavlink_msg_mission_count_get_count(&msg);
                    evaluateMissionState(MISSION_EVENT_COUNT_RECEIVED, &mavlinkNewMissionListSize);
                    processedMissionMessage = TRUE;
#ifdef DEBUG_MISSION_SM
                    mlPending.statustext++;
                    mlStatustext.severity = MAV_SEVERITY_INFO;
                    sprintf(mlStatustext.text,"Got count %d.", mavlinkNewMissionListSize);
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

                // Responding to a mission request entails moving into the first active state and scheduling a MISSION_COUNT message.
                // Will also schedule a transmission of a GPS_ORIGIN message. This is used for translating global to local coordinates
                // in QGC.
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

                // When a mission request message is received, respond with that mission information from the MissionManager
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

                // Allow for clearing waypoints. Here we respond simply with an ACK message if we successfully
                // cleared the mission list.
                case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
                    evaluateMissionState(MISSION_EVENT_CLEAR_ALL_RECEIVED, NULL);
                    processedMissionMessage = TRUE;
                break;

                // Allow for the groundstation to set the current mission. This requires a WAYPOINT_CURRENT response message agreeing with the received current message index.
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
                /*
                 * Replaced mission state machine code.
                 *
                case MAVLINK_MSG_ID_MISSION_COUNT:

                    if (!mlPending.wpTransaction && (mlPending.wpProtState == WP_PROT_IDLE)) {


                        mavlink_msg_mission_count_decode(&msg, &mlWpCount);

                        // Start the transaction
                        mlPending.wpTransaction = 1;

                        // change the state	
                        mlPending.wpProtState = WP_PROT_GETTING_WP_IDLE;

                        // reset the rest of the state machine
                        mlPending.wpTotalWps = mlWpCount.count;
                        mlPending.wpCurrentWpInTransaction = 0;
                        mlPending.wpTimeOut = 0;
                    }

                    break;

                case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:

                    // if there is no transaction going on
                    if (!mlPending.wpTransaction && (mlPending.wpProtState == WP_PROT_IDLE)) {
                        // Start the transaction
                        mlPending.wpTransaction = 1;

                        // change the state
                        mlPending.wpProtState = WP_PROT_LIST_REQUESTED;



                        // reset the rest of the state machine
                        mlPending.wpCurrentWpInTransaction = 0;
                        mlPending.wpTimeOut = 0;
                    }
                    break;

                case MAVLINK_MSG_ID_MISSION_REQUEST:
                    mavlink_msg_mission_request_decode(&msg, &mlWpRequest);

                    if (mlPending.wpTransaction && (mlWpRequest.seq < mlWpValues.wpCount)) {
                        // change the state
                        mlPending.wpProtState = WP_PROT_TX_WP;

                        // reset the rest of the state machine
                        mlPending.wpCurrentWpInTransaction = mlWpRequest.seq;
                        mlPending.wpTimeOut = 0;
                    } else {
                        // TODO: put here a report for a single WP, i.e. not inside a transaction
                    }
                    break;

                case MAVLINK_MSG_ID_MISSION_ACK:
                    mavlink_msg_mission_ack_decode(&msg, &mlWpAck);

                    if (mlPending.wpTransaction) {
                        // End the transaction
                        mlPending.wpTransaction = 0;

                        // change the state
                        mlPending.wpProtState = WP_PROT_IDLE;

                        // reset the rest of the state machine
                        mlPending.wpCurrentWpInTransaction = 0;
                        mlPending.wpTimeOut = 0;

                        // send current waypoint index
                        mlPending.wpSendCurrent = TRUE;
                    }

                    break;

                case MAVLINK_MSG_ID_MISSION_ITEM:
                    writeSuccess = SUCCESS;
                    mavlink_msg_mission_item_decode(&msg, &mlSingleWp);

                    if (mlPending.wpTransaction && (mlPending.wpProtState == WP_PROT_RX_WP)) {
                        mlPending.wpProtState = WP_PROT_GETTING_WP_IDLE;

                    }

                    indx = (uint8_t) mlSingleWp.seq;

                    mlWpValues.lat[indx] = mlSingleWp.x;
                    mlWpValues.lon[indx] = mlSingleWp.y;
                    mlWpValues.alt[indx] = mlSingleWp.z;

                    mlWpValues.type[indx] = mlSingleWp.command;

                    mlWpValues.orbit[indx] = (uint16_t) mlSingleWp.param3;

                    // Record the data in EEPROM
                    writeSuccess = storeWaypointInEeprom(&mlSingleWp);

                    // Set the flag of Aknowledge for the AKN Message
                    // if the write was not successful
                    if (writeSuccess != SUCCESS) {
                        mlPending.wpAck++;

                        //mlWpAck.target_system =
                        mlWpAck.target_component = MAV_COMP_ID_MISSIONPLANNER;
                        mlWpAck.type = MAV_MISSION_ERROR;
                    }

                    break;

                case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:

                    writeSuccess = SUCCESS;

                    // clear the WP values in memory;
                    memset(&mlWpValues, 0, sizeof (mavlink_mission_item_values_t));

                    writeSuccess = clearWaypointsFrom(0);

                    // Set the flag of Aknowledge fail
                    // if the write was unsuccessful
                    if (writeSuccess != SUCCESS) {
                        mlPending.statustext++;

                        mlStatustext.severity = MAV_SEVERITY_ERROR;
                        strncpy(mlStatustext.text, "Failed to clear waypoints from EEPROM.", 49);

                    }


                    // Update the waypoint count
                    mlWpValues.wpCount = 0;

                    // Set the state machine ready to send the WP akn
                    mlPending.wpCurrentWpInTransaction = 0;
                    mlPending.wpTotalWps = 0;
                    mlPending.wpTransaction = 1;
                    mlPending.wpProtState = WP_PROT_GETTING_WP_IDLE;

                    break;
                    */

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

                    if (writeSuccess != SUCCESS) {
                        mlPending.statustext++;

                        mlStatustext.severity = MAV_SEVERITY_ERROR;
                        strncpy(mlStatustext.text, "Failed to write origin to EEPROM.", 49);
                    }
                    else {

                        mlPending.statustext++;

                        mlStatustext.severity = MAV_SEVERITY_INFO;
                        strncpy(mlStatustext.text, "Control DSC GPS origin set.", 49);
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


                case MAVLINK_MSG_ID_COMMAND_ACK:
                    // Received a command ack from the sensor MCU
                    mavlink_msg_command_ack_decode(&msg, &mlCommandAck);

                    switch (mlCommandAck.command) {
                        case MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN:
                            mlPending.statustext++;

                            mlStatustext.severity = MAV_SEVERITY_INFO;
                            strncpy(mlStatustext.text, "Sensor DSC GPS origin set.", 49);
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
                        // HIL Stuff Moved to set_mode
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
        } // if

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
 * The following is an internal helper function for the parameter state machine
 * that queues a parameter to be transmitted.
 * @param id The ID of this parameter.
 */
void _prepareTransmitParameter(uint16_t id)
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
 * The following is an internal helper function for the mission state machine
 * that queues the current mission number to be sent.
 */
void _prepareTransmitCurrentMission(void)
{
    mlPending.miTransaction = MISSION_TRANSACTION_SEND_CURRENT;
}

/**
* Transmit a mission acknowledgement message. The type of message is the sole argument to this
* function (see enum MAV_MISSIONRESULT).
*/
void _prepareTransmitMissionAck(uint8_t type)
{
    mlPending.miTransaction = MISSION_TRANSACTION_SEND_ACK;
    mlPending.miAckType = type;
}

void _prepareTransmitMissionCount(void) {
    mlPending.miTransaction = MISSION_TRANSACTION_SEND_COUNT;
}

void _prepareTransmitMissionItem(uint8_t currentMissionIndex)
{
    if (currentMissionIndex < mlPending.miTotalMissions) {
        mlPending.miTransaction = MISSION_TRANSACTION_SEND_ITEM;
        mlPending.miCurrentMission = currentMissionIndex;
    }
}

void _prepareTransmitMissionRequest(uint8_t currentMissionIndex) {
    if (currentMissionIndex < mlPending.miTotalMissions) {
        mlPending.miTransaction = MISSION_TRANSACTION_SEND_REQUEST;
        mlPending.miCurrentMission = currentMissionIndex;
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
 *
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

void copyBufferToDMA1(unsigned char size) {
    unsigned char i;
    for (i = 0; i < size; i += 1) {
        BufferB[i] = (unsigned int) readFront(uartBufferOut);
    }
}

void send2GS(unsigned char* protData) {
    unsigned int bufLen, i;

    // add the data to the circular buffer
    for (i = 1; i <= protData[0]; i += 1) {
        writeBack(uartBufferOut, protData[i]);
    }

    // get the Length of the logBuffer
    bufLen = getLength(uartBufferOut);


    // if the interrupt catched up with the circularBuffer
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

// Interrupts

void __attribute__((interrupt, no_auto_psv)) _DMA1Interrupt(void) {
    // Clear the DMA1 Interrupt Flag;
    IFS0bits.DMA1IF = 0;
}

void __attribute__((__interrupt__, no_auto_psv)) _U2RXInterrupt(void) {

    // Read the buffer while it has data
    // and add it to the circular buffer
    while (U2STAbits.URXDA == 1) {
        writeBack(uartBufferIn, (unsigned char) U2RXREG);
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

void addMessageToSpiOut(mavlink_message_t* msg) {
    // mlPending.spiCurrentIndex = 0;

    mlPending.spiTotalData += mavlink_msg_to_send_buffer(&(mlPending.spiToSensor[mlPending.spiTotalData]), msg);

}
