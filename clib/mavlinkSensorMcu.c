
#include "mavlinkSensorMcu.h"
#include "apDefinitions.h"


// Declare the global structures that will hold the state of the Sensor MCU

bool sendCommandAcknowledgement; // used for IPC of command acks to control MCU
bool sendGpsOriginMessage;


mavlink_raw_imu_t mlRawImuData;
mavlink_gps_raw_int_t mlGpsData;
mavlink_cpu_load_t mlCpuLoadData;
mavlink_scaled_pressure_t mlAirData;
mavlink_sensor_bias_t mlSensorBiasData;
mavlink_diagnostic_t mlDiagnosticData;
mavlink_raw_pressure_t mlRawPressureData;
mavlink_attitude_t mlAttitudeData;
mavlink_local_position_ned_t mlLocalPositionData;
mavlink_rc_channels_raw_t mlPilotConsoleData;
mavlink_scaled_imu_t mlFilteredData;
mavlink_boot_t mlBoot;
mavlink_gps_date_time_t mlGpsDateTime;
mavlink_coordinate_float_t mlGSLocationFloat;
#if USE_NMEA
mavlink_status_gps_t mlGpsStatus;
#endif
mavlink_novatel_diag_t mlNovatelStatus;
mavlink_sensor_diag_t mlSensorDiag;


// via SPI
mavlink_set_gps_global_origin_t mlGSLocation;
mavlink_command_ack_t mlCommandAck;
mavlink_servo_output_raw_t mlPwmCommands;

// Debug
mavlink_statustext_t mlStatustext;

mavlink_message_t mavlinkMessageBuffer;

/**********************************************************************
 * Function: mavlinkInit
 * @return none
 * @remark Initializes the sensor MCU's mavlink message structures.
 ***********************************************************************/
void mavlinkInit(void) {

    // clear all the variables
    memset(&mlGpsData, 0, sizeof (mavlink_gps_raw_int_t));
    memset(&mlCpuLoadData, 0, sizeof (mavlink_cpu_load_t));
    memset(&mlAirData, 0, sizeof (mavlink_scaled_pressure_t));
    memset(&mlSensorBiasData, 0, sizeof (mavlink_sensor_bias_t));
    memset(&mlDiagnosticData, 0, sizeof (mavlink_diagnostic_t));
    memset(&mlPilotConsoleData, 0, sizeof (mavlink_rc_channels_raw_t));
    memset(&mlRawImuData, 0, sizeof (mavlink_raw_imu_t));
    memset(&mlRawPressureData, 0, sizeof (mavlink_raw_pressure_t));
    memset(&mlAttitudeData, 0, sizeof (mavlink_attitude_t));
    memset(&mlLocalPositionData, 0, sizeof (mavlink_local_position_ned_t));
    memset(&mlFilteredData, 0, sizeof (mavlink_scaled_imu_t));
    memset(&mlBoot, 0, sizeof (mavlink_boot_t));
    memset(&mlGpsDateTime, 0, sizeof (mavlink_gps_date_time_t));
    memset(&mlGSLocationFloat, 0, sizeof (mavlink_coordinate_float_t));
#if USE_NMEA
    memset(&mlGpsStatus, 0, sizeof(mavlink_status_gps_t));
#else 
    memset(&mlNovatelStatus, 0, sizeof(mavlink_novatel_diag_t));
#endif
    memset(&mlSensorDiag, 0, sizeof(mavlink_sensor_diag_t));

    //spi	
    memset(&mlGSLocation, 0, sizeof (mavlink_set_gps_global_origin_t));
    memset(&mlCommandAck, 0, sizeof (mavlink_command_ack_t));
    memset(&mlPwmCommands, 0, sizeof (mavlink_servo_output_raw_t));

    // Debug
    memset(&mlStatustext, 0, sizeof(mavlink_statustext_t));

    // Flags
    sendCommandAcknowledgement = false;
    sendGpsOriginMessage  = false;

    // Load default home location
    mlGSLocationFloat.lat = HOME_LATITUDE;
    mlGSLocationFloat.lon = HOME_LONGITUDE;
    mlGSLocationFloat.alt = HOME_ALTITUDE;
}


// -- HIL Mavlink Messages --
uint8_t _prepareServoOutputMavlink(uint8_t compId, mavlink_channel_t chan, unsigned char *buf) {
    memset(&mavlinkMessageBuffer, 0, sizeof (mavlink_message_t));
    mavlink_msg_servo_output_raw_encode_chan(SLUGS_SYSTEMID,
        compId,
        chan,
        &buf,
        &mlPwmCommands);
    return mavlink_msg_to_send_buffer(buf, &mavlinkMessageBuffer);
}

// -- SPI Mavlink Messages --
uint8_t _prepareGpsMavlink(uint8_t compId, mavlink_channel_t chan, unsigned char *buf) {
    memset(&mavlinkMessageBuffer, 0, sizeof (mavlink_message_t));
    mavlink_msg_gps_raw_int_encode_chan(SLUGS_SYSTEMID,
        compId,
        chan,
        &mavlinkMessageBuffer,
        &mlGpsData);
    return mavlink_msg_to_send_buffer(buf, &mavlinkMessageBuffer);
}

uint8_t _prepareGpsTimeMavlink(uint8_t compId, mavlink_channel_t chan, unsigned char *buf) {
    memset(&mavlinkMessageBuffer, 0, sizeof (mavlink_message_t));
    mavlink_msg_gps_date_time_encode_chan(SLUGS_SYSTEMID,
        compId,
        chan,
        &mavlinkMessageBuffer,
        &mlGpsDateTime);
    return mavlink_msg_to_send_buffer(buf, &mavlinkMessageBuffer);
}

uint8_t _prepareCpuLoadMavlink(uint8_t compId, mavlink_channel_t chan, unsigned char *buf) {
    memset(&mavlinkMessageBuffer, 0, sizeof (mavlink_message_t));
    mavlink_msg_cpu_load_encode_chan(SLUGS_SYSTEMID,
        compId,
        chan,
        &mavlinkMessageBuffer,
        &mlCpuLoadData);
    return mavlink_msg_to_send_buffer(buf, &mavlinkMessageBuffer);
}

uint8_t _prepareAttitudeMavlink(uint8_t compId, mavlink_channel_t chan, unsigned char *buf) {
    memset(&mavlinkMessageBuffer, 0, sizeof (mavlink_message_t));
    mavlink_msg_attitude_encode_chan(SLUGS_SYSTEMID,
        compId,
        chan,
        &mavlinkMessageBuffer,
        &mlAttitudeData);
    return mavlink_msg_to_send_buffer(buf, &mavlinkMessageBuffer);
}

uint8_t _prepareLocalPositionMavlink(uint8_t compId, mavlink_channel_t chan, unsigned char *buf) {
    memset(&mavlinkMessageBuffer, 0, sizeof (mavlink_message_t));
    mavlink_msg_local_position_ned_encode_chan(SLUGS_SYSTEMID,
        compId,
        chan,
        &mavlinkMessageBuffer,
        &mlLocalPositionData);
    return mavlink_msg_to_send_buffer(buf, &mavlinkMessageBuffer);
}

uint8_t _prepareSensorDiagnosticMavlink(uint8_t compId, mavlink_channel_t chan, unsigned char *buf) {
    memset(&mavlinkMessageBuffer, 0, sizeof (mavlink_message_t));
    mavlink_msg_sensor_diag_encode_chan(SLUGS_SYSTEMID,
        compId,
        chan,
        &mavlinkMessageBuffer,
        &mlSensorDiag);
    return mavlink_msg_to_send_buffer(buf, &mavlinkMessageBuffer);
}

uint8_t _prepareGpsStatusMavlink(uint8_t compId, mavlink_channel_t chan, unsigned char *buf) {
    memset(&mavlinkMessageBuffer, 0, sizeof (mavlink_message_t));
#if USE_NMEA
    mavlink_msg_status_gps_encode_chan(SLUGS_SYSTEMID,
        compId,
        chan,
        &mavlinkMessageBuffer,
        &mlGpsStatus);
#else
    mavlink_msg_novatel_diag_encode_chan(SLUGS_SYSTEMID,
        compId,
        chan,
        &mavlinkMessageBuffer,
        &mlNovatelStatus);
#endif
    return mavlink_msg_to_send_buffer(buf, &mavlinkMessageBuffer);
}


uint8_t _prepareSensorBiasMavlink(uint8_t compId, mavlink_channel_t chan, unsigned char *buf) {
    memset(&mavlinkMessageBuffer, 0, sizeof (mavlink_message_t));
    mavlink_msg_sensor_bias_encode_chan(SLUGS_SYSTEMID,
        compId,
        chan,
        &mavlinkMessageBuffer,
        &mlSensorBiasData);
    return mavlink_msg_to_send_buffer(buf, &mavlinkMessageBuffer);
}

uint8_t _prepareRawPressureMavlink(uint8_t compId, mavlink_channel_t chan, unsigned char *buf) {
    memset(&mavlinkMessageBuffer, 0, sizeof (mavlink_message_t));
    mavlink_msg_raw_pressure_encode_chan(SLUGS_SYSTEMID,
        compId,
        chan,
        &mavlinkMessageBuffer,
        &mlRawPressureData);
    return mavlink_msg_to_send_buffer(buf, &mavlinkMessageBuffer);
}

uint8_t _prepareScaledPressureMavlink(uint8_t compId, mavlink_channel_t chan, unsigned char *buf) {
    memset(&mavlinkMessageBuffer, 0, sizeof (mavlink_message_t));
    mavlink_msg_scaled_pressure_encode_chan(SLUGS_SYSTEMID,
        compId,
        chan,
        &mavlinkMessageBuffer,
        &mlAirData);
    return mavlink_msg_to_send_buffer(buf, &mavlinkMessageBuffer);
}

uint8_t _prepareRcChannelsMavlink(uint8_t compId, mavlink_channel_t chan, unsigned char *buf) {
    memset(&mavlinkMessageBuffer, 0, sizeof (mavlink_message_t));
    mavlink_msg_rc_channels_raw_encode_chan(SLUGS_SYSTEMID,
        compId,
        chan,
        &mavlinkMessageBuffer,
        &mlPilotConsoleData);
    return mavlink_msg_to_send_buffer(buf, &mavlinkMessageBuffer);
}

uint8_t _prepareRawImuMavlink(uint8_t compId, mavlink_channel_t chan, unsigned char *buf) {
    memset(&mavlinkMessageBuffer, 0, sizeof (mavlink_message_t));
    mavlink_msg_raw_imu_encode_chan(SLUGS_SYSTEMID,
        compId,
        chan,
        &mavlinkMessageBuffer,
        &mlRawImuData);
    return mavlink_msg_to_send_buffer(buf, &mavlinkMessageBuffer);
}

uint8_t _prepareScaledImuMavlink(uint8_t compId, mavlink_channel_t chan, unsigned char *buf) {
    memset(&mavlinkMessageBuffer, 0, sizeof (mavlink_message_t));
    mavlink_msg_scaled_imu_encode_chan(SLUGS_SYSTEMID,
        compId,
        chan,
        &mavlinkMessageBuffer,
        &mlFilteredData);
    return mavlink_msg_to_send_buffer(buf, &mavlinkMessageBuffer);
}

uint8_t _prepareCommandAckMavlink(uint8_t compId, mavlink_channel_t chan, unsigned char *buf) {
    memset(&mavlinkMessageBuffer, 0, sizeof (mavlink_message_t));
    mavlink_msg_command_ack_encode_chan(SLUGS_SYSTEMID,
        compId,
        chan,
        &mavlinkMessageBuffer,
        &mlCommandAck
    );
    return mavlink_msg_to_send_buffer(buf, &mavlinkMessageBuffer);
}


uint8_t _prepareStatusTextMavlink(uint8_t compId, mavlink_channel_t chan, unsigned char *buf) {
    memset(&mavlinkMessageBuffer, 0, sizeof (mavlink_message_t));
    mavlink_msg_statustext_pack_chan(SLUGS_SYSTEMID,
        compId,
        chan,
        &mavlinkMessageBuffer,
        mlStatustext.severity,
        mlStatustext.text
    );
    return mavlink_msg_to_send_buffer(buf, &mavlinkMessageBuffer);
}

uint8_t _prepareBootMavlink(uint8_t compId, mavlink_channel_t chan, unsigned char *buf) {
    memset(&mavlinkMessageBuffer, 0, sizeof (mavlink_message_t));
    mavlink_msg_boot_pack_chan(SLUGS_SYSTEMID,
        compId,
        chan,
        &mavlinkMessageBuffer,
        mlBoot.version);
    return mavlink_msg_to_send_buffer(buf, &mavlinkMessageBuffer);
}