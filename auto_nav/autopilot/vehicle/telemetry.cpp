/**
 * @file telemetry.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Vehicle telemetry 
 * 
 * @version 0.1
 * @date 2025-02-28
 * 
 * @copyright Copyright (c) 2025
 * 
 */

//=======================================================================================
// Includes 

#include "telemetry.h" 

//=======================================================================================


//=======================================================================================
// MAVLink message decoding 

// MAVLink message decode 
void VehicleTelemetry::MAVLinkMessageDecode(Vehicle &vehicle)
{
    // Check for a connection (heartbeat) timeout 
    if (heartbeat_status_timer++ >= VS_HEARTBEAT_TIMEOUT)
    {
        // Have not seen a heartbeat message from a GCS for too long. The system is 
        // considered to be disconnected. 
        heartbeat_status_timer--; 
        connected = FLAG_CLEAR; 
    }

    // Check if new data is available. If so then get the data and process it. 
    if (vehicle.hardware.data_ready.telemetry_ready == FLAG_SET)
    {
        vehicle.hardware.data_ready.telemetry_ready = FLAG_CLEAR; 
        data_in_index = RESET; 

        // Get a copy of the data so we don't have to hold the comms mutex throughout the 
        // whole decoding process. 
        xSemaphoreTake(vehicle.comms_mutex, portMAX_DELAY); 
        vehicle.hardware.TelemetryGet(data_in_size, data_in_buff); 
        xSemaphoreGive(vehicle.comms_mutex); 
        
        // Look at each byte of the received data and try to decode MAVLink messages 
        // until there is no more data to check. 
        while (data_in_index < data_in_size)
        {
            if (mavlink_parse_char(channel, 
                                   data_in_buff[data_in_index++], 
                                   &msg, 
                                   &status))
            {
                MAVLinkPayloadDecode(vehicle); 
            }
        }
    }

    // Send any needed messages in response to messages received. 
    MAVLinkMessageSend(vehicle); 
}


// MAVLink message payload decode 
void VehicleTelemetry::MAVLinkPayloadDecode(Vehicle &vehicle)
{
    switch (msg.msgid)
    {
        case MAVLINK_MSG_ID_HEARTBEAT: 
            MAVLinkHeartbeatReceive(); 
            break; 

        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: 
            MAVLinkParamRequestListReceive(); 
            break; 

        case MAVLINK_MSG_ID_MISSION_REQUEST: 
            MAVLinkMissionRequestReceive(); 
            break; 

        case MAVLINK_MSG_ID_REQUEST_DATA_STREAM: 
            MAVLinkRequestDataStreamReceive(); 
            break; 
        
        case MAVLINK_MSG_ID_COMMAND_LONG: 
            MAVLinkCommandLongReceive(vehicle); 
            break; 
        
        default: 
            break; 
    }
}

//=======================================================================================


//=======================================================================================
// MAVLink message encoding 

// MAVLink periodic message encode 
void VehicleTelemetry::MAVLinkMessageEncode(Vehicle &vehicle)
{
    MAVLinkHeartbeatSend(); 
    MAVLinkRawIMUSendPeriodic(); 
    MAVLinkGPSRawIntSendPeriodic(); 
    MAVLinkRCChannelScaledSendPeriodic(); 
    MAVLinkRCChannelRawSendPeriodic(); 
    MAVLinkServoOutputRawSendPeriodic(); 
    MAVLinkAttitudeSendPeriodic(); 
    MAVLinkPositionTargetGlobalIntSendPeriodic(); 
    MAVLinkNavControllerSendPeriodic(); 
    MAVLinkLocalPositionNEDSendPeriodic(); 
    MAVLinkGlobalPositionIntSendPeriodic(); 

    MAVLinkMessageSend(vehicle); 
}


// MAVLink message format 
void VehicleTelemetry::MAVLinkMessageFormat(void)
{
    data_out_size += mavlink_msg_to_send_buffer(data_out_buff + data_out_size, &msg); 
}


// MAVLink message send 
void VehicleTelemetry::MAVLinkMessageSend(Vehicle &vehicle)
{
    // If the size of the data output is not zero it means messages have been encoded 
    // to be sent. 
    if (data_out_size)
    {
        // The specific vehicles comms thread must release the telemetry output mutex 
        // after sending the telemetry data. This mutex exists because there are both 
        // periodic message sends and sends in response to incoming messages which have 
        // the potential the overwrite the sending buffer if not protected. 
        xSemaphoreTake(vehicle.telemetry_out_mutex, portMAX_DELAY); 
        xSemaphoreTake(vehicle.comms_mutex, portMAX_DELAY); 
        vehicle.hardware.TelemetrySet(data_out_size, data_out_buff); 
        xSemaphoreGive(vehicle.comms_mutex); 
        vehicle.CommsEventQueueTelemetryWrite(); 
        data_out_size = RESET; 
    }
}

//=======================================================================================


//=======================================================================================
// MAVLink: Heartbeat protocol 

// MAVLink: HEARTBEAT 
void VehicleTelemetry::MAVLinkHeartbeatReceive(void)
{
    mavlink_msg_heartbeat_decode(
        &msg, 
        &mavlink.heartbeat_msg_gcs); 

    // This system is only concerned with heartbeats from the GCS it's communicating with. 
    // The system considers itself connected only if the heatbeat message type and source 
    // are correct. 
    if ((mavlink.heartbeat_msg_gcs.type == MAV_TYPE_GCS) && 
        (mavlink.heartbeat_msg_gcs.autopilot == MAV_AUTOPILOT_INVALID) && 
        (msg.sysid == VS_GCS_ID) && 
        (msg.compid == MAV_COMP_ID_MISSIONPLANNER))
    {
        heartbeat_status_timer = RESET; 
        connected = FLAG_SET; 
    }
}


// Heartbeat message send 
void VehicleTelemetry::MAVLinkHeartbeatSend(void)
{
    if (mavlink.heartbeat_msg_timing.enable && 
       (++mavlink.heartbeat_msg_timing.count >= mavlink.heartbeat_msg_timing.count_lim))
    {
        mavlink.heartbeat_msg_timing.count = RESET; 
        mavlink_msg_heartbeat_encode_chan(
            system_id, 
            component_id, 
            channel, 
            &msg, 
            &mavlink.heartbeat_msg); 
        MAVLinkMessageFormat(); 
    }
}


// Set flight mode based on vehicle state 
void VehicleTelemetry::MAVLinkHeartbeatSetMode(uint8_t mode)
{
    mavlink.heartbeat_msg.custom_mode = (uint32_t)mode; 
}

//=======================================================================================


//=======================================================================================
// MAVLink: Parameter protocol 

// MAVLink: PARAM_REQUEST_LIST 
void VehicleTelemetry::MAVLinkParamRequestListReceive(void)
{
    // 
}

//=======================================================================================


//=======================================================================================
// MAVLink: Mission protocol 

// MAVLink: MISSION_REQUEST 
void VehicleTelemetry::MAVLinkMissionRequestReceive(void)
{
    // 
}

//=======================================================================================


//=======================================================================================
// MAVLink: Command protocol 

// MAVLink: COMMAND_LONG 
void VehicleTelemetry::MAVLinkCommandLongReceive(Vehicle &vehicle)
{
    mavlink_msg_command_long_decode(
        &msg, 
        &mavlink.command_long_msg_gcs); 

    // This system is only concerned with messages meant for this system. If the taget 
    // system and component ID in the message does not match this system then abort. 
    if ((mavlink.command_long_msg_gcs.target_system != system_id) || 
        (mavlink.command_long_msg_gcs.target_component != component_id))
    {
        return; 
    }

    MAVLinkCommandACKSend(); 
    MAVLinkCommandLongDecode(vehicle); 
}


// MAVLink command decode 
void VehicleTelemetry::MAVLinkCommandLongDecode(Vehicle &vehicle)
{
    switch (mavlink.command_long_msg_gcs.command)
    {
        case MAV_CMD_DO_SET_MODE: 
            MAVLinkCommandDoSetModeReceive(vehicle); 
            break; 
        
        case MAV_CMD_REQUEST_MESSAGE: 
            MAVLinkCommandRequestMessageReceive(); 
            break; 

        default: 
            break; 
    }
}


// MAVLink command: DO_SET_MODE 
void VehicleTelemetry::MAVLinkCommandDoSetModeReceive(Vehicle &vehicle)
{
    // Ardupilot sets param1 to MAV_MODE_FLAG_CUSTOM_MODE_ENABLED so we look for that 
    // before attempting to update the vehicle state/mode. 
    if ((uint16_t)mavlink.command_long_msg_gcs.param1 == MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)
    {
        vehicle.MainStateSelect((uint8_t)mavlink.command_long_msg_gcs.param2); 
    }
}


// MAVLink command: REQUEST_MESSAGE 
void VehicleTelemetry::MAVLinkCommandRequestMessageReceive(void)
{
    uint16_t msg_id = (uint16_t)mavlink.command_long_msg_gcs.param1; 

    switch (msg_id)
    {
        case MAVLINK_MSG_ID_AUTOPILOT_VERSION: 
            MAVLinkAutopilotVersionSend(); 
            break; 

        default: 
            break; 
    }
}


// MAVLink command: COMMAND_ACK 
void VehicleTelemetry::MAVLinkCommandACKSend(void)
{
    // mavlink_msg_command_ack_pack_chan(
    //     mavlink.system_id, 
    //     mavlink.component_id, 
    //     mavlink.channel, 
    //     &mavlink.msg, 
    //     mavlink.command_long_msg_gcs.command, 
    //     MAV_RESULT_ACCEPTED, 
    //     ZERO, 
    //     ZERO, 
    //     SIK_TEST_GCS_ID, 
    //     MAV_COMP_ID_MISSIONPLANNER); 
    // Send message to buffer to be sent to telemetry 
}

//=======================================================================================


//=======================================================================================
// Other messages 

// MAVLink: REQUEST_DATA_STREAM 
void VehicleTelemetry::MAVLinkRequestDataStreamReceive(void)
{
    // 
}


// MAVLink: AUTOPILOT_VERSION 
void VehicleTelemetry::MAVLinkAutopilotVersionSend(void)
{
    // 
}

//=======================================================================================


//=======================================================================================
// MAVLink: Requestable outgoing messages 

// MAVLink: RAW_IMU 
void VehicleTelemetry::MAVLinkRawIMUSendPeriodic(void)
{
    if (mavlink.raw_imu_msg_timing.enable && 
       (++mavlink.raw_imu_msg_timing.count >= mavlink.raw_imu_msg_timing.count_lim))
    {
        mavlink.raw_imu_msg_timing.count = RESET; 

        mavlink_raw_imu_t raw_imu_msg; 
        mavlink_msg_raw_imu_encode_chan(
            system_id, 
            component_id, 
            channel, 
            &msg, 
            &raw_imu_msg); 
        // mavlink_msg_raw_imu_pack_chan(
        //     system_id, 
        //     component_id, 
        //     channel, 
        //     &msg, 
        //     SIK_TEST_MOCK_BOOT_TIME,   // Time since boot 
        //     ZERO,                      // X accelerometer 
        //     ZERO,                      // Y accelerometer 
        //     SIK_TEST_MOCK_IMU_DIR,     // Z accelerometer 
        //     ZERO,                      // X gyroscope 
        //     ZERO,                      // Y gyroscope 
        //     ZERO,                      // Z gyroscope 
        //     SIK_TEST_MOCK_IMU_DIR,     // X magnetometer 
        //     ZERO,                      // Y magnetometer 
        //     ZERO,                      // Z magnetometer 
        //     ZERO,                      // IMU ID 
        //     ZERO);                     // Temperature 
        MAVLinkMessageFormat(); 
    }
}


// MAVLink: GPS_RAW_INT 
void VehicleTelemetry::MAVLinkGPSRawIntSendPeriodic(void)
{
    if (mavlink.gps_raw_int_msg_timing.enable && 
       (++mavlink.gps_raw_int_msg_timing.count >= mavlink.gps_raw_int_msg_timing.count_lim))
    {
        mavlink.gps_raw_int_msg_timing.count = RESET; 

        mavlink_gps_raw_int_t gps_raw_int_msg; 
        mavlink_msg_gps_raw_int_encode_chan(
            system_id, 
            component_id, 
            channel, 
            &msg, 
            &gps_raw_int_msg); 
        // mavlink_msg_gps_raw_int_pack_chan(
        //     system_id, 
        //     component_id, 
        //     channel, 
        //     &msg, 
        //     SIK_TEST_MOCK_BOOT_TIME,        // Timestamp 
        //     GPS_FIX_TYPE_3D_FIX,            // GPS fix type 
        //     SIK_TEST_MOCK_LAT,              // Latitude 
        //     SIK_TEST_MOCK_LON,              // Longitude 
        //     SIK_TEST_MOCK_ALTITUDE,         // Altitude 
        //     HIGH_16BIT,                     // GPS HDOP horizontal dilution of position 
        //     HIGH_16BIT,                     // GPS VDOP vertical dilution of position 
        //     HIGH_16BIT,                     // GPS ground speed 
        //     HIGH_16BIT,                     // Course over ground 
        //     SIK_TEST_MOCK_NUM_SATELLITES,   // Number of satellites visible 
        //     SIK_TEST_MOCK_ALTITUDE,         // Altitude 
        //     ZERO,                           // Position uncertainty 
        //     ZERO,                           // Altitude uncertainty 
        //     ZERO,                           // Speed uncertainty 
        //     ZERO,                           // Heading / track uncertainty 
        //     ZERO);                          // Yaw in earth frame from north 
        MAVLinkMessageFormat(); 
    }
}


// MAVLink: RC_CHANNELS_SCALED 
void VehicleTelemetry::MAVLinkRCChannelScaledSendPeriodic(void)
{
    if (mavlink.rc_channels_scaled_msg_timing.enable && 
       (++mavlink.rc_channels_scaled_msg_timing.count >= 
          mavlink.rc_channels_scaled_msg_timing.count_lim))
    {
        mavlink.rc_channels_scaled_msg_timing.count = RESET; 

        mavlink_rc_channels_scaled_t rc_channels_scaled_msg; 
        mavlink_msg_rc_channels_scaled_encode_chan(
            system_id, 
            component_id, 
            channel, 
            &msg, 
            &rc_channels_scaled_msg); 
        // mavlink_msg_rc_channels_scaled_pack_chan(
        //     system_id, 
        //     component_id, 
        //     channel, 
        //     &msg, 
        //     SIK_TEST_MOCK_BOOT_TIME,                          // Time since boot 
        //     ZERO,                                             // Servo output port 
        //     ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO,   // RC channels 1-8 
        //     HIGH_8BIT);                                       // Receive signal strength 
        MAVLinkMessageFormat(); 
    }
}


// MAVLink: RC_CHANNELS_RAW 
void VehicleTelemetry::MAVLinkRCChannelRawSendPeriodic(void)
{
    if (mavlink.rc_channels_raw_msg_timing.enable && 
       (++mavlink.rc_channels_raw_msg_timing.count >= 
          mavlink.rc_channels_raw_msg_timing.count_lim))
    {
        mavlink.rc_channels_raw_msg_timing.count = RESET; 

        mavlink_rc_channels_raw_t rc_channels_raw_msg; 
        mavlink_msg_rc_channels_raw_encode_chan(
            system_id, 
            component_id, 
            channel, 
            &msg, 
            &rc_channels_raw_msg); 
        // mavlink_msg_rc_channels_raw_pack_chan(
        //     system_id, 
        //     component_id, 
        //     channel, 
        //     &msg, 
        //     SIK_TEST_MOCK_BOOT_TIME,                          // Time since boot 
        //     ZERO,                                             // Servo output port 
        //     ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO,   // RC channels 1-8 
        //     HIGH_8BIT);                                       // Receive signal strength 
        MAVLinkMessageFormat(); 
    }
}


// MAVLink: SERVO_OUTPUT_RAW 
void VehicleTelemetry::MAVLinkServoOutputRawSendPeriodic(void)
{
    if (mavlink.servo_output_raw_msg_timing.enable && 
       (++mavlink.servo_output_raw_msg_timing.count >= 
          mavlink.servo_output_raw_msg_timing.count_lim))
    {
        mavlink.servo_output_raw_msg_timing.count = RESET; 

        mavlink_servo_output_raw_t servo_output_raw_msg; 
        mavlink_msg_servo_output_raw_encode_chan(
            system_id, 
            component_id, 
            channel, 
            &msg, 
            &servo_output_raw_msg); 
        // mavlink_msg_servo_output_raw_pack_chan(
        //     system_id, 
        //     component_id, 
        //     channel, 
        //     &msg, 
        //     SIK_TEST_MOCK_BOOT_TIME,                           // Time since boot 
        //     ZERO,                                              // Servo output port 
        //     ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO,    // Servo output 1-8 
        //     ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, ZERO);   // Servo output 9-16 
        MAVLinkMessageFormat(); 
    }
}


// MAVLink: ATTITUDE 
void VehicleTelemetry::MAVLinkAttitudeSendPeriodic(void)
{
    if (mavlink.attitude_msg_timing.enable && 
       (++mavlink.attitude_msg_timing.count >= mavlink.attitude_msg_timing.count_lim))
    {
        mavlink.attitude_msg_timing.count = RESET; 

        mavlink_attitude_t attitude_msg; 
        mavlink_msg_attitude_encode_chan(
            system_id, 
            component_id, 
            channel, 
            &msg, 
            &attitude_msg); 
        // mavlink_msg_attitude_pack_chan(
        //     system_id, 
        //     component_id, 
        //     channel, 
        //     &msg, 
        //     SIK_TEST_MOCK_BOOT_TIME,   // Time since boot 
        //     ZERO,                      // Roll angle (rad) 
        //     ZERO,                      // Pitch angle (rad) 
        //     ZERO,                      // Yaw angle (rad) 
        //     ZERO,                      // Roll angular speed (rad/s) 
        //     ZERO,                      // Pitch angular speed (rad/s) 
        //     ZERO);                     // Yaw angular speed (rad/s) 
        MAVLinkMessageFormat(); 
    }
}


// MAVLink: POSITION_TARGET_GLOBAL_INT 
void VehicleTelemetry::MAVLinkPositionTargetGlobalIntSendPeriodic(void)
{
    if (mavlink.position_target_global_int_msg_timing.enable && 
       (++mavlink.position_target_global_int_msg_timing.count >= 
          mavlink.position_target_global_int_msg_timing.count_lim))
    {
        mavlink.position_target_global_int_msg_timing.count = RESET; 

        mavlink_position_target_global_int_t position_target_global_int_msg; 
        mavlink_msg_position_target_global_int_encode_chan(
            system_id, 
            component_id, 
            channel, 
            &msg, 
            &position_target_global_int_msg); 
        // uint16_t type_mask = POSITION_TARGET_TYPEMASK_VX_IGNORE | 
        //                     POSITION_TARGET_TYPEMASK_VY_IGNORE | 
        //                     POSITION_TARGET_TYPEMASK_VZ_IGNORE | 
        //                     POSITION_TARGET_TYPEMASK_AX_IGNORE | 
        //                     POSITION_TARGET_TYPEMASK_AY_IGNORE | 
        //                     POSITION_TARGET_TYPEMASK_AZ_IGNORE | 
        //                     POSITION_TARGET_TYPEMASK_YAW_IGNORE | 
        //                     POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE | 
        //                     0xF000; 
        // mavlink_msg_position_target_global_int_pack_chan(
        //     system_id, 
        //     component_id, 
        //     channel, 
        //     &msg, 
        //     SIK_TEST_MOCK_BOOT_TIME,   // Time since boot 
        //     MAV_FRAME_GLOBAL,          // Coordinate frame (MAV_FRAME) 
        //     type_mask,                 // Ignored dimensions (POSITION_TARGET_TYPEMASK) 
        //     SIK_TEST_MOCK_LAT,         // Latitude 
        //     SIK_TEST_MOCK_LON,         // Longitude 
        //     SIK_TEST_MOCK_ALTITUDE,    // Altitude 
        //     ZERO,                      // X velocity in NED frame (m/s) 
        //     ZERO,                      // Y velocity in NED frame (m/s) 
        //     ZERO,                      // Z velocity in NED frame (m/s) 
        //     ZERO,                      // X acceleration or force in NED frame (N) 
        //     ZERO,                      // y acceleration or force in NED frame (N) 
        //     ZERO,                      // Z acceleration or force in NED frame (N) 
        //     ZERO,                      // Yaw setpoint (rad) 
        //     ZERO);                     // Yaw rate setpoint (rad/s) 
        MAVLinkMessageFormat(); 
    }
}


// MAVLink: NAV_CONTROLLER 
void VehicleTelemetry::MAVLinkNavControllerSendPeriodic(void)
{
    if (mavlink.nav_controller_output_msg_timing.enable && 
       (++mavlink.nav_controller_output_msg_timing.count >= 
          mavlink.nav_controller_output_msg_timing.count_lim))
    {
        mavlink.nav_controller_output_msg_timing.count = RESET; 

        mavlink_nav_controller_output_t nav_controller_output_msg; 
        mavlink_msg_nav_controller_output_encode_chan(
            system_id, 
            component_id, 
            channel, 
            &msg, 
            &nav_controller_output_msg); 
        // mavlink_msg_nav_controller_output_pack_chan(
        //     system_id, 
        //     component_id, 
        //     channel, 
        //     &msg, 
        //     ZERO,                          // Current desired roll 
        //     ZERO,                          // Current desired pitch 
        //     ZERO,                          // Current desired heading 
        //     ZERO,                          // Bearing to current waypoint/target 
        //     SIK_TEST_MOCK_WP_DISTANCE,     // Distance to active waypoint 
        //     ZERO,                          // Current altitude error 
        //     ZERO,                          // Current airspeed error 
        //     ZERO);                         // Current crosstrack error on x-y plane 
        MAVLinkMessageFormat(); 
    }
}


// MAVLink: LOCAL_POSITION_NED 
void VehicleTelemetry::MAVLinkLocalPositionNEDSendPeriodic(void)
{
    if (mavlink.local_position_ned_msg_timing.enable && 
       (++mavlink.local_position_ned_msg_timing.count >= 
          mavlink.local_position_ned_msg_timing.count_lim))
    {
        mavlink.local_position_ned_msg_timing.count = RESET; 

        mavlink_local_position_ned_t local_position_ned_msg; 
        mavlink_msg_local_position_ned_encode_chan(
            system_id, 
            component_id, 
            channel, 
            &msg, 
            &local_position_ned_msg); 
        // mavlink_msg_local_position_ned_pack_chan(
        //     system_id, 
        //     component_id, 
        //     channel, 
        //     &msg, 
        //     SIK_TEST_MOCK_BOOT_TIME,   // Time since boot 
        //     ZERO,                      // X position (m) 
        //     ZERO,                      // Y position (m) 
        //     ZERO,                      // Z position (m) 
        //     ZERO,                      // X speed (m/s) 
        //     ZERO,                      // Y speed (m/s) 
        //     ZERO);                     // Z speed (m/s) 
        MAVLinkMessageFormat(); 
    }
}


// MAVLink: GLOBAL_POSITION_INT 
void VehicleTelemetry::MAVLinkGlobalPositionIntSendPeriodic(void)
{
    if (mavlink.global_pos_int_msg_timing.enable && 
       (++mavlink.global_pos_int_msg_timing.count >= 
          mavlink.global_pos_int_msg_timing.count_lim))
    {
        mavlink.global_pos_int_msg_timing.count = RESET; 

        mavlink_global_position_int_t global_position_int_msg; 
        mavlink_msg_global_position_int_encode_chan(
            system_id, 
            component_id, 
            channel, 
            &msg, 
            &global_position_int_msg); 
        // mavlink_msg_global_position_int_pack_chan(
        //     system_id, 
        //     component_id, 
        //     channel, 
        //     &msg, 
        //     SIK_TEST_MOCK_BOOT_TIME,   // Time since boot 
        //     SIK_TEST_MOCK_LAT,         // Latitude 
        //     SIK_TEST_MOCK_LON,         // Longitude 
        //     SIK_TEST_MOCK_ALTITUDE,    // Altitude 
        //     SIK_TEST_MOCK_ALTITUDE,    // Relative altitude (above home) 
        //     ZERO,                      // X velocity 
        //     ZERO,                      // Y velocity 
        //     ZERO,                      // Z velocity 
        //     ZERO);                     // Heading (yaw angle) 
        MAVLinkMessageFormat(); 
    }
}

//=======================================================================================
