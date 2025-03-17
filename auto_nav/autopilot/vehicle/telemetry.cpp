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
            MAVLinkParamRequestListReceive(vehicle); 
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
    MAVLinkRawIMUSendPeriodic(vehicle); 
    MAVLinkGPSRawIntSendPeriodic(vehicle); 
    MAVLinkRCChannelScaledSendPeriodic(vehicle); 
    MAVLinkRCChannelRawSendPeriodic(vehicle); 
    MAVLinkServoOutputRawSendPeriodic(vehicle); 
    MAVLinkAttitudeSendPeriodic(vehicle); 
    MAVLinkPositionTargetGlobalIntSendPeriodic(vehicle); 
    MAVLinkNavControllerSendPeriodic(vehicle); 
    MAVLinkLocalPositionNEDSendPeriodic(vehicle); 
    MAVLinkGlobalPositionIntSendPeriodic(vehicle); 
    MAVLinkParamValueSendPeriodic(vehicle); 

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
void VehicleTelemetry::MAVLinkParamRequestListReceive(Vehicle &vehicle)
{
    mavlink_msg_param_request_list_decode(
        &msg, 
        &mavlink.param_request_list_msg_gcs); 

    // This system is only concerned with messages meant for this system. If the taget 
    // system and component ID in the message does not match this system then abort. 
    if ((mavlink.param_request_list_msg_gcs.target_system != system_id) || 
        (mavlink.param_request_list_msg_gcs.target_component != component_id))
    {
        return; 
    }

    // Enable the PARAM_VALUE message which gets sent periodically until all parameters 
    // in the system have been sent. 
    mavlink.param_value_msg_timing.enable = FLAG_SET; 
    vehicle.memory.param_index = RESET; 
}


// MAVLink: PARAM_VALUE 
void VehicleTelemetry::MAVLinkParamValueSendPeriodic(Vehicle &vehicle)
{
    if (mavlink.param_value_msg_timing.enable && 
       (++mavlink.param_value_msg_timing.count >= mavlink.param_value_msg_timing.count_lim))
    {
        mavlink.param_value_msg_timing.count = RESET; 

        if (vehicle.memory.param_index < vehicle.memory.num_params)
        {
            mavlink_msg_param_value_pack_chan(
                system_id, 
                component_id, 
                channel, 
                &msg, 
                parameters[vehicle.memory.param_index].name, 
                parameters[vehicle.memory.param_index].value, 
                vehicle.memory.param_value_type, 
                vehicle.memory.num_params, 
                vehicle.memory.param_index); 
            MAVLinkMessageFormat(); 

            vehicle.memory.param_index++; 
        }
        else 
        {
            mavlink.param_value_msg_timing.enable = FLAG_CLEAR; 
        }
    }
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
void VehicleTelemetry::MAVLinkRawIMUSendPeriodic(Vehicle &vehicle)
{
    if (mavlink.raw_imu_msg_timing.enable && 
       (++mavlink.raw_imu_msg_timing.count >= mavlink.raw_imu_msg_timing.count_lim))
    {
        mavlink.raw_imu_msg_timing.count = RESET; 

        mavlink_msg_raw_imu_pack_chan(
            system_id, 
            component_id, 
            channel, 
            &msg, 
            vehicle.auxiliary.time_usec,      // Time since boot 
            vehicle.navigation.accel.x,       // X accelerometer 
            vehicle.navigation.accel.y,       // Y accelerometer 
            vehicle.navigation.accel.z,       // Z accelerometer 
            vehicle.navigation.gyro.x,        // X gyroscope 
            vehicle.navigation.gyro.y,        // Y gyroscope 
            vehicle.navigation.gyro.z,        // Z gyroscope 
            vehicle.navigation.mag.x,         // X magnetometer 
            vehicle.navigation.mag.y,         // Y magnetometer 
            vehicle.navigation.mag.z,         // Z magnetometer 
            0,                                // IMU ID 
            vehicle.auxiliary.temperature);   // Temperature 
        MAVLinkMessageFormat(); 
    }
}


// MAVLink: GPS_RAW_INT 
void VehicleTelemetry::MAVLinkGPSRawIntSendPeriodic(Vehicle &vehicle)
{
    if (mavlink.gps_raw_int_msg_timing.enable && 
       (++mavlink.gps_raw_int_msg_timing.count >= mavlink.gps_raw_int_msg_timing.count_lim))
    {
        mavlink.gps_raw_int_msg_timing.count = RESET; 

        mavlink_msg_gps_raw_int_pack_chan(
            system_id, 
            component_id, 
            channel, 
            &msg, 
            vehicle.auxiliary.time_usec,        // Timestamp 
            vehicle.navigation.fix_type,        // GPS fix type 
            vehicle.navigation.current.lat,     // Latitude 
            vehicle.navigation.current.lon,     // Longitude 
            vehicle.navigation.current.alt,     // Altitude 
            0xFFFF,                             // GPS HDOP horizontal dilution of position 
            0xFFFF,                             // GPS VDOP vertical dilution of position 
            vehicle.navigation.ground_speed,    // GPS ground speed 
            0xFFFF,                             // Course over ground 
            vehicle.navigation.num_satellite,   // Number of satellites visible 
            0,                                  // Altitude 
            0,                                  // Position uncertainty 
            0,                                  // Altitude uncertainty 
            0,                                  // Speed uncertainty 
            0,                                  // Heading / track uncertainty 
            0);                                 // Yaw in earth frame from north 
        MAVLinkMessageFormat(); 
    }
}


// MAVLink: RC_CHANNELS_SCALED 
void VehicleTelemetry::MAVLinkRCChannelScaledSendPeriodic(Vehicle &vehicle)
{
    if (mavlink.rc_channels_scaled_msg_timing.enable && 
       (++mavlink.rc_channels_scaled_msg_timing.count >= 
          mavlink.rc_channels_scaled_msg_timing.count_lim))
    {
        mavlink.rc_channels_scaled_msg_timing.count = RESET; 

        mavlink_msg_rc_channels_scaled_pack_chan(
            system_id, 
            component_id, 
            channel, 
            &msg, 
            vehicle.auxiliary.time_usec,   // Time since boot 
            0,                             // Servo output port 
            0, 0, 0, 0, 0, 0, 0, 0,        // RC channels 1-8 
            0xFF);                         // Receive signal strength 
        MAVLinkMessageFormat(); 
    }
}


// MAVLink: RC_CHANNELS_RAW 
void VehicleTelemetry::MAVLinkRCChannelRawSendPeriodic(Vehicle &vehicle)
{
    if (mavlink.rc_channels_raw_msg_timing.enable && 
       (++mavlink.rc_channels_raw_msg_timing.count >= 
          mavlink.rc_channels_raw_msg_timing.count_lim))
    {
        mavlink.rc_channels_raw_msg_timing.count = RESET; 

        mavlink_msg_rc_channels_raw_pack_chan(
            system_id, 
            component_id, 
            channel, 
            &msg, 
            vehicle.auxiliary.time_usec,   // Time since boot 
            0,                             // Servo output port 
            0, 0, 0, 0, 0, 0, 0, 0,        // RC channels 1-8 
            0xFF);                         // Receive signal strength 
        MAVLinkMessageFormat(); 
    }
}


// MAVLink: SERVO_OUTPUT_RAW 
void VehicleTelemetry::MAVLinkServoOutputRawSendPeriodic(Vehicle &vehicle)
{
    if (mavlink.servo_output_raw_msg_timing.enable && 
       (++mavlink.servo_output_raw_msg_timing.count >= 
          mavlink.servo_output_raw_msg_timing.count_lim))
    {
        mavlink.servo_output_raw_msg_timing.count = RESET; 

        mavlink_msg_servo_output_raw_pack_chan(
            system_id, 
            component_id, 
            channel, 
            &msg, 
            vehicle.auxiliary.time_usec,   // Time since boot 
            0,                             // Servo output port 
            0, 0, 0, 0, 0, 0, 0, 0,        // Servo output 1-8 
            0, 0, 0, 0, 0, 0, 0, 0);       // Servo output 9-16 
        MAVLinkMessageFormat(); 
    }
}


// MAVLink: ATTITUDE 
void VehicleTelemetry::MAVLinkAttitudeSendPeriodic(Vehicle &vehicle)
{
    if (mavlink.attitude_msg_timing.enable && 
       (++mavlink.attitude_msg_timing.count >= mavlink.attitude_msg_timing.count_lim))
    {
        mavlink.attitude_msg_timing.count = RESET; 

        mavlink_msg_attitude_pack_chan(
            system_id, 
            component_id, 
            channel, 
            &msg, 
            vehicle.auxiliary.time_usec,         // Time since boot 
            vehicle.navigation.roll,             // Roll angle (rad) 
            vehicle.navigation.pitch,            // Pitch angle (rad) 
            vehicle.navigation.yaw,              // Yaw angle (rad) 
            (float)vehicle.navigation.gyro.x,    // Roll angular speed (rad/s) 
            (float)vehicle.navigation.gyro.y,    // Pitch angular speed (rad/s) 
            (float)vehicle.navigation.gyro.z);   // Yaw angular speed (rad/s) 
        MAVLinkMessageFormat(); 
    }
}


// MAVLink: POSITION_TARGET_GLOBAL_INT 
void VehicleTelemetry::MAVLinkPositionTargetGlobalIntSendPeriodic(Vehicle &vehicle)
{
    if (mavlink.position_target_global_int_msg_timing.enable && 
       (++mavlink.position_target_global_int_msg_timing.count >= 
          mavlink.position_target_global_int_msg_timing.count_lim))
    {
        mavlink.position_target_global_int_msg_timing.count = RESET; 

        mavlink_msg_position_target_global_int_pack_chan(
            system_id, 
            component_id, 
            channel, 
            &msg, 
            vehicle.auxiliary.time_usec,             // Time since boot 
            vehicle.navigation.coordinate_frame,     // Coordinate frame (MAV_FRAME) 
            vehicle.navigation.position_type_mask,   // Ignored dimensions (POSITION_TARGET_TYPEMASK) 
            vehicle.navigation.current.lat,          // Latitude 
            vehicle.navigation.current.lon,          // Longitude 
            (float)vehicle.navigation.current.alt,   // Altitude 
            0.0,                                     // X velocity in NED frame (m/s) 
            0.0,                                     // Y velocity in NED frame (m/s) 
            0.0,                                     // Z velocity in NED frame (m/s) 
            0.0,                                     // X acceleration or force in NED frame (N) 
            0.0,                                     // y acceleration or force in NED frame (N) 
            0.0,                                     // Z acceleration or force in NED frame (N) 
            0.0,                                     // Yaw setpoint (rad) 
            0.0);                                    // Yaw rate setpoint (rad/s) 
        MAVLinkMessageFormat(); 
    }
}


// MAVLink: NAV_CONTROLLER 
void VehicleTelemetry::MAVLinkNavControllerSendPeriodic(Vehicle &vehicle)
{
    if (mavlink.nav_controller_output_msg_timing.enable && 
       (++mavlink.nav_controller_output_msg_timing.count >= 
          mavlink.nav_controller_output_msg_timing.count_lim))
    {
        mavlink.nav_controller_output_msg_timing.count = RESET; 

        mavlink_msg_nav_controller_output_pack_chan(
            system_id, 
            component_id, 
            channel, 
            &msg, 
            0,                                      // Current desired roll 
            0,                                      // Current desired pitch 
            vehicle.navigation.heading,             // Current desired heading 
            vehicle.navigation.target_heading,      // Bearing to current waypoint/target 
            vehicle.navigation.waypoint_distance,   // Distance to active waypoint 
            0,                                      // Current altitude error 
            0,                                      // Current airspeed error 
            0);                                     // Current crosstrack error on x-y plane 
        MAVLinkMessageFormat(); 
    }
}


// MAVLink: LOCAL_POSITION_NED 
void VehicleTelemetry::MAVLinkLocalPositionNEDSendPeriodic(Vehicle &vehicle)
{
    if (mavlink.local_position_ned_msg_timing.enable && 
       (++mavlink.local_position_ned_msg_timing.count >= 
          mavlink.local_position_ned_msg_timing.count_lim))
    {
        mavlink.local_position_ned_msg_timing.count = RESET; 

        mavlink_msg_local_position_ned_pack_chan(
            system_id, 
            component_id, 
            channel, 
            &msg, 
            vehicle.auxiliary.time_usec,   // Time since boot 
            0,                             // X position (m) 
            0,                             // Y position (m) 
            0,                             // Z position (m) 
            0,                             // X speed (m/s) 
            0,                             // Y speed (m/s) 
            0);                            // Z speed (m/s) 
        MAVLinkMessageFormat(); 
    }
}


// MAVLink: GLOBAL_POSITION_INT 
void VehicleTelemetry::MAVLinkGlobalPositionIntSendPeriodic(Vehicle &vehicle)
{
    if (mavlink.global_pos_int_msg_timing.enable && 
       (++mavlink.global_pos_int_msg_timing.count >= 
          mavlink.global_pos_int_msg_timing.count_lim))
    {
        mavlink.global_pos_int_msg_timing.count = RESET; 

        mavlink_msg_global_position_int_pack_chan(
            system_id, 
            component_id, 
            channel, 
            &msg, 
            vehicle.auxiliary.time_usec,      // Time since boot 
            vehicle.navigation.current.lat,   // Latitude 
            vehicle.navigation.current.lon,   // Longitude 
            vehicle.navigation.current.alt,   // Altitude 
            vehicle.navigation.current.alt,   // Relative altitude (above home) 
            0,                                // X velocity 
            0,                                // Y velocity 
            0,                                // Z velocity 
            vehicle.navigation.heading);      // Heading (yaw angle) 
        MAVLinkMessageFormat(); 
    }
}

//=======================================================================================
