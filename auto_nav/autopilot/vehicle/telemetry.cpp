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
        data_index = RESET; 

        // Get a copy of the data so we don't have to hold the comms mutex throughout the 
        // whole decoding process. 
        xSemaphoreTake(vehicle.comms_mutex, portMAX_DELAY); 
        vehicle.hardware.TelemetryGet(data_size, data_buff); 
        xSemaphoreGive(vehicle.comms_mutex); 
        
        // Look at each byte of the received data and try to decode MAVLink messages 
        // until there is no more data to check. 
        while (data_index < data_size)
        {
            if (mavlink_parse_char(channel, 
                                   data_buff[data_index++], 
                                   &msg, 
                                   &status))
            {
                MAVLinkPayloadDecode(vehicle); 
            }
        }
    }

    // Look for a flag that indicates if a telemetry write event needs to be queued. 
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
    // 
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
