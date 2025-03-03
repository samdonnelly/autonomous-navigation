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
// MAVLink 

// MAVLink message decode 
void VehicleTelemetry::MAVLinkMessageDecode(VehicleHardware &hardware)
{
    // Check for a connection (heartbeat) timeout 
    if (heartbeat_status_timer++ >= VS_HEARTBEAT_TIMEOUT)
    {
        // Have not seen a heartbeat message from a GCS for too long. The system is 
        // considered to be disconnected. 
        heartbeat_status_timer--; 
        connected = FLAG_CLEAR; 
    }

    // No data protection is done here because this action is queued right after a 
    // telemetry read event so it's unlikely the data will be accessed simultaneously. 

    // Check if new data is available. If so then get the data and process it. 
    if (hardware.data_ready.telemetry_ready == FLAG_SET)
    {
        hardware.data_ready.telemetry_ready = FLAG_CLEAR; 
        data_index = RESET; 
        
        // Look at each byte of the received data and try to decode MAVLink messages 
        // until there is no more data to check. 
        while (data_index < hardware.telemetry_data_size)
        {
            if (mavlink_parse_char(channel, 
                                   *(hardware.telemetry_buff + data_index++), 
                                   &msg, 
                                   &status))
            {
                MAVLinkPayloadDecode(); 
            }
        }
    }
}


// MAVLink message payload decode 
void VehicleTelemetry::MAVLinkPayloadDecode(void)
{
    switch (msg.msgid)
    {
        case MAVLINK_MSG_ID_HEARTBEAT: 
            MAVLinkHeartbeat(); 
            break; 

        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: 
            MAVLinkParamRequestList(); 
            break; 

        case MAVLINK_MSG_ID_MISSION_REQUEST: 
            MAVLinkMissionRequest(); 
            break; 

        case MAVLINK_MSG_ID_REQUEST_DATA_STREAM: 
            MAVLinkRequestDataStream(); 
            break; 
        
        case MAVLINK_MSG_ID_COMMAND_LONG: 
            MAVLinkCommandLong(); 
            break; 
        
        default: 
            break; 
    }
}


// MAVLink: HEARTBEAT 
void VehicleTelemetry::MAVLinkHeartbeat(void)
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


// MAVLink: PARAM_REQUEST_LIST 
void VehicleTelemetry::MAVLinkParamRequestList(void)
{
    // 
} 


// MAVLink: MISSION_REQUEST 
void VehicleTelemetry::MAVLinkMissionRequest(void)
{
    // 
} 


// MAVLink: REQUEST_DATA_STREAM 
void VehicleTelemetry::MAVLinkRequestDataStream(void)
{
    // 
} 


// MAVLink: COMMAND_LONG 
void VehicleTelemetry::MAVLinkCommandLong(void)
{
    // 
} 

//=======================================================================================
