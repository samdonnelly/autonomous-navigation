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
void VehicleTelemetry::MAVLinkMessageDecode(
    SemaphoreHandle_t &mutex, 
    VehicleHardware &hardware)
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
    if (hardware.data_ready.telemetry_ready == FLAG_SET)
    {
        hardware.data_ready.telemetry_ready = FLAG_CLEAR; 
        data_index = RESET; 

        // Get a copy of the data so we don't have to hold the comms mutex throughout the 
        // whole decoding process. 
        xSemaphoreTake(mutex, portMAX_DELAY); 
        hardware.TelemetryGet(data_size, data_buff); 
        xSemaphoreGive(mutex); 
        
        // Look at each byte of the received data and try to decode MAVLink messages 
        // until there is no more data to check. 
        while (data_index < data_size)
        {
            if (mavlink_parse_char(channel, 
                                   data_buff[data_index++], 
                                   &msg, 
                                   &status))
            {
                MAVLinkPayloadDecode(); 
            }
        }
    }

    // Perform any needed higher-level action. 
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

//=======================================================================================


//=======================================================================================
// Heartbeat protocol 

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


// Heartbeat message send 
// Add flight mode 

//=======================================================================================


//=======================================================================================
// Parameter protocol 

// MAVLink: PARAM_REQUEST_LIST 
void VehicleTelemetry::MAVLinkParamRequestList(void)
{
    // 
}

//=======================================================================================


//=======================================================================================
// Mission protocol 

// MAVLink: MISSION_REQUEST 
void VehicleTelemetry::MAVLinkMissionRequest(void)
{
    // 
}

//=======================================================================================


//=======================================================================================
// Command protocol 

// MAVLink: COMMAND_LONG 
void VehicleTelemetry::MAVLinkCommandLong(void)
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

    // // Acknowledge the command 
    // mavlink_msg_command_ack_pack_chan(
    //     system_data.system_id, 
    //     system_data.component_id, 
    //     system_data.channel, 
    //     &system_data.msg, 
    //     system_data.command_long_msg_gcs.command, 
    //     MAV_RESULT_ACCEPTED, 
    //     ZERO, 
    //     ZERO, 
    //     SIK_TEST_GCS_ID, 
    //     MAV_COMP_ID_MISSIONPLANNER); 
    // sik_radio_test_mavlink_send_msg(); 

    // // Perform the needed action based on the command 
    // uint16_t cmd_id = (uint16_t)system_data.command_long_msg_gcs.param1; 

    // switch (cmd_id)
    // {
    //     case MAVLINK_MSG_ID_AUTOPILOT_VERSION: 
    //         break; 

    //     default: 
    //         break; 
    // }
}


// Command decode 

//=======================================================================================


//=======================================================================================
// Other messages 

// MAVLink: REQUEST_DATA_STREAM 
void VehicleTelemetry::MAVLinkRequestDataStream(void)
{
    // 
} 

//=======================================================================================
