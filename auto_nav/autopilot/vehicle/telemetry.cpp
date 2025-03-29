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

#include "vehicle.h" 

//=======================================================================================


//=======================================================================================
// Macros 

// Periodic message send timing data. The send period is dependent on the 
// TELEMETRY_ENCODE event being queued in the corresponding timer thread listed below. 
#define TELEMETRY_SEND_PERIOD S_TO_MS * PERIODIC_TIMER_250MS_PERIOD / configTICK_RATE_HZ 

#define HB_SEND_FREQ 1   // Heartbeat message send frequency 
#define PV_SEND_FREQ 4   // Parameter value message send frequency 

//=======================================================================================


//=======================================================================================
// Initialization 

// Constructor 
VehicleTelemetry::VehicleTelemetry(uint8_t vehicle_type) 
    : channel(MAVLINK_COMM_0), 
      system_id(VS_SYSTEM_ID), 
      component_id(MAV_COMP_ID_AUTOPILOT1), 
      system_id_gcs(VS_SYSTEM_ID_GCS), 
      component_id_gcs(MAV_COMP_ID_MISSIONPLANNER), 
      data_in_index(RESET), 
      data_in_size(RESET), 
      data_out_size(RESET), 
      mission_item_index(RESET), 
      mission_resend_counter(RESET)
{
    mavlink.heartbeat_msg = 
    {
        .custom_mode = RESET, 
        .type = vehicle_type, 
        .autopilot = MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY, 
        .base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | 
                     MAV_MODE_FLAG_MANUAL_INPUT_ENABLED | 
                     MAV_MODE_FLAG_GUIDED_ENABLED | 
                     MAV_MODE_FLAG_SAFETY_ARMED, 
        .system_status = MAV_STATE_ACTIVE, 
        .mavlink_version = RESET   // Gets overwritten by MAVLink 
    };

    uint8_t hb_lim = S_TO_MS / (HB_SEND_FREQ * TELEMETRY_SEND_PERIOD); 
    uint8_t pv_lim = S_TO_MS / (PV_SEND_FREQ * TELEMETRY_SEND_PERIOD); 

    mavlink.heartbeat_msg_timing                  = { RESET, hb_lim, FLAG_SET }; 
    mavlink.raw_imu_msg_timing                    = { RESET, RESET, FLAG_CLEAR }; 
    mavlink.gps_raw_int_msg_timing                = { RESET, RESET, FLAG_CLEAR }; 
    mavlink.rc_channels_scaled_msg_timing         = { RESET, RESET, FLAG_CLEAR }; 
    mavlink.rc_channels_raw_msg_timing            = { RESET, RESET, FLAG_CLEAR }; 
    mavlink.servo_output_raw_msg_timing           = { RESET, RESET, FLAG_CLEAR }; 
    mavlink.attitude_msg_timing                   = { RESET, RESET, FLAG_CLEAR }; 
    mavlink.position_target_global_int_msg_timing = { RESET, RESET, FLAG_CLEAR }; 
    mavlink.nav_controller_output_msg_timing      = { RESET, RESET, FLAG_CLEAR }; 
    mavlink.local_position_ned_msg_timing         = { RESET, RESET, FLAG_CLEAR }; 
    mavlink.global_pos_int_msg_timing             = { RESET, RESET, FLAG_CLEAR }; 
    mavlink.param_value_msg_timing                = { RESET, pv_lim, FLAG_CLEAR }; 

    memset((void *)data_in_buff, RESET, sizeof(data_in_buff)); 
    memset((void *)data_out_buff, RESET, sizeof(data_out_buff)); 

    timers.heartbeat = RESET; 
    timers.mission_upload = RESET; 

    status.heartbeat = FLAG_CLEAR; 
    status.mission_upload = FLAG_CLEAR; 
    status.mission_item_reached = FLAG_CLEAR; 
}

//=======================================================================================


//=======================================================================================
// MAVLink message decoding 

// MAVLink message decode 
void VehicleTelemetry::MAVLinkMessageDecode(Vehicle &vehicle)
{
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
                                   &msg_status))
            {
                MAVLinkPayloadDecode(vehicle); 
            }
        }
    }

    // Message decoding is checked periodically so message timers are checked at the 
    // same time. 
    MsgTimerChecks(); 

    // Send any needed messages in response to messages received and timeouts. 
    MAVLinkMessageSend(vehicle); 
}


// Message timer checks 
void VehicleTelemetry::MsgTimerChecks(void)
{
    // Connection (heartbeat) timeout 
    if (status.heartbeat && (timers.heartbeat++ >= VS_HEARTBEAT_TIMEOUT))
    {
        // Have not seen a heartbeat message from a GCS for too long. The system is 
        // considered to be disconnected from telemetry. 
        status.heartbeat = FLAG_CLEAR; 
    }

    // If a mission upload sequence has been initiated by the GCS then check if the 
    // vehicle hasn't seen the appropriate response. If it hasn't then resend the mission 
    // item request a number of times before cancelling the operation. 
    if (status.mission_upload && (timers.mission_upload++ >= VS_MISSION_TIMEOUT))
    {
        if (mission_resend_counter++ < VS_MISSION_RESEND)
        {
            MAVLinkMissionRequestIntSend(mavlink.mission_count_msg_gcs.mission_type); 
        }
        else 
        {
            status.mission_upload = FLAG_CLEAR; 
            MAVLinkMissionAckSend(
                MAV_MISSION_OPERATION_CANCELLED, 
                mavlink.mission_count_msg_gcs.mission_type, 
                mavlink.mission_count_msg_gcs.opaque_id); 
        }
    }
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

        case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: 
            MAVLinkMissionRequestListReceive(vehicle); 
            break; 

        case MAVLINK_MSG_ID_MISSION_COUNT: 
            MAVLinkMissionCountReceive(vehicle); 
            break; 

        case MAVLINK_MSG_ID_MISSION_REQUEST: 
            MAVLinkMissionRequestReceive(vehicle); 
            break; 

        case MAVLINK_MSG_ID_MISSION_ITEM_INT: 
            MAVLinkMissionItemIntReceive(vehicle); 
            break; 

        case MAVLINK_MSG_ID_MISSION_ACK: 
            MAVLinkMissionAckReceive(); 
            break; 

        case MAVLINK_MSG_ID_MISSION_SET_CURRENT: 
            MAVLinkMissionSetCurrentReceive(vehicle); 
            break; 

        case MAVLINK_MSG_ID_MISSION_CLEAR_ALL: 
            MAVLinkMissionClearAllReceive(vehicle); 
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
    MAVLinkMissionItemReachedSend(vehicle); 

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
        // The specific vehicles comms thread must release the telemetry output semaphore 
        // after sending the telemetry data. This semaphore exists because there are both 
        // periodic message sends and sends in response to incoming messages which have 
        // the potential the overwrite the sending buffer if not protected. 
        osSemaphoreAcquire(vehicle.telemetry_out_semaphore, portMAX_DELAY); 
        vehicle.hardware.TelemetrySet(data_out_size, data_out_buff); 
        vehicle.CommsEventQueue((uint8_t)Vehicle::CommsEvents::TELEMETRY_WRITE); 
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
        (msg.sysid == system_id_gcs) && 
        (msg.compid == component_id_gcs))
    {
        timers.heartbeat = RESET; 
        status.heartbeat = FLAG_SET; 
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

// MAVLink: MISSION_REQUEST_LIST receive 
void VehicleTelemetry::MAVLinkMissionRequestListReceive(Vehicle &vehicle)
{
    mavlink_msg_mission_request_list_decode(
        &msg, 
        &mavlink.mission_request_list_msg_gcs); 

    // This system is only concerned with messages meant for this system. If the taget 
    // system and component ID in the message does not match this system then abort. 
    if ((mavlink.mission_request_list_msg_gcs.target_system != system_id) || 
        (mavlink.mission_request_list_msg_gcs.target_component != component_id))
    {
        return; 
    }

    MAVLinkMissionCountSend(vehicle); 
}


// MAVLink: MISSION_COUNT receive 
void VehicleTelemetry::MAVLinkMissionCountReceive(Vehicle &vehicle)
{
    mavlink_msg_mission_count_decode(
        &msg, 
        &mavlink.mission_count_msg_gcs); 

    // This system is only concerned with messages meant for this system. If the taget 
    // system and component ID in the message does not match this system then abort. 
    if ((mavlink.mission_count_msg_gcs.target_system != system_id) || 
        (mavlink.mission_count_msg_gcs.target_component != component_id))
    {
        return; 
    }

    // If the provided count is 0 then that's the same as clearing the vehicle's 
    // currently stored mission. If the count is not 0 then we proceed to starting 
    // the mission upload process. 
    if (mavlink.mission_count_msg_gcs.count == 0)
    {
        ClearMission(vehicle, mavlink.mission_count_msg_gcs.mission_type); 
    }
    else 
    {
        // TODO overwrite mission file not in use so it's blank or has a header then 
        // add the home location as the first item. 

        status.mission_upload = FLAG_SET; 
        mission_resend_counter = RESET; 
        mission_item_index = RESET; 
        MAVLinkMissionRequestIntSend(mavlink.mission_count_msg_gcs.mission_type); 
    }
}


// MAVLink: MISSION_COUNT send 
void VehicleTelemetry::MAVLinkMissionCountSend(Vehicle &vehicle)
{
    mavlink_msg_mission_count_pack_chan(
        system_id, 
        component_id, 
        channel, 
        &msg, 
        system_id_gcs, 
        component_id_gcs, 
        vehicle.memory.mission_size, 
        vehicle.memory.mission_type, 
        vehicle.memory.mission_id); 
    MAVLinkMessageFormat(); 
}


// MAVLink: MISSION_REQUEST receive 
void VehicleTelemetry::MAVLinkMissionRequestReceive(Vehicle &vehicle)
{
    // Mission planner sends MISSION_REQUEST messages despite the message being 
    // deprecated by MAVLink in favour of MISSION_REQUEST_INT. When this message is 
    // received, Mission Planner expects MISSION_ITEM_INT in return as discovered through 
    // trial and error (i.e. MISSION_ITEM does not work). 
    // MISSION_ITEM_INT takes the system and component IDs in its payload of the system 
    // the message is being sent to. This should not be confused with the full MAVLink 
    // message system and component IDs which identify where a message is coming from. 
    // Mission Planner varries from standard MAVLink mission protocol in that the item 
    // at mission sequence 0 is the home location, not the first waypoint location. 

    mavlink_msg_mission_request_decode(
        &msg, 
        &mavlink.mission_request_msg_gcs); 

    // This system is only concerned with messages meant for this system. If the taget 
    // system and component ID in the message does not match this system then abort. 
    if ((mavlink.mission_request_msg_gcs.target_system != system_id) || 
        (mavlink.mission_request_msg_gcs.target_component != component_id))
    {
        return; 
    }

    MAVLinkMissionItemIntSend(vehicle); 
}


// MAVLink: MISSION_REQUEST_INT send 
void VehicleTelemetry::MAVLinkMissionRequestIntSend(uint8_t mission_type)
{
    mavlink_msg_mission_request_int_pack_chan(
        system_id, 
        component_id, 
        channel, 
        &msg, 
        system_id_gcs, 
        component_id_gcs, 
        mission_item_index, 
        mission_type); 
    MAVLinkMessageFormat(); 

    // Start mission upload timer 
    timers.mission_upload = RESET; 
}


// MAVLink: MISSION_ITEM_INT receive 
void VehicleTelemetry::MAVLinkMissionItemIntReceive(Vehicle &vehicle)
{
    mavlink_msg_mission_item_int_decode(
        &msg, 
        &mavlink.mission_item_int_msg_gcs); 

    // This system is only concerned with messages meant for this system. If the taget 
    // system and component ID in the message does not match this system then abort. 
    if ((mavlink.mission_item_int_msg_gcs.target_system != system_id) || 
        (mavlink.mission_item_int_msg_gcs.target_component != component_id))
    {
        return; 
    }

    // Check if the mission upload process has been initiated. If so then proceed to 
    // evaluate mission item. 
    if (status.mission_upload)
    {
        // Check if the received MISSION_ITEM_INT sequence number matches the requested 
        // item number. If not then the received items are not in the expected sequence. 
        if (mavlink.mission_item_int_msg_gcs.seq == mission_item_index)
        {
            // TODO Append/save the mission item. May need to check other item properties. 
            // Must be a global coordinate frame. 
            // vehicle.memory.mission_items; 

            // Check if the received MISSION_ITEM_INT sequence number matches the total 
            // number of items expected in the mission upload. If so then the mission is 
            // acknowledged as having been successfully received. Otherwise proceed to 
            // request the next expected item. 
            if (mavlink.mission_item_int_msg_gcs.seq == (mavlink.mission_count_msg_gcs.count - 1))
            {
                // TODO generate a new mission id (opaque_id) 
                // Save mission ID, mission size and mission type in memory 

                status.mission_upload = FLAG_CLEAR; 
                vehicle.memory.mission_size = mavlink.mission_count_msg_gcs.count; 
                vehicle.memory.mission_type = mavlink.mission_item_int_msg_gcs.mission_type; 

                MAVLinkMissionAckSend(
                    MAV_MISSION_ACCEPTED, 
                    mavlink.mission_item_int_msg_gcs.mission_type, 
                    vehicle.memory.mission_id); 
            }
            else 
            {
                mission_item_index++; 
                mission_resend_counter = RESET; 
                MAVLinkMissionRequestIntSend(mavlink.mission_item_int_msg_gcs.mission_type); 
            }
        }
        else 
        {
            MAVLinkMissionAckSend(
                MAV_MISSION_INVALID_SEQUENCE, 
                mavlink.mission_item_int_msg_gcs.mission_type, 
                mavlink.mission_count_msg_gcs.opaque_id); 
        }
    }
}


// MAVLink: MISSION_ITEM_INT send 
void VehicleTelemetry::MAVLinkMissionItemIntSend(Vehicle &vehicle)
{
    // Only send the mission item if it exists 
    if (mavlink.mission_request_msg_gcs.seq < vehicle.memory.mission_size)
    {
        mavlink_msg_mission_item_int_encode_chan(
            system_id, 
            component_id, 
            channel, 
            &msg, 
            &vehicle.memory.mission_items[mavlink.mission_request_msg_gcs.seq]); 
        MAVLinkMessageFormat(); 
    }
}


// MAVLink: MISSION_ACK send 
void VehicleTelemetry::MAVLinkMissionAckSend(
    MAV_MISSION_RESULT result, 
    uint8_t mission_type, 
    uint32_t opaque_id)
{
    mavlink_msg_mission_ack_pack_chan(
        system_id, 
        component_id, 
        channel, 
        &msg, 
        system_id_gcs, 
        component_id_gcs, 
        result, 
        mission_type, 
        opaque_id); 
    MAVLinkMessageFormat(); 
}


// MAVLink: MISSION_ACK receive 
void VehicleTelemetry::MAVLinkMissionAckReceive(void)
{
    mavlink_msg_mission_ack_decode(
        &msg, 
        &mavlink.mission_ack_msg_gcs); 

    // This system is only concerned with messages meant for this system. If the taget 
    // system and component ID in the message does not match this system then abort. 
    if ((mavlink.mission_ack_msg_gcs.target_system != system_id) || 
        (mavlink.mission_ack_msg_gcs.target_component != component_id))
    {
        return; 
    }

    // Unclear what should be done when this is received. Whether a GCS specifies 
    // success or failure on mission download, MAVLink mission protocol doesn't 
    // specify a particular action. 
}


// MAVLink: MISSION_CURRENT send 
void VehicleTelemetry::MAVLinkMissionCurrentSend(Vehicle &vehicle)
{
    mavlink_msg_mission_current_pack_chan(
        system_id, 
        component_id, 
        channel, 
        &msg, 
        0,   // Sequence number 
        vehicle.memory.mission_size,   // ? 
        0,   // Mission state 
        0,   // Mission mode 
        vehicle.memory.mission_id,   // ? 
        0,   // Fence ID 
        0);   // Ralley point ID 
    MAVLinkMessageFormat(); 
}


// MAVLink: MISSION_SET_CURRENT receive 
void VehicleTelemetry::MAVLinkMissionSetCurrentReceive(Vehicle &vehicle)
{
    mavlink_msg_mission_set_current_decode(
        &msg, 
        &mavlink.mission_set_current_msg_gcs); 

    // This system is only concerned with messages meant for this system. If the taget 
    // system and component ID in the message does not match this system then abort. 
    if ((mavlink.mission_set_current_msg_gcs.target_system != system_id) || 
        (mavlink.mission_set_current_msg_gcs.target_component != component_id))
    {
        return; 
    }

    // Update the mission item if valid 
    if (mavlink.mission_set_current_msg_gcs.seq < vehicle.memory.mission_size)
    {
        // TODO 

        MAVLinkMissionCurrentSend(vehicle); 
    }
}


// MAVLink: MISSION_CLEAR_ALL receive 
void VehicleTelemetry::MAVLinkMissionClearAllReceive(Vehicle &vehicle)
{
    mavlink_msg_mission_clear_all_decode(
        &msg, 
        &mavlink.mission_clear_all_msg_gcs); 

    // This system is only concerned with messages meant for this system. If the taget 
    // system and component ID in the message does not match this system then abort. 
    if ((mavlink.mission_clear_all_msg_gcs.target_system != system_id) || 
        (mavlink.mission_clear_all_msg_gcs.target_component != component_id))
    {
        return; 
    }

    ClearMission(vehicle, mavlink.mission_clear_all_msg_gcs.mission_type); 
}


// MAVLink: MISSION_ITEM_REACHED send 
void VehicleTelemetry::MAVLinkMissionItemReachedSend(Vehicle &vehicle)
{
    if (status.mission_item_reached)
    {
        mavlink_msg_mission_item_reached_pack_chan(
            system_id, 
            component_id, 
            channel, 
            &msg, 
            vehicle.memory.mission_index); 
        MAVLinkMessageFormat(); 

        status.mission_item_reached = FLAG_CLEAR; 
    }
}


// Clear current mission 
void VehicleTelemetry::ClearMission(
    Vehicle &vehicle, 
    uint8_t mission_type)
{
    // TODO Clear the vehicle mission and generate a new opaque_id 
    // Acknowledge that the mission has been cleared 
    MAVLinkMissionAckSend(
        MAV_MISSION_ACCEPTED, 
        mission_type, 
        vehicle.memory.mission_id); 
}


// Set flag to trigger a MISSION_ITEM_REACHED message send 
void VehicleTelemetry::MAVLinkMissionItemReachedSet(void)
{
    status.mission_item_reached = FLAG_SET; 
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
        MAVLinkCommandACKSend(); 
    }
}


// MAVLink command: REQUEST_MESSAGE 
void VehicleTelemetry::MAVLinkCommandRequestMessageReceive(void)
{
    uint16_t msg_id = (uint16_t)mavlink.command_long_msg_gcs.param1; 

    switch (msg_id)
    {
        case MAVLINK_MSG_ID_AUTOPILOT_VERSION: 
            MAVLinkCommandACKSend(); 
            MAVLinkAutopilotVersionSend(); 
            break; 

        default: 
            break; 
    }
}


// MAVLink command: COMMAND_ACK 
void VehicleTelemetry::MAVLinkCommandACKSend(void)
{
    mavlink_msg_command_ack_pack_chan(
        system_id, 
        component_id, 
        channel, 
        &msg, 
        mavlink.command_long_msg_gcs.command, 
        MAV_RESULT_ACCEPTED, 
        0, 
        0, 
        system_id_gcs, 
        component_id_gcs); 
    MAVLinkMessageFormat(); 
}

//=======================================================================================


//=======================================================================================
// Other messages 

// MAVLink: REQUEST_DATA_STREAM 
void VehicleTelemetry::MAVLinkRequestDataStreamReceive(void)
{
    // Mission Planner sends this message to request data from the autopilot. This 
    // message is often sent in bursts to request all the needed messages. 

    mavlink_msg_request_data_stream_decode(
        &msg, 
        &mavlink.request_data_stream_msg_gcs); 
    
    // This system is only concerned with messages meant for this system. If the taget 
    // system and component ID in the message does not match this system then abort. 
    if ((mavlink.request_data_stream_msg_gcs.target_system != system_id) || 
        (mavlink.request_data_stream_msg_gcs.target_component != component_id))
    {
        return; 
    }

    // This message comes with a cooresponding requested message rate. The calculated 
    // timer counter limit is the same calculation for each requested message so it's 
    // done once here and assigned to the requested message. Note that the periodic 
    // interrupt period should be equipped to handle whatever the requested rate is. 
    uint8_t timer_limit = (uint8_t)(S_TO_MS / 
        (mavlink.request_data_stream_msg_gcs.req_message_rate * TELEMETRY_SEND_PERIOD)); 
    uint8_t enable = mavlink.request_data_stream_msg_gcs.start_stop; 

    // Enable/disable the requested message and assign the message timer counter limit 
    // so it gets sent to the GCS at the requested rate. 
    switch (mavlink.request_data_stream_msg_gcs.req_stream_id)
    {
        case MAV_DATA_STREAM_ALL: 
            break; 
        
        case MAV_DATA_STREAM_RAW_SENSORS: 
            mavlink.raw_imu_msg_timing.enable = enable; 
            mavlink.raw_imu_msg_timing.count_lim = timer_limit; 

            mavlink.gps_raw_int_msg_timing.enable = enable; 
            mavlink.gps_raw_int_msg_timing.count_lim = timer_limit; 
            break; 

        case MAV_DATA_STREAM_EXTENDED_STATUS: 
            break; 

        case MAV_DATA_STREAM_RC_CHANNELS: 
            mavlink.rc_channels_scaled_msg_timing.enable = enable; 
            mavlink.rc_channels_scaled_msg_timing.count_lim = timer_limit; 

            mavlink.rc_channels_raw_msg_timing.enable = enable; 
            mavlink.rc_channels_raw_msg_timing.count_lim = timer_limit; 

            mavlink.servo_output_raw_msg_timing.enable = enable; 
            mavlink.servo_output_raw_msg_timing.count_lim = timer_limit; 
            break; 

        case MAV_DATA_STREAM_RAW_CONTROLLER: 
            mavlink.attitude_msg_timing.enable = enable; 
            mavlink.attitude_msg_timing.count_lim = timer_limit; 

            mavlink.position_target_global_int_msg_timing.enable = enable; 
            mavlink.position_target_global_int_msg_timing.count_lim = timer_limit; 

            mavlink.nav_controller_output_msg_timing.enable = enable; 
            mavlink.nav_controller_output_msg_timing.count_lim = timer_limit; 
            break; 

        case MAV_DATA_STREAM_POSITION: 
            mavlink.local_position_ned_msg_timing.enable = enable; 
            mavlink.local_position_ned_msg_timing.count_lim = timer_limit; 

            mavlink.global_pos_int_msg_timing.enable = enable; 
            mavlink.global_pos_int_msg_timing.count_lim = timer_limit; 
            break; 

        case MAV_DATA_STREAM_EXTRA1: 
            break; 

        case MAV_DATA_STREAM_EXTRA2: 
            break; 

        case MAV_DATA_STREAM_EXTRA3: 
            break; 

        default: 
            break; 
    }
}


// MAVLink: AUTOPILOT_VERSION 
void VehicleTelemetry::MAVLinkAutopilotVersionSend(void)
{
    uint8_t version[8], hw_uid[18]; 
    memset((void *)version, RESET, sizeof(version)); 
    memset((void *)hw_uid, RESET, sizeof(hw_uid)); 

    uint32_t capabilities = MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT | 
                            MAV_PROTOCOL_CAPABILITY_MISSION_INT | 
                            MAV_PROTOCOL_CAPABILITY_COMMAND_INT | 
                            MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED | 
                            MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT | 
                            MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET | 
                            MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                            MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION | 
                            MAV_PROTOCOL_CAPABILITY_MAVLINK2; 

    mavlink_msg_autopilot_version_pack_chan(
        system_id, 
        component_id, 
        channel, 
        &msg, 
        capabilities,   // Autopilot capabilities (bitmap) 
        0,              // Firmware version number 
        0,              // Middleware version number 
        0,              // Operating system version number 
        0,              // HW/board version 
        version,        // Custom firmware version 
        version,        // Custom middleware version 
        version,        // Custom operating system version 
        0,              // ID of board vendor 
        0,              // ID of the product 
        0,              // UID if provided by hardware 
        hw_uid);        // UID if provided by hardware 
    MAVLinkMessageFormat(); 
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
