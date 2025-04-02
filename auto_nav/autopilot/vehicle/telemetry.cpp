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

#define HB_SEND_FREQ 1   // HEARTBEAT message send frequency 
#define PV_SEND_FREQ 4   // PARAMETER_VALUE value message send frequency 
#define MC_SEND_FREQ 1   // MISSION_CURRENT message send frequency 

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
    uint8_t mc_lim = S_TO_MS / (MC_SEND_FREQ * TELEMETRY_SEND_PERIOD); 

    // Outgoing message timing 
    mavlink.heartbeat_msg_timing                  = { RESET, hb_lim, FLAG_SET }; 
    mavlink.param_value_msg_timing                = { RESET, pv_lim, FLAG_CLEAR }; 
    mavlink.mission_current_msg_timing            = { RESET, mc_lim, FLAG_CLEAR }; 
    mavlink.raw_imu_msg_timing                    = { RESET, RESET, FLAG_CLEAR }; 
    mavlink.gps_raw_int_msg_timing                = { RESET, RESET, FLAG_CLEAR }; 
    // mavlink.rc_channels_scaled_msg_timing         = { RESET, RESET, FLAG_CLEAR }; 
    // mavlink.rc_channels_raw_msg_timing            = { RESET, RESET, FLAG_CLEAR }; 
    // mavlink.servo_output_raw_msg_timing           = { RESET, RESET, FLAG_CLEAR }; 
    mavlink.attitude_msg_timing                   = { RESET, RESET, FLAG_CLEAR }; 
    mavlink.position_target_global_int_msg_timing = { RESET, RESET, FLAG_CLEAR }; 
    mavlink.nav_controller_output_msg_timing      = { RESET, RESET, FLAG_CLEAR }; 
    // mavlink.local_position_ned_msg_timing         = { RESET, RESET, FLAG_CLEAR }; 
    mavlink.global_pos_int_msg_timing             = { RESET, RESET, FLAG_CLEAR }; 

    memset((void *)data_in_buff, RESET, sizeof(data_in_buff)); 
    memset((void *)data_out_buff, RESET, sizeof(data_out_buff)); 

    timers.heartbeat = RESET; 
    timers.mission_upload = RESET; 

    status.heartbeat = FLAG_CLEAR; 
    status.param_read = FLAG_CLEAR; 
    status.mission_upload = FLAG_CLEAR; 
    status.mission_current = FLAG_CLEAR; 
    status.mission_item_reached = FLAG_CLEAR; 
}

//=======================================================================================


//=======================================================================================
// Message decoding 

// Message decode 
void VehicleTelemetry::MessageDecode(Vehicle &vehicle)
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
            MAVLinkMissionRequestSend(mavlink.mission_count_msg_gcs.mission_type); 
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

        case MAVLINK_MSG_ID_PARAM_REQUEST_READ: 
            MAVLinkParamRequestReadReceive(vehicle); 
            break; 

        case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: 
            MAVLinkMissionRequestListReceive(vehicle); 
            break; 

        case MAVLINK_MSG_ID_MISSION_COUNT: 
            MAVLinkMissionCountReceive(vehicle); 
            break; 

        case MAVLINK_MSG_ID_MISSION_REQUEST_INT: 
            MAVLinkMissionRequestIntReceive(vehicle); 
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
// Message encoding 

// Periodic message encode 
void VehicleTelemetry::MessageEncode(Vehicle &vehicle)
{
    // Heartbeat protocol 
    MAVLinkHeartbeatSend(); 

    // Parameter protocol 
    MAVLinkParamValueSend(vehicle); 

    // Mission protocol 
    MAVLinkMissionCurrentSend(vehicle); 
    MAVLinkMissionItemReachedSend(vehicle); 
    
    // Requestable periodic messages 
    MAVLinkRawIMUSendPeriodic(vehicle); 
    MAVLinkGPSRawIntSendPeriodic(vehicle); 
    // MAVLinkRCChannelScaledSendPeriodic(vehicle); 
    // MAVLinkRCChannelRawSendPeriodic(vehicle); 
    // MAVLinkServoOutputRawSendPeriodic(vehicle); 
    MAVLinkAttitudeSendPeriodic(vehicle); 
    MAVLinkPositionTargetGlobalIntSendPeriodic(vehicle); 
    MAVLinkNavControllerSendPeriodic(vehicle); 
    // MAVLinkLocalPositionNEDSendPeriodic(vehicle); 
    MAVLinkGlobalPositionIntSendPeriodic(vehicle); 

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

//==================================================
// Receive 

// MAVLink: PARAM_REQUEST_LIST receive 
void VehicleTelemetry::MAVLinkParamRequestListReceive(Vehicle &vehicle)
{
    mavlink_msg_param_request_list_decode(
        &msg, 
        &mavlink.param_request_list_msg_gcs); 
        
    if (VerifyVehicleIDs(mavlink.param_request_list_msg_gcs.target_system, 
                         mavlink.param_request_list_msg_gcs.target_component))
    {
        return; 
    }

    // Enable the PARAM_VALUE message which gets sent periodically until all parameters 
    // in the system have been sent. 
    mavlink.param_value_msg_timing.enable = FLAG_SET; 
    vehicle.memory.param_index = RESET; 
}


// MAVLink: PARAM_REQUEST_READ receive 
void VehicleTelemetry::MAVLinkParamRequestReadReceive(Vehicle &vehicle)
{
    mavlink_msg_param_request_read_decode(
        &msg, 
        &mavlink.param_request_read_msg_gcs); 

    if (VerifyVehicleIDs(mavlink.param_request_read_msg_gcs.target_system, 
                         mavlink.param_request_read_msg_gcs.target_component))
    {
        return; 
    }

    // MAVLink protocol gives the GCS the option on how to fetch the parameter. The 
    // index is checked to be within range in the PARAM_VALUE send function. 
    if (mavlink.param_request_read_msg_gcs.param_index < 0)
    {
        vehicle.memory.ParameterLookUp(mavlink.param_request_read_msg_gcs.param_id); 
    }
    else 
    {
        vehicle.memory.param_index = (uint8_t)mavlink.param_request_read_msg_gcs.param_index; 
    }

    status.param_read = FLAG_SET; 
    MAVLinkParamValueSend(vehicle); 
}


// MAVLink: PARAM_SET receive 
void VehicleTelemetry::MAVLinkParamSetReceive(Vehicle &vehicle)
{
    mavlink_msg_param_set_decode(
        &msg, 
        &mavlink.param_set_msg_gcs); 

    if (VerifyVehicleIDs(mavlink.param_set_msg_gcs.target_system, 
                         mavlink.param_set_msg_gcs.target_component))
    {
        return; 
    }

    vehicle.memory.ParameterSet(mavlink.param_set_msg_gcs.param_id, 
                                mavlink.param_set_msg_gcs.param_value); 

    status.param_read = FLAG_SET; 
    MAVLinkParamValueSend(vehicle); 
}

//==================================================


//==================================================
// Send 

// MAVLink: PARAM_VALUE send 
void VehicleTelemetry::MAVLinkParamValueSend(Vehicle &vehicle)
{
    if ((mavlink.param_value_msg_timing.enable && 
        (++mavlink.param_value_msg_timing.count >= mavlink.param_value_msg_timing.count_lim)) || 
        status.param_read)
    {
        mavlink.param_value_msg_timing.count = RESET; 
        status.param_read = FLAG_CLEAR; 
        
        if (vehicle.memory.param_index < parameters.size())
        {
            mavlink_msg_param_value_pack_chan(
                system_id, 
                component_id, 
                channel, 
                &msg, 
                parameters[vehicle.memory.param_index].name, 
                *parameters[vehicle.memory.param_index].value, 
                vehicle.memory.param_value_type, 
                parameters.size(), 
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

//==================================================
        
//=======================================================================================


//=======================================================================================
// MAVLink: Mission protocol 

//==================================================
// Receive 

// MAVLink: MISSION_REQUEST_LIST receive 
void VehicleTelemetry::MAVLinkMissionRequestListReceive(Vehicle &vehicle)
{
    mavlink_msg_mission_request_list_decode(
        &msg, 
        &mavlink.mission_request_list_msg_gcs); 

    if (VerifyVehicleIDs(mavlink.mission_request_list_msg_gcs.target_system, 
                         mavlink.mission_request_list_msg_gcs.target_component))
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

    if (VerifyVehicleIDs(mavlink.mission_count_msg_gcs.target_system, 
                         mavlink.mission_count_msg_gcs.target_component))
    {
        return; 
    }

    // TODO check how old missions are cleared when overwriting a mission. 

    // If the provided count is 0 then that's the same as clearing the vehicles currently 
    // stored mission. If the count is larger than the available mission item storage 
    // space then the upload must be rejected. Otherwise the mission upload sequence is 
    // started. 
    if (mavlink.mission_count_msg_gcs.count == 0)
    {
        ClearMission(vehicle, mavlink.mission_count_msg_gcs.mission_type); 
    }
    else if (mavlink.mission_count_msg_gcs.count > (MAX_MISSION_SIZE - HOME_OFFSET))
    {
        // The home location is stored at index 0 so the max number of mission items is 
        // one less than the total storage size. 

        MAVLinkMissionAckSend(
            MAV_MISSION_NO_SPACE, 
            mavlink.mission_count_msg_gcs.mission_type, 
            vehicle.memory.mission_id); 
    }
    else 
    {
        status.mission_upload = FLAG_SET; 
        mission_resend_counter = RESET; 
        mission_item_index = RESET; 
        MAVLinkMissionRequestSend(mavlink.mission_count_msg_gcs.mission_type); 
    }
}


// MAVLink: MISSION_REQUEST_INT receive 
void VehicleTelemetry::MAVLinkMissionRequestIntReceive(Vehicle &vehicle)
{
    // Mission Planner uses MISSION_REQUEST_INT when downloading a mission from the 
    // vehicle instead of MISSION_REQUEST like it used for getting the home location. 

    mavlink_msg_mission_request_int_decode(
        &msg, 
        &mavlink.mission_request_int_msg_gcs); 

    if (VerifyVehicleIDs(mavlink.mission_request_int_msg_gcs.target_system, 
                         mavlink.mission_request_int_msg_gcs.target_component))
    {
        return; 
    }

    MAVLinkMissionItemIntSend(vehicle, mavlink.mission_request_int_msg_gcs.seq); 
}


// MAVLink: MISSION_REQUEST receive 
void VehicleTelemetry::MAVLinkMissionRequestReceive(Vehicle &vehicle)
{
    // Mission Planner only uses MISSION_REQUEST for retrieving the home position, and 
    // when doing so expects MISSION_ITEM_INT in return. Mission Planner and Ardupilot 
    // vary from standard MAVLink mission protocol in that the item at mission sequence 
    // 0 is the home location, not the first waypoint location. 

    mavlink_msg_mission_request_decode(
        &msg, 
        &mavlink.mission_request_msg_gcs); 

    if (VerifyVehicleIDs(mavlink.mission_request_msg_gcs.target_system, 
                         mavlink.mission_request_msg_gcs.target_component))
    {
        return; 
    }

    MAVLinkMissionItemIntSend(vehicle, mavlink.mission_request_msg_gcs.seq); 
}


// MAVLink: MISSION_ITEM_INT receive 
void VehicleTelemetry::MAVLinkMissionItemIntReceive(Vehicle &vehicle)
{
    mavlink_msg_mission_item_int_decode(
        &msg, 
        &mavlink.mission_item_int_msg_gcs); 

    if (VerifyVehicleIDs(mavlink.mission_item_int_msg_gcs.target_system, 
                         mavlink.mission_item_int_msg_gcs.target_component))
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
            // The home location is stored at index 0 even though Mission Planner sends 
            // the first mission item with a sequence/index of 0. For this reason, the 
            // saved mission item is stored at +1 index and the sequence is incremented. 
            vehicle.memory.mission[mission_item_index + HOME_OFFSET] = mavlink.mission_item_int_msg_gcs; 
            vehicle.memory.mission[mission_item_index + HOME_OFFSET].seq++; 

            // Check if the received MISSION_ITEM_INT sequence number matches the total 
            // number of items expected in the mission upload. If so then the mission 
            // upload sequence is done. Otherwise proceed to request the next item. 
            if (mavlink.mission_item_int_msg_gcs.seq >= (mavlink.mission_count_msg_gcs.count - 1))
            {
                // Increment the mission ID to signify a new mission, update the mission 
                // size, acknowledge a successful mission upload and end the sequence. 
                status.mission_upload = FLAG_CLEAR; 
                vehicle.memory.mission_size = mavlink.mission_count_msg_gcs.count + HOME_OFFSET; 
                vehicle.memory.mission_type = mavlink.mission_item_int_msg_gcs.mission_type; 

                MAVLinkMissionAckSend(
                    MAV_MISSION_ACCEPTED, 
                    mavlink.mission_item_int_msg_gcs.mission_type, 
                    ++vehicle.memory.mission_id); 
            }
            else 
            {
                mission_item_index++; 
                mission_resend_counter = RESET; 
                MAVLinkMissionRequestSend(mavlink.mission_item_int_msg_gcs.mission_type); 
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


// MAVLink: MISSION_ACK receive 
void VehicleTelemetry::MAVLinkMissionAckReceive(void)
{
    mavlink_msg_mission_ack_decode(
        &msg, 
        &mavlink.mission_ack_msg_gcs); 

    if (VerifyVehicleIDs(mavlink.mission_ack_msg_gcs.target_system, 
                         mavlink.mission_ack_msg_gcs.target_component))
    {
        return; 
    }

    // Unclear what should be done when this is received. Whether a GCS specifies 
    // success or failure on mission download, MAVLink mission protocol doesn't 
    // specify a particular action. 
}


// MAVLink: MISSION_SET_CURRENT receive 
void VehicleTelemetry::MAVLinkMissionSetCurrentReceive(Vehicle &vehicle)
{
    mavlink_msg_mission_set_current_decode(
        &msg, 
        &mavlink.mission_set_current_msg_gcs); 

    if (VerifyVehicleIDs(mavlink.mission_set_current_msg_gcs.target_system, 
                         mavlink.mission_set_current_msg_gcs.target_component))
    {
        return; 
    }

    // Update the mission item if valid 
    // TODO verify sequence indexing 
    if (mavlink.mission_set_current_msg_gcs.seq < vehicle.memory.mission_size)
    {
        vehicle.memory.mission_index = mavlink.mission_set_current_msg_gcs.seq; 
        status.mission_current = FLAG_SET; 
        MAVLinkMissionCurrentSend(vehicle); 
    }
}


// MAVLink: MISSION_CLEAR_ALL receive 
void VehicleTelemetry::MAVLinkMissionClearAllReceive(Vehicle &vehicle)
{
    mavlink_msg_mission_clear_all_decode(
        &msg, 
        &mavlink.mission_clear_all_msg_gcs); 

    if (VerifyVehicleIDs(mavlink.mission_clear_all_msg_gcs.target_system, 
                         mavlink.mission_clear_all_msg_gcs.target_component))
    {
        return; 
    }

    ClearMission(vehicle, mavlink.mission_clear_all_msg_gcs.mission_type); 
}

//==================================================


//==================================================
// Send 

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


// MAVLink: MISSION_REQUEST send 
void VehicleTelemetry::MAVLinkMissionRequestSend(uint8_t mission_type)
{
    // Mission Planner prefers MISSION_REQUEST over MISSION_REQUEST_INT when doing a 
    // mission upload for some reason. Both work but it was observed that MISSION_REQUEST 
    // uploaded much faster. 

    mavlink_msg_mission_request_pack_chan(
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


// MAVLink: MISSION_ITEM_INT send 
void VehicleTelemetry::MAVLinkMissionItemIntSend(
    Vehicle &vehicle, 
    uint16_t sequence)
{
    // Mission Planner looks for MISSION_ITEM_INT when getting mission items from the 
    // vehicle during mission download and when getting the home location despite 
    // MISSION_REQUEST_INT and MISSION_REQUEST being used respectfully. 

    // Only send the mission item if it exists 
    if (sequence < vehicle.memory.mission_size)
    {
        mavlink_mission_item_int_t mission_item = vehicle.memory.mission[sequence]; 

        mission_item.target_system = system_id_gcs; 
        mission_item.target_component = component_id_gcs; 

        mavlink_msg_mission_item_int_encode_chan(
            system_id, 
            component_id, 
            channel, 
            &msg, 
            &mission_item); 
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


// MAVLink: MISSION_CURRENT send 
void VehicleTelemetry::MAVLinkMissionCurrentSend(Vehicle &vehicle)
{
    if ((mavlink.mission_current_msg_timing.enable && 
        (++mavlink.mission_current_msg_timing.count >= mavlink.mission_current_msg_timing.count_lim)) || 
        status.mission_current)
    {
        status.mission_current = FLAG_CLEAR; 

        mavlink_msg_mission_current_pack_chan(
            system_id, 
            component_id, 
            channel, 
            &msg, 
            vehicle.memory.mission_index,   // Sequence number 
            vehicle.memory.mission_size,    // Total number of mission items 
            MISSION_STATE_UNKNOWN,          // Mission state - not supported 
            0,                              // Mission mode - not supported 
            vehicle.memory.mission_id,      // Mission ID 
            0,                              // Fence ID - not supported 
            0);                             // Ralley point ID - not supported 
        MAVLinkMessageFormat(); 
    }
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

//==================================================


//==================================================
// Other 

// Clear current mission 
void VehicleTelemetry::ClearMission(
    Vehicle &vehicle, 
    uint8_t mission_type)
{
    // Clear the vehicle mission (but not the home position), generate a new mission ID 
    // and acknowledge that the mission has been cleared. 

    memset((void *)&vehicle.memory.mission[HOME_OFFSET], 
           RESET, 
           (MAX_MISSION_SIZE - HOME_OFFSET)*sizeof(vehicle.memory.mission[0])); 

    MAVLinkMissionAckSend(
        MAV_MISSION_ACCEPTED, 
        mission_type, 
        ++vehicle.memory.mission_id); 
}


// Enable the MISSION_CURRENT message 
void VehicleTelemetry::MAVLinkMissionCurrentEnable(void)
{
    mavlink.mission_current_msg_timing.enable = FLAG_SET; 
}


// Disable the MISSION_CURRENT message 
void VehicleTelemetry::MAVLinkMissionCurrentDisable(void)
{
    mavlink.mission_current_msg_timing.enable = FLAG_CLEAR; 
}


// Set flag to trigger a MISSION_ITEM_REACHED message send 
void VehicleTelemetry::MAVLinkMissionItemReachedSet(void)
{
    status.mission_item_reached = FLAG_SET; 
}

//==================================================

//=======================================================================================


//=======================================================================================
// MAVLink: Command protocol 

//==================================================
// Receive 

// MAVLink: COMMAND_LONG receive 
void VehicleTelemetry::MAVLinkCommandLongReceive(Vehicle &vehicle)
{
    mavlink_msg_command_long_decode(
        &msg, 
        &mavlink.command_long_msg_gcs); 
        
    if (VerifyVehicleIDs(mavlink.command_long_msg_gcs.target_system, 
                         mavlink.command_long_msg_gcs.target_component))
    {
        return; 
    }

    mavlink_cmd_msg_t cmd_msg = 
    {
        .param1 = mavlink.command_long_msg_gcs.param1, 
        .param2 = mavlink.command_long_msg_gcs.param2, 
        .param3 = mavlink.command_long_msg_gcs.param3, 
        .param4 = mavlink.command_long_msg_gcs.param4, 
        .param5 = mavlink.command_long_msg_gcs.param5, 
        .param6 = mavlink.command_long_msg_gcs.param6, 
        .param7 = mavlink.command_long_msg_gcs.param7, 
        .x = RESET, 
        .y = RESET, 
        .z = RESET, 
        .command = mavlink.command_long_msg_gcs.command, 
        .frame = RESET, 
        .confirmation = mavlink.command_long_msg_gcs.confirmation 
    }; 
    
    MAVLinkCommandDecode(vehicle, cmd_msg); 
}


// MAVLink: COMMAND_INT receive 
void VehicleTelemetry::MAVLinkCommandIntReceive(Vehicle &vehicle)
{
    mavlink_msg_command_int_decode(
        &msg, 
        &mavlink.command_int_msg_gcs); 
        
    if (VerifyVehicleIDs(mavlink.command_int_msg_gcs.target_system, 
                         mavlink.command_int_msg_gcs.target_component))
    {
        return; 
    }

    mavlink_cmd_msg_t cmd_msg = 
    {
        .param1 = mavlink.command_int_msg_gcs.param1, 
        .param2 = mavlink.command_int_msg_gcs.param2, 
        .param3 = mavlink.command_int_msg_gcs.param3, 
        .param4 = mavlink.command_int_msg_gcs.param4, 
        .param5 = RESET, 
        .param6 = RESET, 
        .param7 = RESET, 
        .x = mavlink.command_int_msg_gcs.x, 
        .y = mavlink.command_int_msg_gcs.y, 
        .z = mavlink.command_int_msg_gcs.z, 
        .command = mavlink.command_int_msg_gcs.command, 
        .frame = mavlink.command_int_msg_gcs.frame, 
        .confirmation = RESET 
    }; 
        
    MAVLinkCommandDecode(vehicle, cmd_msg); 
}


// MAVLink command decode 
void VehicleTelemetry::MAVLinkCommandDecode(
    Vehicle &vehicle, 
    mavlink_cmd_msg_t &cmd_msg)
{
    switch (cmd_msg.command)
    {
        case MAV_CMD_DO_SET_MODE: 
            MAVLinkCommandDoSetModeReceive(vehicle, cmd_msg); 
            break; 
        
        case MAV_CMD_DO_SET_HOME: 
            MAVLinkCommandDoSetHomeReceive(cmd_msg); 
            break; 

        case MAV_CMD_COMPONENT_ARM_DISARM: 
            break; 
        
        case MAV_CMD_REQUEST_MESSAGE: 
            MAVLinkCommandRequestMessageReceive(cmd_msg); 
            break; 
        
        default: 
            break; 
    }
}


// MAVLink command: DO_SET_MODE receive 
void VehicleTelemetry::MAVLinkCommandDoSetModeReceive(
    Vehicle &vehicle, 
    mavlink_cmd_msg_t &cmd_msg)
{
    // Ardupilot sets param1 to MAV_MODE_FLAG_CUSTOM_MODE_ENABLED so we look for that 
    // before attempting to update the vehicle state/mode. 
    if ((uint16_t)cmd_msg.param1 == MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)
    {
        vehicle.MainStateSelect((uint8_t)mavlink.command_long_msg_gcs.param2); 
        MAVLinkCommandACKSend(); 
    }
}


// MAVLink command: DO_SET_HOME receive 
void VehicleTelemetry::MAVLinkCommandDoSetHomeReceive(mavlink_cmd_msg_t &cmd_msg)
{
    // TODO is this the only point at which Mission Planner sets the home positon? 
}


// MAVLink command: REQUEST_MESSAGE 
void VehicleTelemetry::MAVLinkCommandRequestMessageReceive(mavlink_cmd_msg_t &cmd_msg)
{
    uint16_t msg_id = (uint16_t)cmd_msg.param1; 
    
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

//==================================================


//==================================================
// Send 

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

//==================================================

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

    if (VerifyVehicleIDs(mavlink.request_data_stream_msg_gcs.target_system, 
                         mavlink.request_data_stream_msg_gcs.target_component))
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
            // mavlink.rc_channels_scaled_msg_timing.enable = enable; 
            // mavlink.rc_channels_scaled_msg_timing.count_lim = timer_limit; 

            // mavlink.rc_channels_raw_msg_timing.enable = enable; 
            // mavlink.rc_channels_raw_msg_timing.count_lim = timer_limit; 

            // mavlink.servo_output_raw_msg_timing.enable = enable; 
            // mavlink.servo_output_raw_msg_timing.count_lim = timer_limit; 
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
            // mavlink.local_position_ned_msg_timing.enable = enable; 
            // mavlink.local_position_ned_msg_timing.count_lim = timer_limit; 

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


// MAVLink: HOME_POSITION send 

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


// // MAVLink: RC_CHANNELS_SCALED 
// void VehicleTelemetry::MAVLinkRCChannelScaledSendPeriodic(Vehicle &vehicle)
// {
//     if (mavlink.rc_channels_scaled_msg_timing.enable && 
//        (++mavlink.rc_channels_scaled_msg_timing.count >= 
//           mavlink.rc_channels_scaled_msg_timing.count_lim))
//     {
//         mavlink.rc_channels_scaled_msg_timing.count = RESET; 

//         mavlink_msg_rc_channels_scaled_pack_chan(
//             system_id, 
//             component_id, 
//             channel, 
//             &msg, 
//             vehicle.auxiliary.time_usec,   // Time since boot 
//             0,                             // Servo output port 
//             0, 0, 0, 0, 0, 0, 0, 0,        // RC channels 1-8 
//             0xFF);                         // Receive signal strength 
//         MAVLinkMessageFormat(); 
//     }
// }


// // MAVLink: RC_CHANNELS_RAW 
// void VehicleTelemetry::MAVLinkRCChannelRawSendPeriodic(Vehicle &vehicle)
// {
//     if (mavlink.rc_channels_raw_msg_timing.enable && 
//        (++mavlink.rc_channels_raw_msg_timing.count >= 
//           mavlink.rc_channels_raw_msg_timing.count_lim))
//     {
//         mavlink.rc_channels_raw_msg_timing.count = RESET; 

//         mavlink_msg_rc_channels_raw_pack_chan(
//             system_id, 
//             component_id, 
//             channel, 
//             &msg, 
//             vehicle.auxiliary.time_usec,   // Time since boot 
//             0,                             // Servo output port 
//             0, 0, 0, 0, 0, 0, 0, 0,        // RC channels 1-8 
//             0xFF);                         // Receive signal strength 
//         MAVLinkMessageFormat(); 
//     }
// }


// // MAVLink: SERVO_OUTPUT_RAW 
// void VehicleTelemetry::MAVLinkServoOutputRawSendPeriodic(Vehicle &vehicle)
// {
//     if (mavlink.servo_output_raw_msg_timing.enable && 
//        (++mavlink.servo_output_raw_msg_timing.count >= 
//           mavlink.servo_output_raw_msg_timing.count_lim))
//     {
//         mavlink.servo_output_raw_msg_timing.count = RESET; 

//         mavlink_msg_servo_output_raw_pack_chan(
//             system_id, 
//             component_id, 
//             channel, 
//             &msg, 
//             vehicle.auxiliary.time_usec,   // Time since boot 
//             0,                             // Servo output port 
//             0, 0, 0, 0, 0, 0, 0, 0,        // Servo output 1-8 
//             0, 0, 0, 0, 0, 0, 0, 0);       // Servo output 9-16 
//         MAVLinkMessageFormat(); 
//     }
// }


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


// // MAVLink: LOCAL_POSITION_NED 
// void VehicleTelemetry::MAVLinkLocalPositionNEDSendPeriodic(Vehicle &vehicle)
// {
//     if (mavlink.local_position_ned_msg_timing.enable && 
//        (++mavlink.local_position_ned_msg_timing.count >= 
//           mavlink.local_position_ned_msg_timing.count_lim))
//     {
//         mavlink.local_position_ned_msg_timing.count = RESET; 

//         mavlink_msg_local_position_ned_pack_chan(
//             system_id, 
//             component_id, 
//             channel, 
//             &msg, 
//             vehicle.auxiliary.time_usec,   // Time since boot 
//             0,                             // X position (m) 
//             0,                             // Y position (m) 
//             0,                             // Z position (m) 
//             0,                             // X speed (m/s) 
//             0,                             // Y speed (m/s) 
//             0);                            // Z speed (m/s) 
//         MAVLinkMessageFormat(); 
//     }
// }


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


//=======================================================================================
// Helper functions 

uint8_t VehicleTelemetry::VerifyVehicleIDs(
    uint8_t target_system, 
    uint8_t target_component)
{
    // This system is only concerned with messages meant for this system. If the taget 
    // system and component ID in the message does not match this system then abort 
    // message decoding. 
    if ((target_system != system_id) || (target_component != component_id))
    {
        return true; 
    }

    return false; 
}

//=======================================================================================
