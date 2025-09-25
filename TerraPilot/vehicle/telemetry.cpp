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
static constexpr uint16_t telemetry_send_period = s_to_ms * PERIODIC_TIMER_250MS_PERIOD / configTICK_RATE_HZ;

// Message send frequencies (Hz) 
static constexpr uint8_t 
heartbeat_send_freq = 1,               // HEARTBEAT 
parameter_value_send_freq = 4,         // PARAMETER_VALUE 
mission_current_send_freq = 1,         // MISSION_CURRENT 
raw_imu_send_freq = 2,                 // RAW_IMU 
gps_raw_int_send_freq = 2,             // GPS_RAW_INT 
attitude_send_freq = 4,                // ATTITUDE 
pos_target_global_int_send_freq = 2,   // POSITION_TARGET_GLOBAL_INT 
nav_controller_out_send_freq = 2,      // NAV_CONTROLLER_OUTPUT 
global_pos_int_send_freq = 2;          // GLOBAL_POS_INT 

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
      parameter_index(RESET), 
      mission_item_index(RESET), 
      mission_resend_counter(RESET) 
{
    mavlink.heartbeat_msg = 
    {
        .custom_mode = RESET, 
        .type = vehicle_type, 
        .autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA,   // Allows for using ArduPilot modes 
        .base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | 
                     MAV_MODE_FLAG_MANUAL_INPUT_ENABLED | 
                     MAV_MODE_FLAG_GUIDED_ENABLED | 
                     MAV_MODE_FLAG_SAFETY_ARMED, 
        .system_status = MAV_STATE_ACTIVE, 
        .mavlink_version = RESET   // Gets overwritten by MAVLink 
    };

    const uint8_t 
    freq_to_count = s_to_ms / telemetry_send_period,
    heartbeat_lim             = freq_to_count / heartbeat_send_freq,
    parameter_value_lim       = freq_to_count / parameter_value_send_freq,
    mission_current_lim       = freq_to_count / mission_current_send_freq,
    raw_imu_lim               = freq_to_count / raw_imu_send_freq,
    gps_raw_int_lim           = freq_to_count / gps_raw_int_send_freq,
    attitude_lim              = freq_to_count / attitude_send_freq,
    pos_target_global_int_lim = freq_to_count / pos_target_global_int_send_freq,
    nav_controller_lim        = freq_to_count / nav_controller_out_send_freq,
    global_pos_int_lim        = freq_to_count / global_pos_int_send_freq;

    // Outgoing message timing 
    mavlink.heartbeat_msg_timing                  = { RESET, heartbeat_lim,             FLAG_SET };
    mavlink.param_value_msg_timing                = { RESET, parameter_value_lim,       FLAG_CLEAR };
    mavlink.mission_current_msg_timing            = { RESET, mission_current_lim,       FLAG_SET };
    mavlink.raw_imu_msg_timing                    = { RESET, raw_imu_lim,               FLAG_SET };
    mavlink.gps_raw_int_msg_timing                = { RESET, gps_raw_int_lim,           FLAG_SET };
    // mavlink.rc_channels_scaled_msg_timing         = { RESET, RESET, FLAG_CLEAR };
    // mavlink.rc_channels_raw_msg_timing            = { RESET, RESET, FLAG_CLEAR };
    // mavlink.servo_output_raw_msg_timing           = { RESET, RESET, FLAG_CLEAR };
    mavlink.attitude_msg_timing                   = { RESET, attitude_lim,              FLAG_SET };
    mavlink.position_target_global_int_msg_timing = { RESET, pos_target_global_int_lim, FLAG_CLEAR };
    mavlink.nav_controller_output_msg_timing      = { RESET, nav_controller_lim,        FLAG_CLEAR };
    // mavlink.local_position_ned_msg_timing         = { RESET, RESET, FLAG_CLEAR };
    mavlink.global_pos_int_msg_timing             = { RESET, global_pos_int_lim,        FLAG_SET };

    memset((void *)data_in_buff, RESET, sizeof(data_in_buff)); 
    memset((void *)data_out_buff, RESET, sizeof(data_out_buff)); 

    timers.heartbeat = RESET; 
    timers.mission_upload = RESET; 

    status.flags = RESET; 
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
            MAVLinkMissionRequestSend(); 
        }
        else 
        {
            status.mission_upload = FLAG_CLEAR; 
            MAVLinkMissionAckSend(
                MAV_MISSION_OPERATION_CANCELLED, 
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

        case MAVLINK_MSG_ID_PARAM_SET: 
            MAVLinkParamSetReceive(vehicle); 
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

        case MAVLINK_MSG_ID_REQUEST_DATA_STREAM: 
            MAVLinkRequestDataStreamReceive(); 
            break; 

        case MAVLINK_MSG_ID_COMMAND_INT: 
            MAVLinkCommandIntReceive(vehicle); 
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
    parameter_index = RESET; 
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

    // MAVLink protocol gives the GCS the option on how to fetch the parameter. If the 
    // provided index is -1 then the parameter ID/name (string) is used, otherwise the 
    // provided index is used. The index is checked in the PARAM_VALUE send function. 
    parameter_index = (mavlink.param_request_read_msg_gcs.param_index < 0) ? 
                      vehicle.memory.ParameterLookUp(mavlink.param_request_read_msg_gcs.param_id) : 
                      (uint8_t)mavlink.param_request_read_msg_gcs.param_index; 

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

    parameter_index = vehicle.memory.ParameterSet(vehicle, 
                                                  mavlink.param_set_msg_gcs.param_id, 
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
        
        if (vehicle.memory.ParameterIndexCheck(parameter_index))
        {
            mavlink_msg_param_value_pack_chan(
                system_id, 
                component_id, 
                channel, 
                &msg, 
                parameters[parameter_index].name, 
                *parameters[parameter_index].value, 
                parameters[parameter_index].type, 
                parameters.size(), 
                parameter_index); 
            MAVLinkMessageFormat(); 
            
            parameter_index++; 
        }
        else 
        {
            mavlink.param_value_msg_timing.enable = FLAG_CLEAR; 

            // If the index is not valid then no PARAM_VALUE message gets sent. If a 
            // PARAM_REQUEST_READ or PARAM_SET request comes through with an invalid 
            // ID or index then a STATUS_TEXT message should be emitted here, however 
            // it's not necessary as the GCS will simply time out when it doesn't get 
            // a reponse soon enough. 
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

    // If the provided count is 0 then that's the same as clearing the vehicles currently 
    // stored mission. If the count is larger than the available mission item storage 
    // space then the upload must be rejected. Otherwise the mission upload sequence is 
    // started. The count is the number of mission items not including the home location 
    // (index 0). 
    if (mavlink.mission_count_msg_gcs.count == HOME_INDEX)
    {
        // Clear the vehicle mission (but not the home position), update the mission size, 
        // generate a new mission ID and acknowledge that the mission has been cleared. 

        vehicle.memory.MissionClear(); 
        MAVLinkMissionAckSend(MAV_MISSION_ACCEPTED, vehicle.memory.MissionIDUpdate()); 
    }
    else if (mavlink.mission_count_msg_gcs.count > MAX_MISSION_SIZE)
    {
        // The home location is stored at index 0 so the max number of mission items has 
        // to be less than the max mission storage size. 
        MAVLinkMissionAckSend(MAV_MISSION_NO_SPACE, vehicle.memory.MissionIDGet()); 
    }
    else 
    {
        status.mission_upload = FLAG_SET; 
        mission_resend_counter = RESET; 
        mission_item_index = RESET; 
        MAVLinkMissionRequestSend(); 
    }
}


// MAVLink: MISSION_REQUEST_INT receive 
void VehicleTelemetry::MAVLinkMissionRequestIntReceive(Vehicle &vehicle)
{
    // Mission Planner uses MISSION_REQUEST_INT when downloading a mission from the 
    // vehicle instead of MISSION_REQUEST like it uses for getting the home location. 

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
            vehicle.memory.MissionItemSet(mavlink.mission_item_int_msg_gcs); 

            // Check if the received MISSION_ITEM_INT sequence number matches the total 
            // number of items expected in the mission upload. If so then the mission 
            // upload sequence is done. Otherwise proceed to request the next item. 
            if (mavlink.mission_item_int_msg_gcs.seq >= (mavlink.mission_count_msg_gcs.count - 1))
            {
                // Mission upload complete. Update the mission size and type, reset the 
                // mission target, acknowledge a successful mission upload and end the 
                // sequence. 
                status.mission_upload = FLAG_CLEAR; 
                vehicle.memory.MissionSizeSet(mavlink.mission_count_msg_gcs.count); 
                vehicle.memory.MissionTypeSet(mavlink.mission_count_msg_gcs.mission_type); 
                vehicle.memory.MissionTargetSet(HOME_OFFSET); 

                MAVLinkMissionAckSend(MAV_MISSION_ACCEPTED, vehicle.memory.MissionIDUpdate()); 
            }
            else 
            {
                mission_item_index++; 
                mission_resend_counter = RESET; 
                MAVLinkMissionRequestSend(); 
            }
        }
        else 
        {
            MAVLinkMissionAckSend(
                MAV_MISSION_INVALID_SEQUENCE, 
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


/**
 * @brief MAVLink: MISSION_SET_CURRENT receive 
 * 
 * @details Mission Planner seems to use this mission command as opposed to 
 *          MAV_CMD_DO_SET_MISSION_CURRENT to set the current mission target. 
 * 
 * @param vehicle : vehicle object 
 */
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
    vehicle.memory.MissionTargetSet(mavlink.mission_set_current_msg_gcs.seq); 
    status.mission_current = FLAG_SET; 
    MAVLinkMissionCurrentSend(vehicle); 
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
        vehicle.memory.MissionSizeGet(), 
        vehicle.memory.MissionTypeGet(), 
        vehicle.memory.MissionIDGet()); 
    MAVLinkMessageFormat(); 
}


// MAVLink: MISSION_REQUEST send 
void VehicleTelemetry::MAVLinkMissionRequestSend(void)
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
        mavlink.mission_count_msg_gcs.mission_type); 
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

    MissionItem mission_item = vehicle.memory.MissionItemGet(sequence); 

    if (mission_item.seq == sequence)
    {
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
        mavlink.mission_count_msg_gcs.mission_type, 
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
            vehicle.memory.MissionTargetGet(),   // Current sequence/index/target 
            vehicle.memory.MissionSizeGet(),     // Total number of mission items 
            MISSION_STATE_UNKNOWN,               // Mission state - not supported 
            0,                                   // Mission mode - not supported 
            vehicle.memory.MissionIDGet(),       // Mission ID 
            0,                                   // Fence ID - not supported 
            0);                                  // Ralley point ID - not supported 
        MAVLinkMessageFormat(); 
    }
}


// MAVLink: MISSION_ITEM_REACHED send 
void VehicleTelemetry::MAVLinkMissionItemReachedSend(Vehicle &vehicle)
{
    if (status.mission_item_reached)
    {
        status.mission_item_reached = FLAG_CLEAR; 

        mavlink_msg_mission_item_reached_pack_chan(
            system_id, 
            component_id, 
            channel, 
            &msg, 
            vehicle.memory.MissionTargetGet()); 
        MAVLinkMessageFormat(); 
    }
}

//==================================================


//==================================================
// Other 

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

    switch (mavlink.command_int_msg_gcs.command)
    {
        case MAV_CMD_DO_SET_HOME: 
            MAVLinkCommandDoSetHomeReceive(vehicle); 
            break; 

        case MAV_CMD_DO_SET_MISSION_CURRENT: 
            MAVLinkCommandDoSetMissionCurrentReceive(vehicle); 
            break; 
        
        default: 
            MAVLinkCommandACKSend(mavlink.command_int_msg_gcs.command, MAV_RESULT_FAILED); 
            break; 
    }
}


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

    switch (mavlink.command_long_msg_gcs.command)
    {
        case MAV_CMD_DO_SET_MODE: 
            MAVLinkCommandDoSetModeReceive(vehicle); 
            break; 
        
        case MAV_CMD_REQUEST_MESSAGE: 
            MAVLinkCommandRequestMessageReceive(); 
            break; 
        
        default: 
            MAVLinkCommandACKSend(mavlink.command_long_msg_gcs.command, MAV_RESULT_FAILED); 
            break; 
    }
}


// MAVLink: MAV_CMD_DO_SET_MODE receive 
void VehicleTelemetry::MAVLinkCommandDoSetModeReceive(Vehicle &vehicle)
{
    MAVLinkCommandACKSend(MAV_CMD_DO_SET_MODE, MAV_RESULT_ACCEPTED); 

    // Ardupilot sets param1 to MAV_MODE_FLAG_CUSTOM_MODE_ENABLED so we look for that 
    // before attempting to update the vehicle state/mode. 
    if ((uint16_t)mavlink.command_long_msg_gcs.param1 == MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)
    {
        vehicle.MainStateSelect((uint8_t)mavlink.command_long_msg_gcs.param2); 
    }
}


// MAVLink: MAV_CMD_DO_SET_HOME receive 
void VehicleTelemetry::MAVLinkCommandDoSetHomeReceive(Vehicle &vehicle)
{
    MAVLinkCommandACKSend(MAV_CMD_DO_SET_HOME, MAV_RESULT_ACCEPTED); 

    VehicleNavigation::Location home; 

    // If param1 is set that means the vehicles current location should be used to set 
    // the home position. A position lock is not checked for because Mission Planner 
    // won't send this command and specify that the vehicle use its current location as 
    // the home position unless the vehicle is already reporting a valid GPS position. 
    if (mavlink.command_int_msg_gcs.param1)
    {
        home = vehicle.navigation.LocationGet(); 
    }
    else 
    {
        home.latI = mavlink.command_int_msg_gcs.x; 
        home.lonI = mavlink.command_int_msg_gcs.y; 
        home.alt = mavlink.command_int_msg_gcs.z; 
    }

    vehicle.memory.MissionHomeLocationSet(home.latI, home.lonI, home.alt); 

    // Respond with the new home position so Mission Planner knows it worked. 
    MAVLinkHomePositionSend(vehicle); 
}


// MAVLink: MAV_CMD_DO_SET_MISSION_CURRENT receive 
void VehicleTelemetry::MAVLinkCommandDoSetMissionCurrentReceive(Vehicle &vehicle)
{
    MAVLinkCommandACKSend(MAV_CMD_DO_SET_MISSION_CURRENT, MAV_RESULT_ACCEPTED); 

    // Note that "Reset Mission" is not supported at this time. 
    if (mavlink.command_int_msg_gcs.param1 >= 0)
    {
        vehicle.memory.MissionTargetSet((uint16_t)mavlink.command_int_msg_gcs.param1); 
    }

    status.mission_current = FLAG_SET; 
    MAVLinkMissionCurrentSend(vehicle); 
}


// MAVLink: MAV_CMD_REQUEST_MESSAGE receive 
void VehicleTelemetry::MAVLinkCommandRequestMessageReceive(void)
{
    MAVLinkCommandACKSend(MAV_CMD_REQUEST_MESSAGE, MAV_RESULT_ACCEPTED); 

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

//==================================================


//==================================================
// Send 

// MAVLink: COMMAND_ACK send 
void VehicleTelemetry::MAVLinkCommandACKSend(
    uint16_t command, 
    MAV_RESULT result)
{
    mavlink_msg_command_ack_pack_chan(
        system_id, 
        component_id, 
        channel, 
        &msg, 
        command, 
        result, 
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

// MAVLink: REQUEST_DATA_STREAM receive 
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
    // uint8_t timer_limit = (uint8_t)(s_to_ms / 
    //     (mavlink.request_data_stream_msg_gcs.req_message_rate * telemetry_send_period)); 
    // uint8_t enable = mavlink.request_data_stream_msg_gcs.start_stop; 

    // Enable/disable the requested message and assign the message timer counter limit 
    // so it gets sent to the GCS at the requested rate. 
    switch (mavlink.request_data_stream_msg_gcs.req_stream_id)
    {
        case MAV_DATA_STREAM_ALL: 
            break; 
        
        case MAV_DATA_STREAM_RAW_SENSORS: 
            // mavlink.raw_imu_msg_timing.enable = enable; 
            // mavlink.raw_imu_msg_timing.count_lim = timer_limit; 

            // mavlink.gps_raw_int_msg_timing.enable = enable; 
            // mavlink.gps_raw_int_msg_timing.count_lim = timer_limit; 
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
            // mavlink.attitude_msg_timing.enable = enable; 
            // mavlink.attitude_msg_timing.count_lim = timer_limit; 

            // mavlink.position_target_global_int_msg_timing.enable = enable; 
            // mavlink.position_target_global_int_msg_timing.count_lim = timer_limit; 

            // mavlink.nav_controller_output_msg_timing.enable = enable; 
            // mavlink.nav_controller_output_msg_timing.count_lim = timer_limit; 
            break; 

        case MAV_DATA_STREAM_POSITION: 
            // mavlink.local_position_ned_msg_timing.enable = enable; 
            // mavlink.local_position_ned_msg_timing.count_lim = timer_limit; 

            // mavlink.global_pos_int_msg_timing.enable = enable; 
            // mavlink.global_pos_int_msg_timing.count_lim = timer_limit; 
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


// MAVLink: AUTOPILOT_VERSION receive 
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
void VehicleTelemetry::MAVLinkHomePositionSend(Vehicle &vehicle)
{
    mavlink_home_position_t home_position; 
    MissionItem home = vehicle.memory.MissionItemGet(HOME_INDEX); 

    // The other fields of HOME_POSITION are not currently supported. 
    home_position.latitude = home.x; 
    home_position.longitude = home.y; 
    home_position.altitude = (int32_t)(home.z * coordinate_scalar); 

    mavlink_msg_home_position_encode_chan(
        system_id, 
        component_id, 
        channel, 
        &msg, 
        &home_position); 
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

        VehicleNavigation::Vector<float> 
        accel = vehicle.navigation.AccelGet(), 
        gyro = vehicle.navigation.GyroGet(), 
        mag = vehicle.navigation.MagGet(); 

        mavlink_msg_raw_imu_pack_chan(
            system_id, 
            component_id, 
            channel, 
            &msg, 
            vehicle.auxiliary.time_usec,      // Time since boot 
            static_cast<int16_t>(accel.x),    // X accelerometer 
            static_cast<int16_t>(accel.y),    // Y accelerometer 
            static_cast<int16_t>(accel.z),    // Z accelerometer 
            static_cast<int16_t>(gyro.x),     // X gyroscope 
            static_cast<int16_t>(gyro.y),     // Y gyroscope 
            static_cast<int16_t>(gyro.z),     // Z gyroscope 
            static_cast<int16_t>(mag.x),      // X magnetometer 
            static_cast<int16_t>(mag.y),      // Y magnetometer 
            static_cast<int16_t>(mag.z),      // Z magnetometer 
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

        VehicleNavigation::Location location = vehicle.navigation.LocationGet(); 

        mavlink_msg_gps_raw_int_pack_chan(
            system_id, 
            component_id, 
            channel, 
            &msg, 
            vehicle.auxiliary.time_usec,        // Timestamp 
            vehicle.navigation.fix_type,        // GPS fix type 
            location.latI,                      // Latitude (degE7) (WGS84) 
            location.lonI,                      // Longitude (degE7) (WGS84) 
            // WGS84 altitude. MSL altitude not supported yet. 
            location.altI,                      // Altitude (mm) (MSL) 
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

        VehicleNavigation::Vector<float> orient = vehicle.navigation.OrientationGet(); 
        VehicleNavigation::Vector<float> gyro = vehicle.navigation.GyroGet(); 

        mavlink_msg_attitude_pack_chan(
            system_id, 
            component_id, 
            channel, 
            &msg, 
            vehicle.auxiliary.time_usec,   // Time since boot 
            orient.x,                      // Roll angle (rad) 
            orient.y,                      // Pitch angle (rad) 
            orient.z,                      // Yaw angle (rad) 
            gyro.x,                        // Roll angular speed (rad/s) 
            gyro.y,                        // Pitch angular speed (rad/s) 
            gyro.z);                       // Yaw angular speed (rad/s) 
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

        VehicleNavigation::Location location = vehicle.navigation.LocationGet(); 

        mavlink_msg_position_target_global_int_pack_chan(
            system_id, 
            component_id, 
            channel, 
            &msg, 
            vehicle.auxiliary.time_usec,             // Time since boot 
            vehicle.navigation.coordinate_frame,     // Coordinate frame (MAV_FRAME) 
            vehicle.navigation.position_type_mask,   // Ignored dimensions (POSITION_TARGET_TYPEMASK) 
            location.latI,                           // Latitude 
            location.lonI,                           // Longitude 
            location.alt,                            // Altitude 
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

        VehicleNavigation::Vector<float> orient = vehicle.navigation.OrientationGet();
        float heading = vehicle.navigation.HeadingGet();
        float waypoint_heading = vehicle.navigation.HeadingTargetGet();
        float waypoint_distance = vehicle.navigation.WaypointDistanceGet();

        mavlink_msg_nav_controller_output_pack_chan(
            system_id, 
            component_id, 
            channel, 
            &msg, 
            orient.x*rad_to_deg,                        // Current desired roll 
            orient.y*rad_to_deg,                        // Current desired pitch 
            static_cast<int16_t>(heading),              // Current desired heading 
            static_cast<int16_t>(waypoint_heading),     // Bearing to current waypoint/target 
            static_cast<uint16_t>(waypoint_distance),   // Distance to active waypoint 
            0,                                          // Current altitude error 
            0,                                          // Current airspeed error 
            0);                                         // Current crosstrack error on x-y plane 
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

        VehicleNavigation::Location location = vehicle.navigation.LocationGet(); 
        float heading = vehicle.navigation.HeadingGet();

        mavlink_msg_global_position_int_pack_chan(
            system_id, 
            component_id, 
            channel, 
            &msg, 
            vehicle.auxiliary.time_usec,               // Time since boot 
            location.latI,                             // Latitude (degE7) (WGS84) 
            location.lonI,                             // Longitude (degE7) (WGS84) 
            // WGS84 altitude. MSL altitude not supported yet. 
            location.altI,                             // Altitude (mm) (MSL) 
            location.altI,                             // Altitude above home (mm) (WGS84) 
            0,                                         // X velocity 
            0,                                         // Y velocity 
            0,                                         // Z velocity 
            static_cast<uint16_t>(heading));           // Heading (yaw angle) (degrees) 
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
