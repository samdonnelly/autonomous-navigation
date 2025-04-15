/**
 * @file telemetry.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Vehicle telemetry interface 
 * 
 * @version 0.1
 * @date 2025-02-28
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef _TELEMETRY_H_ 
#define _TELEMETRY_H_ 

//=======================================================================================
// Includes 

#include "includes.h" 

//=======================================================================================


//=======================================================================================
// Classes 

class Vehicle; 


class VehicleMAVLink 
{
    // MAVLink messages and properites common to all vehicles. 
    
public:   // Public members 

    struct MsgTiming 
    {
        uint8_t count; 
        uint8_t count_lim; 
        uint8_t enable : 1; 
    }; 

    //==================================================
    // Incoming messages from the GCS 

    // Heartbeat protocol 
    mavlink_heartbeat_t heartbeat_msg_gcs;                         // HEARTBEAT 
    
    // Parameter protocol 
    mavlink_param_request_list_t param_request_list_msg_gcs;       // PARAM_REQUEST_LIST 
    mavlink_param_request_read_t param_request_read_msg_gcs;       // PARAM_REQUEST_READ 
    mavlink_param_set_t param_set_msg_gcs;                         // PARAM_SET 
    
    // Mission protocol 
    mavlink_mission_request_list_t mission_request_list_msg_gcs;   // MISSION_REQUEST_LIST 
    mavlink_mission_count_t mission_count_msg_gcs;                 // MISSION_COUNT 
    mavlink_mission_request_int_t mission_request_int_msg_gcs;     // MISSION_REQUEST_INT 
    mavlink_mission_request_t mission_request_msg_gcs;             // MISSION_REQUEST 
    mavlink_mission_item_int_t mission_item_int_msg_gcs;           // MISSION_ITEM_INT 
    mavlink_mission_ack_t mission_ack_msg_gcs;                     // MISSION_ACK 
    mavlink_mission_set_current_t mission_set_current_msg_gcs;     // MISSION_SET_CURRENT 
    mavlink_mission_clear_all_t mission_clear_all_msg_gcs;         // MISSION_CLEAR_ALL 

    // Command protocol 
    mavlink_command_long_t command_long_msg_gcs;                   // COMMAND_LONG 
    mavlink_command_int_t command_int_msg_gcs;                     // COMMAND_INT 

    // Other messages 
    mavlink_request_data_stream_t request_data_stream_msg_gcs;     // REQUEST_DATA_STREAM 
    
    //==================================================

    //==================================================
    // Outgoing messages 

    // Heartbeat protocol 
    mavlink_heartbeat_t heartbeat_msg;                             // HEARTBEAT 
    
    //==================================================
    
    //==================================================
    // Outgoing message timing 
    
    // Heartbeat protocol 
    MsgTiming heartbeat_msg_timing;                                // HEARTBEAT 

    // Parameter protocol 
    MsgTiming param_value_msg_timing;                              // PARAM_VALUE 

    // Mission protocol 
    MsgTiming mission_current_msg_timing;                          // MISSION_CURRENT 

    // Periodic requestable messages 
    MsgTiming raw_imu_msg_timing;                                  // RAW_IMU 
    MsgTiming gps_raw_int_msg_timing;                              // GPS_RAW_INT 
    // MsgTiming rc_channels_scaled_msg_timing;                       // RC_CHANNELS_SCALED 
    // MsgTiming rc_channels_raw_msg_timing;                          // RC_CHANNELS_RAW 
    // MsgTiming servo_output_raw_msg_timing;                         // SERVO_OUTPUT_RAW 
    MsgTiming attitude_msg_timing;                                 // ATTITUDE 
    MsgTiming position_target_global_int_msg_timing;               // POSITION_TARGET_GLOBAL_INT 
    MsgTiming nav_controller_output_msg_timing;                    // NAV_CONTROLLER_OUTPUT 
    // MsgTiming local_position_ned_msg_timing;                       // LOCAL_POSITION_NED 
    MsgTiming global_pos_int_msg_timing;                           // GLOBAL_POSITION_INT 

    //==================================================
};


class VehicleTelemetry 
{
private:   // Private members 

    // MAVLink identification 
    int channel; 
    uint8_t system_id; 
    uint8_t component_id; 
    uint8_t system_id_gcs; 
    uint8_t component_id_gcs; 

    // MAVLink packet handling 
    VehicleMAVLink mavlink; 
    mavlink_message_t msg; 
    mavlink_status_t msg_status; 
    uint16_t data_in_index; 
    uint16_t data_in_size; 
    uint8_t data_in_buff[VS_TELEMETRY_BUFF]; 
    uint16_t data_out_size; 
    uint8_t data_out_buff[VS_TELEMETRY_BUFF]; 

    // Parameter protocol 
    uint8_t parameter_index; 

    // Mission protocol 
    uint16_t mission_item_index; 
    uint8_t mission_resend_counter; 

    // Status timers 
    struct Timers 
    {
        uint8_t heartbeat; 
        uint8_t mission_upload; 
    }
    timers; 

    // Status flags 
    union Status 
    {
        struct 
        {
            uint32_t heartbeat            : 1; 
            uint32_t param_read           : 1; 
            uint32_t mission_upload       : 1; 
            uint32_t mission_current      : 1; 
            uint32_t mission_item_reached : 1; 
        }; 
        uint32_t flags; 
    }
    status; 

private:   // Private methods 

    // MAVLink message decode 
    void MsgTimerChecks(void); 
    void MAVLinkPayloadDecode(Vehicle &vehicle); 

    // MAVLink message encode 
    void MAVLinkMessageFormat(void); 
    void MAVLinkMessageSend(Vehicle &vehicle); 

    // MAVLink: Heartbeat protocol 
    void MAVLinkHeartbeatReceive(void); 
    void MAVLinkHeartbeatSend(void); 

    // MAVLink: Parameter protocol 
    void MAVLinkParamRequestListReceive(Vehicle &vehicle); 
    void MAVLinkParamRequestReadReceive(Vehicle &vehicle); 
    void MAVLinkParamSetReceive(Vehicle &vehicle); 
    void MAVLinkParamValueSend(Vehicle &vehicle); 

    // MAVLink: Mission protocol 
    void MAVLinkMissionRequestListReceive(Vehicle &vehicle); 
    void MAVLinkMissionCountReceive(Vehicle &vehicle); 
    void MAVLinkMissionRequestIntReceive(Vehicle &vehicle); 
    void MAVLinkMissionRequestReceive(Vehicle &vehicle); 
    void MAVLinkMissionItemIntReceive(Vehicle &vehicle); 
    void MAVLinkMissionAckReceive(void); 
    void MAVLinkMissionSetCurrentReceive(Vehicle &vehicle); 
    void MAVLinkMissionCountSend(Vehicle &vehicle); 
    void MAVLinkMissionRequestSend(void); 
    void MAVLinkMissionItemIntSend(Vehicle &vehicle, uint16_t sequence); 
    void MAVLinkMissionAckSend(MAV_MISSION_RESULT result, uint32_t opaque_id); 
    void MAVLinkMissionCurrentSend(Vehicle &vehicle); 
    void MAVLinkMissionItemReachedSend(Vehicle &vehicle); 

    // MAVLink: Command protocol 
    void MAVLinkCommandLongReceive(Vehicle &vehicle); 
    void MAVLinkCommandIntReceive(Vehicle &vehicle); 
    void MAVLinkCommandDoSetModeReceive(Vehicle &vehicle); 
    void MAVLinkCommandDoSetHomeReceive(Vehicle &vehicle); 
    void MAVLinkCommandDoSetMissionCurrentReceive(Vehicle &vehicle); 
    void MAVLinkCommandRequestMessageReceive(void); 
    void MAVLinkCommandACKSend(uint16_t command, MAV_RESULT result); 

    // MAVLink: other messages 
    void MAVLinkRequestDataStreamReceive(void); 
    void MAVLinkAutopilotVersionSend(void); 
    void MAVLinkHomePositionSend(Vehicle &vehicle); 

    // MAVLink: Requestable outgoing messages 
    void MAVLinkRawIMUSendPeriodic(Vehicle &vehicle); 
    void MAVLinkGPSRawIntSendPeriodic(Vehicle &vehicle); 
    // void MAVLinkRCChannelScaledSendPeriodic(Vehicle &vehicle); 
    // void MAVLinkRCChannelRawSendPeriodic(Vehicle &vehicle); 
    // void MAVLinkServoOutputRawSendPeriodic(Vehicle &vehicle); 
    void MAVLinkAttitudeSendPeriodic(Vehicle &vehicle); 
    void MAVLinkPositionTargetGlobalIntSendPeriodic(Vehicle &vehicle); 
    void MAVLinkNavControllerSendPeriodic(Vehicle &vehicle); 
    // void MAVLinkLocalPositionNEDSendPeriodic(Vehicle &vehicle); 
    void MAVLinkGlobalPositionIntSendPeriodic(Vehicle &vehicle); 

    // Helper functions 
    uint8_t VerifyVehicleIDs(uint8_t target_system, uint8_t target_component); 

public:   // Public methods 

    // Constructor 
    VehicleTelemetry(uint8_t vehicle_type); 

    // MAVLink message handling 
    void MessageDecode(Vehicle &vehicle); 
    void MessageEncode(Vehicle &vehicle); 

    // Setters 
    void MAVLinkHeartbeatSetMode(uint8_t mode); 
    void MAVLinkMissionCurrentEnable(void); 
    void MAVLinkMissionCurrentDisable(void); 
    void MAVLinkMissionItemReachedSet(void); 

    // Getters 
    uint8_t MAVLinkHeartbeatGetMode(void); 
};

//=======================================================================================

#endif   // _TELEMETRY_H_ 
