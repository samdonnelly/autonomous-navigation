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
    mavlink_heartbeat_t heartbeat_msg_gcs;                       // HEARTBEAT 
    
    // Parameter protocol 
    mavlink_param_request_list_t param_request_list_msg_gcs;     // PARAM_REQUEST_LIST 
    
    // Mission protocol 
    mavlink_mission_count_t mission_count_msg_gcs;               // MISSION_COUNT 
    mavlink_mission_request_t mission_request_msg_gcs;           // MISSION_REQUEST 
    mavlink_mission_item_int_t mission_item_int_msg_gcs;         // MISSION_ITEM_INT 

    // mavlink_mission_current_t g; 
    // mavlink_mission_ack_t t; 

    // Command protocol 
    mavlink_command_long_t command_long_msg_gcs;                 // COMMAND_LONG 

    // Other messages 
    mavlink_request_data_stream_t request_data_stream_msg_gcs;   // REQUEST_DATA_STREAM 
    
    //==================================================

    //==================================================
    // Outgoing messages 

    // Heartbeat protocol 
    mavlink_heartbeat_t heartbeat_msg;                           // HEARTBEAT 
    
    // Periodic outgoing message timing info 
    MsgTiming heartbeat_msg_timing;                              // HEARTBEAT 
    MsgTiming raw_imu_msg_timing;                                // RAW_IMU 
    MsgTiming gps_raw_int_msg_timing;                            // GPS_RAW_INT 
    MsgTiming rc_channels_scaled_msg_timing;                     // RC_CHANNELS_SCALED 
    MsgTiming rc_channels_raw_msg_timing;                        // RC_CHANNELS_RAW 
    MsgTiming servo_output_raw_msg_timing;                       // SERVO_OUTPUT_RAW 
    MsgTiming attitude_msg_timing;                               // ATTITUDE 
    MsgTiming position_target_global_int_msg_timing;             // POSITION_TARGET_GLOBAL_INT 
    MsgTiming nav_controller_output_msg_timing;                  // NAV_CONTROLLER_OUTPUT 
    MsgTiming local_position_ned_msg_timing;                     // LOCAL_POSITION_NED 
    MsgTiming global_pos_int_msg_timing;                         // GLOBAL_POSITION_INT 
    MsgTiming param_value_msg_timing;                            // PARAM_VALUE 
    
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
    mavlink_message_t msg; 
    mavlink_status_t status; 
    uint16_t data_in_index; 
    uint16_t data_in_size; 
    uint8_t data_in_buff[VS_TELEMETRY_BUFF]; 
    uint16_t data_out_size; 
    uint8_t data_out_buff[VS_TELEMETRY_BUFF]; 

    // MAVLink messages 
    VehicleMAVLink mavlink; 

    mavlink_mission_item_int_t mission[5]; 

    // Mission protocol 
    uint16_t mission_item_count; 

    // Status timers 
    uint8_t heartbeat_status_timer; 
    uint8_t mission_upload_timer; 

    // Status flags 
    uint8_t connected : 1; 

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
    void MAVLinkParamValueSendPeriodic(Vehicle &vehicle); 

    // MAVLink: Mission protocol 
    void MAVLinkMissionCountReceive(void); 
    void MAVLinkMissionRequestReceive(Vehicle &vehicle); 
    void MAVLinkMissionRequestIntSend(void); 
    void MAVLinkMissionItemIntReceive(void); 
    void MAVLinkMissionAck(MAV_MISSION_RESULT result); 

    // MAVLink: Command protocol 
    void MAVLinkCommandLongReceive(Vehicle &vehicle); 
    void MAVLinkCommandLongDecode(Vehicle &vehicle); 
    void MAVLinkCommandDoSetModeReceive(Vehicle &vehicle); 
    void MAVLinkCommandRequestMessageReceive(void); 
    void MAVLinkCommandACKSend(void); 

    // MAVLink: other commands 
    void MAVLinkRequestDataStreamReceive(void); 
    void MAVLinkAutopilotVersionSend(void); 

    // MAVLink: Requestable outgoing messages 
    void MAVLinkRawIMUSendPeriodic(Vehicle &vehicle); 
    void MAVLinkGPSRawIntSendPeriodic(Vehicle &vehicle); 
    void MAVLinkRCChannelScaledSendPeriodic(Vehicle &vehicle); 
    void MAVLinkRCChannelRawSendPeriodic(Vehicle &vehicle); 
    void MAVLinkServoOutputRawSendPeriodic(Vehicle &vehicle); 
    void MAVLinkAttitudeSendPeriodic(Vehicle &vehicle); 
    void MAVLinkPositionTargetGlobalIntSendPeriodic(Vehicle &vehicle); 
    void MAVLinkNavControllerSendPeriodic(Vehicle &vehicle); 
    void MAVLinkLocalPositionNEDSendPeriodic(Vehicle &vehicle); 
    void MAVLinkGlobalPositionIntSendPeriodic(Vehicle &vehicle); 

public:   // Public methods 

    VehicleTelemetry(); 

    // MAVLink message handling 
    void MAVLinkMessageDecode(Vehicle &vehicle); 
    void MAVLinkMessageEncode(Vehicle &vehicle); 

    // Setters 
    void MAVLinkHeartbeatSetMode(uint8_t mode); 
};

//=======================================================================================

#endif   // _TELEMETRY_H_ 
