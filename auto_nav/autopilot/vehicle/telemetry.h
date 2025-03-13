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

#include "vehicle.h" 

extern "C"
{
    // For C headers without C++ guards 
    #include "common/mavlink.h" 
}

//=======================================================================================


//=======================================================================================
// Classes 

class VehicleTelemetry 
{
private:   // Private members 

    // MAVLink identification 
    int channel; 
    uint8_t system_id; 
    uint8_t component_id; 

    // MAVLink packet handling 
    mavlink_message_t msg; 
    mavlink_status_t status; 
    // Incoming data 
    uint16_t data_in_index; 
    uint16_t data_in_size; 
    uint8_t data_in_buff[VS_TELEMETRY_BUFF]; 
    // Outgoing data 
    uint16_t data_out_size; 
    uint8_t data_out_buff[VS_TELEMETRY_BUFF]; 

    // MAVLink messages 
    VehicleMAVLink mavlink; 

    // Status timers 
    uint8_t heartbeat_status_timer; 

    // Status flags 
    uint8_t connected : 1; 

private:   // Private methods 

    // MAVLink message decode 
    void MAVLinkPayloadDecode(Vehicle &vehicle); 

    // MAVLink message encode 
    void MAVLinkMessageFormat(void); 
    void MAVLinkMessageSend(Vehicle &vehicle); 

    // MAVLink: Heartbeat protocol 
    void MAVLinkHeartbeatReceive(void); 
    void MAVLinkHeartbeatSend(void); 

    // MAVLink: Parameter protocol 
    void MAVLinkParamRequestListReceive(void); 

    // MAVLink: Mission protocol 
    void MAVLinkMissionRequestReceive(void); 

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
    void MAVLinkRawIMUSendPeriodic(void); 
    void MAVLinkGPSRawIntSendPeriodic(void); 
    void MAVLinkRCChannelScaledSendPeriodic(void); 
    void MAVLinkRCChannelRawSendPeriodic(void); 
    void MAVLinkServoOutputRawSendPeriodic(void); 
    void MAVLinkAttitudeSendPeriodic(void); 
    void MAVLinkPositionTargetGlobalIntSendPeriodic(void); 
    void MAVLinkNavControllerSendPeriodic(void); 
    void MAVLinkLocalPositionNEDSendPeriodic(void); 
    void MAVLinkGlobalPositionIntSendPeriodic(void); 

public:   // Public methods 

    // MAVLink message handling 
    void MAVLinkMessageDecode(Vehicle &vehicle); 
    void MAVLinkMessageEncode(Vehicle &vehicle); 

    // Setters 
    void MAVLinkHeartbeatSetMode(uint8_t mode); 
};


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

    // Incoming messages from the GCS 
    mavlink_heartbeat_t heartbeat_msg_gcs;                       // HEARTBEAT 
    mavlink_command_long_t command_long_msg_gcs;                 // COMMAND_LONG 

    // Outgoing messages 
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
};

//=======================================================================================

#endif   // _TELEMETRY_H_ 
