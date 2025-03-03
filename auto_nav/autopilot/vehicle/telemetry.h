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

#include "hardware.h" 

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
    uint16_t data_index; 

    // MAVLink messages 
    VehicleMAVLink mavlink; 

    // Status timers 
    uint8_t heartbeat_status_timer; 

    // Status flags 
    uint8_t connected : 1; 

private:   // Private methods 

    // MAVLink message payload decode 
    void MAVLinkPayloadDecode(void); 

    // Payload handling 
    void MAVLinkHeartbeat(void); 
    void MAVLinkParamRequestList(void); 
    void MAVLinkMissionRequest(void); 
    void MAVLinkRequestDataStream(void); 
    void MAVLinkCommandLong(void); 

public:   // Public methods 

    // MAVLink message decode 
    void MAVLinkMessageDecode(VehicleHardware &hardware); 
};


class VehicleMAVLink 
{
    // MAVLink messages and properites common to all vehicles. 
    
public:   // Public members 

    // Incoming messages 
    mavlink_heartbeat_t heartbeat_msg_gcs;                       // HEARTBEAT 

    // Outgoing messages 
    mavlink_heartbeat_t heartbeat_msg;                           // HEARTBEAT 
};

//=======================================================================================

#endif   // _TELEMETRY_H_ 
