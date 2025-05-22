/**
 * @file telemetry_utest.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Vehicle telemetry unit tests 
 * 
 * @version 0.1
 * @date 2025-03-28
 * 
 * @copyright Copyright (c) 2025
 * 
 */

//=======================================================================================
// Notes 
//=======================================================================================


//=======================================================================================
// Includes 

#include "CppUTest/TestHarness.h" 

// Production code 
#include "vehicle.h" 

// Mocks 
#include "vehicle_hardware_mock.h" 

// Library 
#include <iostream> 

extern "C"
{
	// Add your C-only include files here 
}

//=======================================================================================


//=======================================================================================
// Macros 
//=======================================================================================


//=======================================================================================
// Test data 

class Craft : public Vehicle 
{
private: 

    void VehicleSetup(void) override
    {
        // 
    }

    void MainStateSelect(uint8_t state) override
    {
        // 
    }

    void MainStateRCModeMap(uint8_t &mode) override
    {
        // 
    }

    void ManualDrive(VehicleControl::ChannelFunctions main_channels) override
    {
        // 
    }

    void AutoDrive(int16_t heading_error) override
    {
        // 
    }

public: 

    // Constructor/Destructor 
    Craft() 
        : Vehicle(MAV_TYPE_GENERIC) {}
    
    ~Craft() {}

    void TelemetryReadySet(void)
    {
        hardware.data_ready.telemetry_ready = FLAG_SET; 
    }

    void TelemetryDecode(void)
    {
        telemetry.MessageDecode(*this); 
    }

    void TelemetryEncode(void)
    {
        telemetry.MessageEncode(*this); 
    }
}; 

static Craft craft; 

//=======================================================================================


//=======================================================================================
// Test group 

TEST_GROUP(vehicle_telemetry_test)
{
    // Global test group variables 

    // Constructor 
    void setup()
    {
        hardware_mock.HardwareMockInit(); 
    }

    // Destructor 
    void teardown()
    {
        // 
    }
}; 

//=======================================================================================


//=======================================================================================
// Helper functions 

void RequestDataStreamQueue(
    uint16_t msg_rate, 
    uint8_t stream_id, 
    uint8_t start_stop)
{
    mavlink_message_t msg; 
    uint8_t msg_buff[sizeof(mavlink_message_t)]; 

    mavlink_request_data_stream_t request_data_stream_msg = 
    {
        .req_message_rate = msg_rate, 
        .target_system = VS_SYSTEM_ID, 
        .target_component = MAV_COMP_ID_AUTOPILOT1, 
        .req_stream_id = stream_id, 
        .start_stop = start_stop, 
    };

    mavlink_msg_request_data_stream_encode(
        VS_SYSTEM_ID_GCS, 
        MAV_COMP_ID_MISSIONPLANNER, 
        &msg, 
        &request_data_stream_msg); 

    hardware_mock.TelemetryInAppend(mavlink_msg_to_send_buffer(msg_buff, &msg), msg_buff); 
    craft.TelemetryReadySet(); 
}

//=======================================================================================


//=======================================================================================
// Tests 

// Telemetry: Send buffer size 
TEST(vehicle_telemetry_test, telemetry_out_size)
{
    // Enable periodic send functions by decoding a REQUEST_DATA_STREAM message then 
    // call the encoding function numerous times to observe the total size of all 
    // messages to be sent. 

    //==================================================
    // Enable messages to send 

    RequestDataStreamQueue(4, MAV_DATA_STREAM_RAW_SENSORS, 1); 
    RequestDataStreamQueue(4, MAV_DATA_STREAM_RC_CHANNELS, 1); 
    RequestDataStreamQueue(4, MAV_DATA_STREAM_RAW_CONTROLLER, 1); 
    RequestDataStreamQueue(4, MAV_DATA_STREAM_POSITION, 1); 

    craft.TelemetryDecode(); 

    //==================================================

    //==================================================
    // Send messages and observe volume 
    
    // Call encoding function multiple times (arbitrary) to see if the total send length is 
    // larger than the available buffer. 
    for (uint8_t i = RESET; i < 10; i++)
    {
        craft.TelemetryEncode(); 
        UNSIGNED_LONGS_EQUAL(true, (hardware_mock.TelemetryOutGetSize() < VS_TELEMETRY_BUFF)); 
    }
    
    //==================================================
}

//=======================================================================================
