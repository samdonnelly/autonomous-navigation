/**
 * @file navigation_utest.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Vehicle navigation unit tests 
 * 
 * @version 0.1
 * @date 2025-05-09
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
#include "hardware_mock.h" 

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

TEST_GROUP(vehicle_navigation_test)
{
    // Global test group variables 

    // Constructor 
    void setup()
    {
        // 
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
//=======================================================================================


//=======================================================================================
// Tests 

// Test 0 
TEST(vehicle_navigation_test, test0)
{
    // 
}

//=======================================================================================
