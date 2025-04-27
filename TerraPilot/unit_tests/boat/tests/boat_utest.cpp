/**
 * @file boat_utest.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Boat unit tests 
 * 
 * @version 0.1
 * @date 2025-04-22
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
#include "boat.h" 

// Mocks 
#include "vehicle_mock.h" 
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

Boat boat_utest; 

//=======================================================================================


//=======================================================================================
// Test group 

TEST_GROUP(boat_test)
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

// Manual control - differential thrust output 
TEST(boat_test, manual_diff_thrust_output)
{
    // Input data 
    std::array<VehicleControl::ChannelFunctions, VEHICLE_MOCK_CHANNEL_BUFF> channels = 
    {{
        // Throttle, roll, pitch, yaw, mode_select, mode, aux 3-10 
        { 2000, 2000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },   // 1 
        { 2000, 1750, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },   // 2 
        { 2000, 1500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },   // 3 
        { 2000, 1250, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },   // 4 
        { 2000, 1000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },   // 5 
        { 1750, 2000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },   // 6 
        { 1750, 1750, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },   // 7 
        { 1750, 1500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },   // 8 
        { 1750, 1250, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },   // 9 
        { 1750, 1000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },   // 10 
        { 1500, 2000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },   // 11 
        { 1500, 1750, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },   // 12 
        { 1500, 1500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },   // 13 
        { 1500, 1250, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },   // 14 
        { 1500, 1000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },   // 15 
        { 1250, 2000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },   // 16 
        { 1250, 1750, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },   // 17 
        { 1250, 1500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },   // 18 
        { 1250, 1250, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },   // 19 
        { 1250, 1000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },   // 20 
        { 1000, 2000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },   // 21 
        { 1000, 1750, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },   // 22 
        { 1000, 1500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },   // 23 
        { 1000, 1250, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },   // 24 
        { 1000, 1000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }    // 25 
    }}; 

    // Expected output 
    std::array<HardwareMock::ControlSetpoints, HW_MOCK_MAX_CONTROL_SETPOINTS> setpoints = 
    {{
        { 2000, 1500, 0, 0, 0 },   // 1 
        { 2000, 1750, 0, 0, 0 },   // 2 
        { 2000, 2000, 0, 0, 0 },   // 3 
        { 1750, 2000, 0, 0, 0 },   // 4 
        { 1500, 2000, 0, 0, 0 },   // 5 
        { 1750, 1500, 0, 0, 0 },   // 6 
        { 1750, 1625, 0, 0, 0 },   // 7 
        { 1750, 1750, 0, 0, 0 },   // 8 
        { 1625, 1750, 0, 0, 0 },   // 9 
        { 1500, 1750, 0, 0, 0 },   // 10 
        { 1500, 1500, 0, 0, 0 },   // 11 
        { 1500, 1500, 0, 0, 0 },   // 12
        { 1500, 1500, 0, 0, 0 },   // 13 
        { 1500, 1500, 0, 0, 0 },   // 14 
        { 1500, 1500, 0, 0, 0 },   // 15 
        { 1250, 1500, 0, 0, 0 },   // 16 
        { 1250, 1375, 0, 0, 0 },   // 17 
        { 1250, 1250, 0, 0, 0 },   // 18 
        { 1375, 1250, 0, 0, 0 },   // 19 
        { 1500, 1250, 0, 0, 0 },   // 20 
        { 1000, 1500, 0, 0, 0 },   // 21 
        { 1000, 1250, 0, 0, 0 },   // 22 
        { 1000, 1000, 0, 0, 0 },   // 23 
        { 1250, 1000, 0, 0, 0 },   // 24 
        { 1500, 1000, 0, 0, 0 }    // 25 
    }}; 

    hardware_mock.HardwareMockInit(); 
    vehicle_mock.TestFunctionIndexSet(VehicleMock::MANUAL_DRIVE); 
    vehicle_mock.RCChannelSet(channels); 
    
    // Entry point to the vehicle code 
    boat_utest.Loop(); 

    // Check that the data was formatted and written properly 
    std::array<HardwareMock::ControlSetpoints, HW_MOCK_MAX_CONTROL_SETPOINTS> output; 
    hardware_mock.ControlSetpointGet(output, channels.size()); 

    for (uint8_t i = RESET; i < channels.size(); i++)
    {
        // Differences are checked because integer math can lead to rounding. 
        
        uint16_t diff_1 = (setpoints[i].throttle_1 < output[i].throttle_1) ? 
                          (output[i].throttle_1 - setpoints[i].throttle_1) : 
                          (setpoints[i].throttle_1 - output[i].throttle_1); 

        uint16_t diff_2 = (setpoints[i].throttle_2 < output[i].throttle_2) ? 
                          (output[i].throttle_2 - setpoints[i].throttle_2) : 
                          (setpoints[i].throttle_2 - output[i].throttle_2); 
        
        UNSIGNED_LONGS_EQUAL(true, diff_1 <= 1); 
        UNSIGNED_LONGS_EQUAL(true, diff_2 <= 1); 
    }
}


// Autonomous control - differential thrust output 
TEST(boat_test, auto_diff_thrust_output)
{
    // Input data 
    std::array<int16_t, VEHICLE_MOCK_HEADING_ERROR_BUFF> heading_errors = 
    {
        -900,   // 1 
        -800,   // 2 
        -700,   // 3 
        -600,   // 4 
        -500,   // 5 
        -400,   // 6 
        -300,   // 7 
        -200,   // 8 
        -100,   // 9 
        0,      // 10 
        100,    // 11 
        200,    // 12 
        300,    // 13 
        400,    // 14 
        500,    // 15 
        600,    // 16 
        700,    // 17 
        800,    // 18 
        900     // 19 
    }; 

    // Expected output - based on max thruster PWM of 1750 and max heading error of 900 
    std::array<HardwareMock::ControlSetpoints, HW_MOCK_MAX_CONTROL_SETPOINTS> setpoints = 
    {{
        { 1500, 1750, 0, 0, 0 },   // 1 
        { 1527, 1750, 0, 0, 0 },   // 2 
        { 1555, 1750, 0, 0, 0 },   // 3 
        { 1583, 1750, 0, 0, 0 },   // 4 
        { 1611, 1750, 0, 0, 0 },   // 5 
        { 1638, 1750, 0, 0, 0 },   // 6 
        { 1666, 1750, 0, 0, 0 },   // 7 
        { 1694, 1750, 0, 0, 0 },   // 8 
        { 1722, 1750, 0, 0, 0 },   // 9 
        { 1750, 1750, 0, 0, 0 },   // 10 
        { 1750, 1722, 0, 0, 0 },   // 11 
        { 1750, 1694, 0, 0, 0 },   // 12 
        { 1750, 1666, 0, 0, 0 },   // 13 
        { 1750, 1638, 0, 0, 0 },   // 14 
        { 1750, 1611, 0, 0, 0 },   // 15 
        { 1750, 1583, 0, 0, 0 },   // 16 
        { 1750, 1555, 0, 0, 0 },   // 17 
        { 1750, 1527, 0, 0, 0 },   // 18 
        { 1750, 1500, 0, 0, 0 }    // 19 
    }}; 

    hardware_mock.HardwareMockInit(); 
    vehicle_mock.TestFunctionIndexSet(VehicleMock::AUTO_DRIVE); 
    vehicle_mock.HeadingErrorsSet(heading_errors); 
    
    // Entry point to the vehicle code 
    boat_utest.Loop(); 

    // Check that the data was formatted and written properly 
    std::array<HardwareMock::ControlSetpoints, HW_MOCK_MAX_CONTROL_SETPOINTS> output; 
    hardware_mock.ControlSetpointGet(output, heading_errors.size()); 

    for (uint8_t i = RESET; i < heading_errors.size(); i++)
    {
        // Differences are checked because integer math can lead to rounding. 

        uint16_t diff_1 = (setpoints[i].throttle_1 < output[i].throttle_1) ? 
                          (output[i].throttle_1 - setpoints[i].throttle_1) : 
                          (setpoints[i].throttle_1 - output[i].throttle_1); 

        uint16_t diff_2 = (setpoints[i].throttle_2 < output[i].throttle_2) ? 
                          (output[i].throttle_2 - setpoints[i].throttle_2) : 
                          (setpoints[i].throttle_2 - output[i].throttle_2); 
        
        UNSIGNED_LONGS_EQUAL(true, diff_1 <= 1); 
        UNSIGNED_LONGS_EQUAL(true, diff_2 <= 1); 
    }
}

//=======================================================================================
