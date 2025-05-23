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

#define NAV_TEST_NUM_DIRS 12 

//=======================================================================================


//=======================================================================================
// Test data 

class Craft : public Vehicle 
{
public:   // public members 

    int16_t heading_diff; 

private:   // private methods 

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
        heading_diff = heading_error; 
    }

public:   // public methods 

    // Constructor/Destructor 
    Craft() 
        : Vehicle(MAV_TYPE_GENERIC) {}
    
    ~Craft() {}

    void NavDataReadySet(void)
    {
        hardware.data_ready.gps_ready = FLAG_SET; 
        hardware.data_ready.imu_ready = FLAG_SET; 
    }

    void NavLocationUpdate(void)
    {
        navigation.LocationUpdate(*this); 
    }

    void NavOrientationUpdate(void)
    {
        navigation.OrientationUpdate(*this); 
    }

    void NavTargetAssess(void)
    {
        navigation.TargetAssess(*this); 
    }

    void NavCourseCorrection(void)
    {
        navigation.CourseCorrection(*this); 
    }

    void NavTrueNorthOffsetSet(int16_t tn_offset)
    {
        navigation.TrueNorthOffsetSet(tn_offset); 
    }
}; 

static Craft craft; 


static const std::array<VehicleNavigation::Vector<int16_t>, NAV_TEST_NUM_DIRS> mag_axis = 
{{
    // x, y, z 
    {  5,  0, 0 },   // 1 - North 
    {  4,  3, 0 },   // 2 
    {  3,  4, 0 },   // 3 
    {  0,  5, 0 },   // 4 - East 
    { -3,  4, 0 },   // 5 
    { -4,  3, 0 },   // 6 
    { -5,  0, 0 },   // 7 - South 
    { -4, -3, 0 },   // 8 
    { -3, -4, 0 },   // 9 
    {  0, -5, 0 },   // 10 - West 
    {  3, -4, 0 },   // 11 
    {  4, -3, 0 }    // 12 
}};


static const std::array<int16_t, NAV_TEST_NUM_DIRS> heading_errors = 
{
    0,       // 1 - North 
    -368,    // 2 
    -531,    // 3 
    -900,    // 4 - East 
    -1268,   // 5 
    -1431,   // 6 
    1800,    // 7 - South 
    1431,    // 8 
    1268,    // 9 
    900,     // 10 - West 
    531,     // 11 
    368,     // 12 
};

//=======================================================================================


//=======================================================================================
// Test group 

TEST_GROUP(vehicle_navigation_test)
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
//=======================================================================================


//=======================================================================================
// Tests 

// Heading: magnetic North 
TEST(vehicle_navigation_test, heading_magnetic_north)
{
    // This test checks the heading error output for various magnetometer readings and 
    // no true north offset correction. 

    // Test data 
    int16_t tn_offset = 0; 

    // Set parameters needed for the heading calculations 
    craft.NavTrueNorthOffsetSet(tn_offset); 

    // Perform the heading calculation and check the results 
    for (uint8_t i = RESET; i < NAV_TEST_NUM_DIRS; i++)
    {
        // Update the magnetometer data the autopilot gets 
        hardware_mock.IMUSetAxisData(mag_axis[i]); 

        // Makes sure the GPS and IMU connection flags are set so navigation calculations 
        // will be performed by the code. 
        craft.NavDataReadySet(); 

        // Run the update functions so GPS and IMU devices are recorded as connected. 
        craft.NavLocationUpdate(); 
        craft.NavOrientationUpdate(); 

        // Perform the heading calculations 
        craft.NavCourseCorrection(); 
        LONGS_EQUAL(heading_errors[i], craft.heading_diff); 
    }
}


// Heading: true North 
TEST(vehicle_navigation_test, heading_true_north)
{
    // Test different true North offsets 
}

//=======================================================================================
