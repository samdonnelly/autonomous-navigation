/**
 * @file nav_module_utest.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Navigation module unit tests 
 * 
 * @version 0.1
 * @date 2024-06-12
 * 
 * @copyright Copyright (c) 2024
 * 
 */

//=======================================================================================
// Notes 
//=======================================================================================


//=======================================================================================
// Includes 

#include "CppUTest/TestHarness.h" 

// Production code 
#include "nav_module.h" 

// Standard library 
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
// Classes 

class VehicleNav : public NavModule 
{
public: 
    // Constructor 
    VehicleNav() 
        : NavModule(0, 0.0, 0) {}

    // Destructor 
    ~VehicleNav() {} 

    // Heading Error wrapper 
    int16_t NavTestHeadingError(int16_t heading)
    {
        return HeadingError(heading); 
    }

    // Location Error wrapper 
    void NavTestLocationError(
        gps_waypoints_t& position, 
        const gps_waypoints_t *waypoints)
    {
        LocationError(position, waypoints); 
    }

    // Set Target Waypoint wrapper 
    uint8_t NavTestSetTargetWaypoint(
        uint8_t index, 
        const gps_waypoints_t *waypoints)
    {
        return SetTargetWaypoint(index, waypoints); 
    }

    // Set Current Location wrapper 
    void NavTestSetCurrentLocation(const gps_waypoints_t& position)
    {
        SetCurrentLocation(position); 
    }

    // Set Num Waypoints wrapper 
    void NavTestSetNumWaypoints(uint8_t num_waypoints)
    {
        SetNumWaypoints(num_waypoints); 
    }
}; 

//=======================================================================================


//=======================================================================================
// Helper functions 
//=======================================================================================


//=======================================================================================
// Test group 

TEST_GROUP(nav_module_test)
{
    // Global test group variables 
    VehicleNav navigation; 

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
// Tests 

// Check heading error 
// Change coordinate radius and check waypoint update 
// Check waypoint update when radius gets small enough 
// Check that the target waypoint updates when a valid index is requested 
// Use the heading error & location error calcs to verify a current location update 
// Use the location error calc to verify a change in number of waypoints 

// Add a getter for the current target waypoint 

// Test 0 
TEST(nav_module_test, test0)
{
    // 
}

//=======================================================================================
