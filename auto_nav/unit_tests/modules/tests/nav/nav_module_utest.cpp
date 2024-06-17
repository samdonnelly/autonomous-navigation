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

#define DEFAULT_GPS_RAD 100     // Default GPS coordinate radius threshold (m*10) 
#define GPS_FILTER_GAIN 0.5     // GPS coordinate filter gain 
#define DEFAULT_TN_OFFSET 130   // Default true north offset from magnetic north 
#define NUM_GPS_WAYPOINTS 10    // Number of waypoints in the sample mission 

//=======================================================================================


//=======================================================================================
// Coordinates 

const gps_waypoints_t gps_waypoints[NUM_GPS_WAYPOINTS] = 
{
    {43.50423880, -79.44145200},  // Index 0 
    {43.46288730, -79.49226380},  // Index 1 
    {43.40604520, -79.46960450},  // Index 2 
    {43.41602140, -79.25880430},  // Index 3 
    {43.37111570, -79.18876650},  // Index 4 
    {43.32168100, -79.24575810},  // Index 5 
    {43.32068190, -79.33982850},  // Index 6 
    {43.36462650, -79.43527220},  // Index 7 
    {43.46537920, -79.26567080},  // Index 8 
    {43.51170900, -79.33364870}   // Index 9 
}; 

//=======================================================================================


//=======================================================================================
// Classes 

class VehicleNav : public NavModule 
{
public: 
    // Constructor 
    VehicleNav() 
        : NavModule(DEFAULT_GPS_RAD, GPS_FILTER_GAIN, DEFAULT_TN_OFFSET) {}

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

// The nav module uses the navigation calculations library code but these tests are not 
// meant to specifically test these calculations as they already have unit tests. 

// Check heading error 
// Change coordinate radius and check waypoint update 
// Check waypoint update when radius gets small enough 
// Check that the target waypoint updates when a valid index is requested 
// Use the heading error & location error calcs to verify a current location update 
// Use the location error calc to verify a change in number of waypoints 

// Heading error calculation 
TEST(nav_module_test, heading_error_calc)
{
    // Sample location (interpolated between two mission coordinates) 
    gps_waypoints_t sample_location = 
    {
        (gps_waypoints[1].lat + gps_waypoints[8].lat) / 2, 
        (gps_waypoints[1].lon + gps_waypoints[8].lon) / 2 
    }; 

    // Sample vehicle heading. Different headings are used to test different errors. 
    int16_t sample_headings[4] = 
    {
        450, 
        1350, 
        2250, 
        3150 
    }; 

    int16_t sample_errors[4] = 
    {
        sample_headings[0] + DEFAULT_TN_OFFSET, 
        sample_headings[1] + DEFAULT_TN_OFFSET, 
        sample_headings[2] + DEFAULT_TN_OFFSET, 
        sample_headings[3] + DEFAULT_TN_OFFSET 
    }; 

    // Set the number of waypoints 
    navigation.NavTestSetNumWaypoints(NUM_GPS_WAYPOINTS); 
    // Set the target waypoint (SetTargetWaypoint) - this should be done before 
    // setting the current location 
    navigation.NavTestSetTargetWaypoint(0, gps_waypoints); 
    // Set the current location (SetCurrentLocation) 
    navigation.NavTestSetCurrentLocation(sample_location); 

    // 
    LONGS_EQUAL(0, navigation.NavTestHeadingError(sample_headings[0])); 
    LONGS_EQUAL(0, navigation.NavTestHeadingError(sample_headings[1])); 
    LONGS_EQUAL(0, navigation.NavTestHeadingError(sample_headings[2])); 
    LONGS_EQUAL(0, navigation.NavTestHeadingError(sample_headings[3])); 
}

//=======================================================================================
