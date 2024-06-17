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

// Module info 
#define DEFAULT_GPS_RAD 100      // Default GPS coordinate radius threshold (m*10) 
#define GPS_FILTER_GAIN 0.5      // GPS coordinate filter gain 
#define DEFAULT_TN_OFFSET 130    // Default true north offset from magnetic north 

// Calculation info 
#define NUM_GPS_WAYPOINTS 10     // Number of waypoints in the sample mission 
#define HEAD_ERROR_THRESH 2      // Max allowed difference in heading error checks 
#define HEAD_ERROR_OFFSET 3600   // Correction for when heading crosses 0/360 degrees 

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


enum waypoint_index : uint8_t 
{
    WAYPOINT_INDEX_0, 
    WAYPOINT_INDEX_1, 
    WAYPOINT_INDEX_2, 
    WAYPOINT_INDEX_3, 
    WAYPOINT_INDEX_4, 
    WAYPOINT_INDEX_5, 
    WAYPOINT_INDEX_6, 
    WAYPOINT_INDEX_7, 
    WAYPOINT_INDEX_8, 
    WAYPOINT_INDEX_9, 
    WAYPOINT_INDEX_10   // Out of range 
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

// Heading error calculation 
TEST(nav_module_test, heading_error_calc)
{
    // For this test, a vehicle is simulated to be at some location and one of the 
    // gps_waypoints is used are the target destination. The vehicle is simulated 
    // to be at various headings so the error between the vehcile heading and the 
    // coordinate heading can be found. 
    // - Target waypoint: gps_waypoints[0] 
    // - Current location: interpolation between gps_waypoints[1] and gps_waypoints[8] 

    // Sample location (interpolated between two mission coordinates) 
    gps_waypoints_t sample_location = 
    {
        (gps_waypoints[WAYPOINT_INDEX_1].lat + gps_waypoints[WAYPOINT_INDEX_8].lat) / 2, 
        (gps_waypoints[WAYPOINT_INDEX_1].lon + gps_waypoints[WAYPOINT_INDEX_8].lon) / 2 
    }; 

    // Heading between vechile location and target waypoint. Calculated using the 
    // simulation scripts. 
    int16_t coordinate_heading = 3115; 

    // Sample vehicle headings and their errors with the coordinate heading. True north 
    // heading is used as this is the correction made in the heading error calculation. 
    int16_t sample_headings[4] = { 450, 1350, 2250, 3150 }; 
    int16_t sample_errors[4] = 
    {
        coordinate_heading - (sample_headings[0] + DEFAULT_TN_OFFSET) - HEAD_ERROR_OFFSET, 
        coordinate_heading - (sample_headings[1] + DEFAULT_TN_OFFSET), 
        coordinate_heading - (sample_headings[2] + DEFAULT_TN_OFFSET), 
        coordinate_heading - (sample_headings[3] + DEFAULT_TN_OFFSET) 
    }; 

    // Set the number of waypoints, set the target waypoint and set the current location 
    // within the navigation module. The target waypoint must be set before the current 
    // location because the current location set function also updates the coordinate 
    // heading. 
    navigation.NavTestSetNumWaypoints(NUM_GPS_WAYPOINTS); 
    navigation.NavTestSetTargetWaypoint(WAYPOINT_INDEX_0, gps_waypoints); 
    navigation.NavTestSetCurrentLocation(sample_location); 

    // The difference between the exected and actual error is found ahead of time to 
    // make the code cleaner. 
    int16_t error_calc_check[4] = 
    {
        sample_errors[0] - navigation.NavTestHeadingError(sample_headings[0]), 
        sample_errors[1] - navigation.NavTestHeadingError(sample_headings[1]), 
        sample_errors[2] - navigation.NavTestHeadingError(sample_headings[2]), 
        sample_errors[3] - navigation.NavTestHeadingError(sample_headings[3]) 
    }; 

    // Check that the heading error calculation falls within the threshold 
    LONGS_EQUAL(TRUE, abs(error_calc_check[0]) <= HEAD_ERROR_THRESH); 
    LONGS_EQUAL(TRUE, abs(error_calc_check[1]) <= HEAD_ERROR_THRESH); 
    LONGS_EQUAL(TRUE, abs(error_calc_check[2]) <= HEAD_ERROR_THRESH); 
    LONGS_EQUAL(TRUE, abs(error_calc_check[3]) <= HEAD_ERROR_THRESH); 
}


// Target waypoint hit 
TEST(nav_module_test, target_waypoint_hit)
{
    // For this test, a vehicle is simulated to be at some location and one of the 
    // gps_waypoints is used are the target destination. The threshold radius (distance) 
    // between the current and target location that determines if a waypoint has been 
    // "hit" is updated to get the code to jump to the next target. In practice, the 
    // vehicle would move in the direction of the target and the calculated radius would 
    // decrease until it's within the threshold but for this test it's easier to just 
    // change the radius threshold. It also gives the opportunity to use the radius 
    // setter function. 
    // - Target waypoint: gps_waypoints[5] 
    // - Current location: gps_waypoints[4] 

    // Target and sample location 
    gps_waypoints_t target_location = { 0.0, 0.0 }; 
    gps_waypoints_t sample_location = 
    {
        gps_waypoints[WAYPOINT_INDEX_4].lat, 
        gps_waypoints[WAYPOINT_INDEX_4].lon 
    }; 
    uint8_t waypoint_index = WAYPOINT_INDEX_0; 

    // Set the number of waypoints, set the target waypoint and set the current location 
    // within the navigation module. The target waypoint must be set before the current 
    // location because the current location set function also updates the coordinate 
    // heading. 
    navigation.NavTestSetNumWaypoints(NUM_GPS_WAYPOINTS); 
    navigation.NavTestSetTargetWaypoint(WAYPOINT_INDEX_5, gps_waypoints); 
    navigation.NavTestSetCurrentLocation(sample_location); 

    // Run the location error function and check that the target waypoint is still the same. 
    navigation.NavTestLocationError(sample_location, gps_waypoints); 
    waypoint_index = navigation.GetTargetWaypoint(target_location); 
    UNSIGNED_LONGS_EQUAL(WAYPOINT_INDEX_5, waypoint_index); 
    DOUBLES_EQUAL(gps_waypoints[WAYPOINT_INDEX_5].lat, target_location.lat, 0.000000005); 
    DOUBLES_EQUAL(gps_waypoints[WAYPOINT_INDEX_5].lon, target_location.lon, 0.000000005); 

    // Change the coordinate radius to something larger than the actual radius. The 
    // default coordinate radius in the module is set to 10 meters (or 100 meters*10) and 
    // the distance between coordinates at index 4 and 5 ~7173 meters. The coordinate 
    // radius gets updated to anything larger than 7173 meters (or 71730 meters*10) so the 
    // vehcile is now within the threshold of hitting the waypoint. The distance between 
    // these points was found using the simulation scripts. 
    navigation.SetCoordinateRadius(80000); 

    // Run the location error function again and check that the target waypoint has been 
    // updated. 
    navigation.NavTestLocationError(sample_location, gps_waypoints); 
    waypoint_index = navigation.GetTargetWaypoint(target_location); 
    UNSIGNED_LONGS_EQUAL(WAYPOINT_INDEX_6, waypoint_index); 
    DOUBLES_EQUAL(gps_waypoints[WAYPOINT_INDEX_6].lat, target_location.lat, 0.000000005); 
    DOUBLES_EQUAL(gps_waypoints[WAYPOINT_INDEX_6].lon, target_location.lon, 0.000000005); 
}


// Target waypoint change 
TEST(nav_module_test, target_waypoint_change)
{
    // Check that the target waypoint updates only when a valid index is requested 

    gps_waypoints_t target_location = { 0.0, 0.0 }; 
    uint8_t waypoint_index = WAYPOINT_INDEX_0; 
    uint8_t set_status = TRUE; 

    // Set the number of waypoints and target waypoint within the navigation module. 
    navigation.NavTestSetNumWaypoints(NUM_GPS_WAYPOINTS); 
    navigation.NavTestSetTargetWaypoint(WAYPOINT_INDEX_3, gps_waypoints); 

    // Check that the target waypoint is what we expect 
    waypoint_index = navigation.GetTargetWaypoint(target_location); 
    UNSIGNED_LONGS_EQUAL(WAYPOINT_INDEX_3, waypoint_index); 
    DOUBLES_EQUAL(gps_waypoints[WAYPOINT_INDEX_3].lat, target_location.lat, 0.000000005); 
    DOUBLES_EQUAL(gps_waypoints[WAYPOINT_INDEX_3].lon, target_location.lon, 0.000000005); 

    // Try setting a target that is out of range then check that the setter returns 
    // false and the target is unchanged. 
    set_status = navigation.NavTestSetTargetWaypoint(WAYPOINT_INDEX_10, gps_waypoints); 
    UNSIGNED_LONGS_EQUAL(FALSE, set_status); 
    waypoint_index = navigation.GetTargetWaypoint(target_location); 
    UNSIGNED_LONGS_EQUAL(WAYPOINT_INDEX_3, waypoint_index); 
    DOUBLES_EQUAL(gps_waypoints[WAYPOINT_INDEX_3].lat, target_location.lat, 0.000000005); 
    DOUBLES_EQUAL(gps_waypoints[WAYPOINT_INDEX_3].lon, target_location.lon, 0.000000005); 

    // Try setting a different target that's within bounds and check that the setter 
    // returns true and that the target is updated. 
    set_status = navigation.NavTestSetTargetWaypoint(WAYPOINT_INDEX_7, gps_waypoints); 
    UNSIGNED_LONGS_EQUAL(TRUE, set_status); 
    waypoint_index = navigation.GetTargetWaypoint(target_location); 
    UNSIGNED_LONGS_EQUAL(WAYPOINT_INDEX_7, waypoint_index); 
    DOUBLES_EQUAL(gps_waypoints[WAYPOINT_INDEX_7].lat, target_location.lat, 0.000000005); 
    DOUBLES_EQUAL(gps_waypoints[WAYPOINT_INDEX_7].lon, target_location.lon, 0.000000005); 
}

//=======================================================================================
