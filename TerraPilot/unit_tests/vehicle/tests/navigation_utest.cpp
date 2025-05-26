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

#define NAV_TEST_HEADING_NUM_DIRS 12 
#define NAV_TEST_HEADING_NUM_TARGETS 5 
#define NAV_TEST_HEADING_NUM_OFFSETS 3 
#define NAV_TEST_HEADING_NUM_CASES NAV_TEST_HEADING_NUM_TARGETS * NAV_TEST_HEADING_NUM_OFFSETS 

//=======================================================================================


//=======================================================================================
// Test vehicle 

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

    void NavMissionItemSet(MissionItem mission_item)
    {
        memory.MissionItemSet(mission_item); 
    }

    void NavMissionTargetSet(uint16_t sequence)
    {
        memory.MissionTargetSet(sequence); 
    }

    void NavMissionSizeSet(uint16_t size)
    {
        memory.MissionSizeSet(size); 
    }
}; 

static Craft craft; 

//=======================================================================================


//=======================================================================================
// Test data 

static const VehicleNavigation::Location current_location = 
{
    .latI = RESET, 
    .lonI = RESET, 
    .altI = RESET, 
    .lat = 50.6119710, 
    .lon = -115.1235290, 
    .alt = RESET 
};

//==================================================
// Heading calculations 

static const std::array<VehicleNavigation::Vector<int16_t>, NAV_TEST_HEADING_NUM_DIRS> mag_axis = 
{{
    // x,  y,  z 
    {  5,  0,  0 },   // 1 - North 
    {  4,  3,  0 },   // 2 
    {  3,  4,  0 },   // 3 
    {  0,  5,  0 },   // 4 - East 
    { -3,  4,  0 },   // 5 
    { -4,  3,  0 },   // 6 
    { -5,  0,  0 },   // 7 - South 
    { -4, -3,  0 },   // 8 
    { -3, -4,  0 },   // 9 
    {  0, -5,  0 },   // 10 - West 
    {  3, -4,  0 },   // 11 
    {  4, -3,  0 }    // 12 
}};


static const std::array<VehicleNavigation::Location, NAV_TEST_HEADING_NUM_TARGETS> heading_targets = 
{{
    // These are based on the 'current_location' 
    // latI, lonI, altI, lat, lon, alt 
    { 506149710, -1151235290, 0, 0, 0, 0 },   // 1 --> Target heading == 0 deg heading 
    { 506149710, -1151231150, 0, 0, 0, 0 },   // 2 --> Target heading == 5 deg heading 
    { 506149710, -1151243600, 0, 0, 0, 0 },   // 3 --> Target heading == 350 deg heading 
    { 506089710, -1151231150, 0, 0, 0, 0 },   // 4 --> Target heading == 175 deg heading 
    { 506089710, -1151243600, 0, 0, 0, 0 }    // 5 --> Target heading == 190 deg heading 
}};


static const std::array<int16_t, NAV_TEST_HEADING_NUM_OFFSETS> tn_offsets = 
{
    0,     // 1 
    135,   // 2 
    -400   // 3 
};


static const std::array<std::array<int16_t, NAV_TEST_HEADING_NUM_CASES>, NAV_TEST_HEADING_NUM_CASES> heading_errors = 
{{
    //    N,    NE,    EN,     E,    ES,    SE,     S,    SW,    WS,     W,    WN,    NW 
    // Target heading 1 
    {     0,  -368,  -531,  -900, -1268, -1431,  1800,  1431,  1268,   900,   531,   368 },   // TN offset 1 
    {  -135,  -503,  -666, -1035, -1403, -1566,  1665,  1296,  1133,   765,   396,   233 },   // TN offset 2 
    {   400,    32,  -131,  -500,  -868, -1031, -1400, -1769,  1668,  1300,   931,   768 },   // TN offset 3 
    // Target heading 2 
    {    50,  -318,  -481,  -850, -1218, -1381, -1750,  1481,  1318,   950,   581,   418 },   // TN offset 1 
    {   -85,  -453,  -616,  -985, -1353, -1516,  1715,  1346,  1183,   815,   446,   283 },   // TN offset 2 
    {   450,    82,   -81,  -450,  -818,  -981, -1350, -1719,  1718,  1350,   981,   818 },   // TN offset 3 
    // Target heading 3 
    {  -100,  -468,  -631, -1000, -1368, -1531,  1700,  1331,  1168,   800,   431,   268 },   // TN offset 1 
    {  -235,  -603,  -766, -1135, -1503, -1666,  1565,  1196,  1033,   665,   296,   133 },   // TN offset 2 
    {   300,   -68,  -231,  -600,  -968, -1131, -1500,  1731,  1568,  1200,   831,   668 },   // TN offset 3 
    // Target heading 4 
    {  1750,  1382,  1219,   850,   482,   319,   -50,  -419,  -582,  -950, -1319, -1482 },   // TN offset 1 
    {  1615,  1247,  1084,   715,   347,   184,  -185,  -554,  -717, -1085, -1454, -1617 },   // TN offset 2 
    { -1450,  1782,  1619,  1250,   882,   719,   350,   -19,  -182,  -550,  -919, -1082 },   // TN offset 3 
    // Target heading 5 
    { -1700,  1532,  1369,  1000,   632,   469,   100,  -269,  -432,  -800, -1169, -1332 },   // TN offset 1 
    {  1765,  1397,  1234,   865,   497,   334,   -35,  -404,  -567,  -935, -1304, -1467 },   // TN offset 2 
    { -1300, -1668,  1769,  1400,  1032,   869,   500,   131,   -32,  -400,  -769,  -932 }    // TN offset 3 
}};

//==================================================

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

// Heading calculation 
TEST(vehicle_navigation_test, heading_calculation)
{
    // This test checks the heading error output for various magnetometer readings and 
    // no true north offset correction. 

    uint8_t case_index = RESET; 
    MissionItem mission_item = 
    {
        .param1 = RESET, 
        .param2 = RESET, 
        .param3 = RESET, 
        .param4 = RESET, 
        .x = RESET, 
        .y = RESET, 
        .z = RESET, 
        .seq = HOME_INDEX, 
        .command = MAV_CMD_NAV_WAYPOINT, 
        .target_system = RESET, 
        .target_component = RESET, 
        .frame = RESET, 
        .current = RESET, 
        .autocontinue = false, 
        .mission_type = MAV_MISSION_TYPE_MISSION 
    };

    hardware_mock.GPSSetLocation(current_location); 
    craft.NavMissionSizeSet(MAX_MISSION_SIZE); 

    for (uint8_t i = RESET; i < NAV_TEST_HEADING_NUM_TARGETS; i++)
    {
        // Select a target heading 
        mission_item.x = heading_targets[i].latI; 
        mission_item.y = heading_targets[i].lonI; 
        craft.NavMissionItemSet(mission_item); 

        for (uint8_t j = RESET; j < NAV_TEST_HEADING_NUM_OFFSETS; j++)
        {
            // Select a ture north offset 
            craft.NavTrueNorthOffsetSet(tn_offsets[j]); 
    
            // Perform the heading calculation and check the results 
            for (uint8_t k = RESET; k < NAV_TEST_HEADING_NUM_DIRS; k++)
            {
                // Update the magnetometer data the autopilot gets 
                hardware_mock.IMUSetAxisData(mag_axis[k]); 
    
                // Makes sure the GPS and IMU connection flags are set so navigation calculations 
                // will be performed by the code. 
                craft.NavDataReadySet(); 
    
                // Run the update functions so GPS and IMU devices are recorded as connected. 
                craft.NavLocationUpdate(); 
                craft.NavOrientationUpdate(); 

                craft.NavTargetAssess(); 
    
                // Perform the heading calculations 
                craft.NavCourseCorrection(); 
                LONGS_EQUAL(true, abs(heading_errors[case_index][k] - craft.heading_diff) <= 1); 
            }

            case_index++; 
        }

        craft.NavMissionTargetSet(++mission_item.seq); 
    }
}

//=======================================================================================
