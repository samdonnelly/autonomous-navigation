/**
 * @file navigation.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Vehicle navigation interface 
 * 
 * @version 0.1
 * @date 2025-03-13
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef _NAVIGATION_H_ 
#define _NAVIGATION_H_ 

//=======================================================================================
// Includes 

#include "includes.h" 

//=======================================================================================


//=======================================================================================
// Macros 

#define DEGREE_DATATYPE 10000000   // int32_t <--> float - removes or adds decimal places 

//=======================================================================================


//=======================================================================================
// Classes 

class Vehicle; 


class VehicleNavigation 
{
private:   // private types 

    enum HeadingDirections : int16_t 
    {
        HEADING_NORTH = 0,      // Heading reading when facing North (0 deg*10) 
        HEADING_SOUTH = 1800,   // Heading reading when facing South (180 deg*10) 
        HEADING_RANGE = 3600    // Full heading range (360 deg*10) 
    }; 

public:   // public types 

    struct Location 
    {
        int32_t latI, lonI, altI;   // degE7, mm 
        float lat, lon, alt;        // deg, m 
    };

    template <typename T>
    struct Vector 
    {
        T x, y, z; 
    };

private:   // private members 

    struct Timers 
    {
        uint8_t gps_connection; 
        uint8_t imu_connection; 
    }
    timers; 

    union Status
    {
        struct 
        {
            uint32_t gps_connected     : 1; 
            uint32_t gps_status_change : 1; 
            uint32_t imu_connected     : 1; 
        }; 
        uint32_t flags; 
    }
    status; 

    // Location 
    Location location_current, location_filtered, location_target;   // WGS84 
    mavlink_mission_item_int_t mission_target; 
    float waypoint_distance, waypoint_radius;                        // Distance to target waypoint & waypoint acceptance 
    float coordinate_lpf_gain;                                       // Low pass filter gain for GPS coordinates 

    // Orientation 
    Vector<int16_t> accel_raw, gyro_raw, mag_raw;                    // Accelerometer, gyroscope and magnetometer raw data 
    Vector<float> orient;                                            // x = roll, y = pitch, z = yaw (degrees) 
    Vector<float> mag_cal, mag_hi, mag_sid, mag_sio;                 // Magnetometer correction 
    int16_t true_north_offset;                                       // True north offset (degrees*10) 
    int16_t heading, heading_target;                                 // 0-3599 (degrees*10) 
    
public:   // public members 
    
    // Location 
    GPS_FIX_TYPE fix_type; 
    MAV_FRAME coordinate_frame; 
    uint16_t position_type_mask; 
    uint16_t ground_speed; 
    uint16_t num_satellite; 

private:   // private methods 

    // Navigation data handling 
    void LocationChecks(Vehicle &vehicle); 
    void OrientationChecks(Vehicle &vehicle); 
    
    // Mission execution 
    void TargetUpdate(Vehicle &vehicle); 
    
    // Location calculations 
    void TargetWaypoint(Vehicle &vehicle); 
    void WaypointError(void); 

    // Heading calculations 
    int16_t HeadingError(void); 
    void HeadingRangeCheck(int16_t &heading_value); 

public:   // public methods 

    // Constructor 
    VehicleNavigation(); 

    // Navigation data handling 
    void LocationUpdate(Vehicle &vehicle); 
    void OrientationUpdate(Vehicle &vehicle); 
    
    // Mission execution 
    void TargetAssess(Vehicle &vehicle); 
    void CourseCorrection(Vehicle &vehicle); 

    // Getters 
    Location LocationCurrentGet(void); 
    Vector<int16_t> AccelCurrentGet(void); 
    Vector<int16_t> GyroCurrentGet(void); 
    Vector<int16_t> MagCurrentGet(void); 
    Vector<float> OrientationCurrentGet(void); 
    int16_t HeadingCurrentGet(void); 
    int16_t HeadingTargetGet(void); 
    uint16_t WaypointDistanceGet(void); 
    
    // Setters 
    void TrueNorthOffsetSet(int16_t compass_tn); 
    void MagHardIronXSet(float compass_hix); 
    void MagHardIronYSet(float compass_hiy); 
    void MagHardIronZSet(float compass_hiz); 
    void MagSoftIronDiagonalXSet(float compass_sidx); 
    void MagSoftIronDiagonalYSet(float compass_sidy); 
    void MagSoftIronDiagonalZSet(float compass_sidz); 
    void MagSoftIronOffDiagonalXSet(float compass_siox); 
    void MagSoftIronOffDiagonalYSet(float compass_sioy); 
    void MagSoftIronOffDiagonalZSet(float compass_sioz); 
    void WaypointRadiusSet(float wp_radius); 
}; 

//=======================================================================================

#endif   // _NAVIGATION_H_ 
