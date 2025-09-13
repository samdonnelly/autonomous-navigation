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
// Constants 

constexpr float coordinate_scalar = 10000000.0f;   // Adds or removes decimal places of lat, lon, and alt 

//=======================================================================================


//=======================================================================================
// Classes 

class Vehicle; 


class VehicleNavigation 
{
public:

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

    // Location 
    GPS_FIX_TYPE fix_type; 
    MAV_FRAME coordinate_frame; 
    uint16_t position_type_mask; 
    uint16_t ground_speed; 
    uint16_t num_satellite;

    /**
     * @brief Constructor 
     */
    VehicleNavigation();

    /**
     * @brief Destructor 
     */
    ~VehicleNavigation() = default;

    // Navigation data handling 
    void OrientationUpdate(Vehicle &vehicle);
    void LocationUpdate(Vehicle &vehicle);
    
    // Mission execution 
    void TargetAssess(Vehicle &vehicle); 
    void CourseCorrection(Vehicle &vehicle); 

    // Getters 
    Location LocationCurrentGet(void); 
    Vector<float> AccelCurrentGet(void); 
    Vector<float> GyroCurrentGet(void); 
    Vector<float> MagCurrentGet(void); 
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

private:

    enum HeadingDirections : int16_t 
    {
        HEADING_NORTH = 0,      // Heading reading when facing North (0 deg*10) 
        HEADING_SOUTH = 1800,   // Heading reading when facing South (180 deg*10) 
        HEADING_RANGE = 3600    // Full heading range (360 deg*10) 
    };

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

    // Orientation 
    Vector<float> accel, gyro, mag;                      // Body frame accelerometer, gyroscope and magnetometer data 
    Vector<float> orient;                                // x = roll, y = pitch, z = yaw (radians) from NED frame 
    Vector<float> accel_ned;                             // Acceleration in the NED frame 
    Vector<float> mag_hi, mag_sid, mag_sio;              // Magnetometer calibration correction 
    int16_t true_north_offset;                           // Magnetic declination (degrees*10) 
    int16_t heading, heading_target;                     // 0-3599 (degrees*10) 
    float q0, q1, q2, q3;	                             // Madgwick quaternion of sensor frame relative to Earth frame 
    float beta, dt;                                      // Madgwick parameters - correction weight and sample period (s) 
    float r11, r12, r13, r21, r22, r23, r31, r32, r33;   // Madgwick quaternion rotation matrix elements 

    // Location 
    Location location_current, location_filtered, location_target;   // WGS84 
    mavlink_mission_item_int_t mission_target; 
    float waypoint_distance, waypoint_radius;                        // Distance to target waypoint & waypoint acceptance 
    float coordinate_lpf_gain;                                       // Low pass filter gain for GPS coordinates 

    // Navigation data handling 
    void OrientationChecks(Vehicle &vehicle);
    void LocationChecks(Vehicle &vehicle);
    
    // Mission execution 
    void TargetUpdate(Vehicle &vehicle); 

    // Orientation calculations 
    void MagnetometerCorrection(void);
    void OrientationCalcs(void);
    void MadgwickCalcs(void);
    void OrientationNED(void);
    void AccelNED(void);
    int16_t HeadingError(void);
    
    // Position calculations 
    void TargetWaypoint(Vehicle &vehicle); 
    void WaypointError(void);

    // Helper functions 
    float invSqrt(const float &x) const;
}; 

//=======================================================================================

#endif   // _NAVIGATION_H_ 
