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

constexpr float coordinate_scalar = 10000000.0f;   // Adds or removes decimal places of lat and lon 
constexpr float altitude_scalar = 1000.0f;         // Adds or removes decimal places of alt 

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

    // Velocity 
    struct Velocity
    {
        float sog, cog, vvel;   // Speed over ground, course over ground, vertical velocity 
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
    Location LocationGet(void);
    Vector<float> AccelGet(void);
    Vector<float> GyroGet(void);
    Vector<float> MagGet(void);
    Vector<float> OrientationGet(void);
    float HeadingGet(void);
    float HeadingTargetGet(void);
    float WaypointDistanceGet(void);
    
    // Setters 
    void AccelUncertaintyXSet(float accel_sx);
    void AccelUncertaintyYSet(float accel_sy);
    void AccelUncertaintyZSet(float accel_sz);
    void MagHardIronXSet(float compass_hix);
    void MagHardIronYSet(float compass_hiy);
    void MagHardIronZSet(float compass_hiz);
    void MagSoftIronDiagonalXSet(float compass_sidx);
    void MagSoftIronDiagonalYSet(float compass_sidy);
    void MagSoftIronDiagonalZSet(float compass_sidz);
    void MagSoftIronOffDiagonalXSet(float compass_siox);
    void MagSoftIronOffDiagonalYSet(float compass_sioy);
    void MagSoftIronOffDiagonalZSet(float compass_sioz);
    void TrueNorthOffsetSet(float compass_tn);
    void MadgwickBetaSet(float madgwick_b);
    void WaypointRadiusSet(float wp_radius);

private:

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
    Vector<float> accel, gyro, mag;                      // Body frame accelerometer, gyroscope and magnetometer sensor data 
    Vector<float> accel_uncertainty;                     // Acceleration uncertainty in sensor frame 
    Vector<float> mag_hi, mag_sid, mag_sio;              // Magnetometer calibration correction 
    float true_north_offset;                             // Magnetic declination (degrees) 
    float q0, q1, q2, q3;	                             // Madgwick quaternion of sensor frame relative to Earth frame 
    float m_beta, m_dt;                                  // Madgwick parameters - correction weight and sample period (s) 
    float r11, r12, r13, r21, r22, r23, r31, r32, r33;   // Madgwick quaternion rotation matrix elements 
    Vector<float> orient;                                // x = roll, y = pitch, z = yaw (radians) from NED frame 
    Vector<float> accel_ned, accel_ned_uncertainty;      // Acceleration and its uncertainty in the NED frame 
    float heading, heading_target;                       // Current and desired heading (0-359.9 degrees) 

    // Location 
    mavlink_mission_item_int_t mission_target;                        // Mission target information 
    Location location_gps, location_gps_uncertainty;                  // Measure GPS location and uncertainty 
    Velocity velocity_gps, velocity_gps_uncertainty;                  // Measure GPS velocity and uncertainty 
    float k_dt;                                                       // Kalman filter prediction/sample interval (s) 
    Vector<float> k_vel, k_pos;                                       // Kalman filter local velocity and position 
    Vector<float> s2_v, s2_p;                                         // Variance in Kalman filter velocity and position 
    Velocity velocity_filtered;                                       // Vehicle velocity 
    Location location_filtered, location_previous, location_target;   // Vehicle location information (WGS84) 
    float waypoint_distance, waypoint_radius;                         // Distance to target waypoint & waypoint acceptance 

    // Navigation data handling 
    void OrientationChecks(Vehicle &vehicle);
    void LocationChecks(Vehicle &vehicle);
    
    // Mission execution 
    void TargetUpdate(Vehicle &vehicle); 

    // Orientation calculations 
    void OrientationCalcs(void);
    void MagnetometerCorrection(void);
    void MadgwickFilter(void);
    void AttitudeNED(void);
    void AccelNED(void);
    void AccelUncertaintyEarth(void);
    void TrueNorthEarthAccel(Vector<float> &accel) const;
    void BodyToEarthAccel(Vector<float> &a_xyz, Vector<float> &a_ned) const;
    float HeadingError(void);
    
    // Position calculations 
    void KalmanPosePredict(void);
    void KalmanPoseUpdate(void);
    void TargetWaypoint(Vehicle &vehicle); 
    void WaypointError(void);

    // Helper functions 
    float invSqrt(const float &x) const;
}; 

//=======================================================================================

#endif   // _NAVIGATION_H_ 
