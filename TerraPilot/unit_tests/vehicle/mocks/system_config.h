/**
 * @file system_config.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief System configuration - vehicle unit tests 
 * 
 * @version 0.1
 * @date 2025-05-21
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef _SYSTEM_CONFIG_VEHICLE_UTEST_H_ 
#define _SYSTEM_CONFIG_VEHICLE_UTEST_H_ 

//=======================================================================================
// Vehicle Type (VT) 
//=======================================================================================


//=======================================================================================
// Vehicle Hardware (VH) 
//=======================================================================================


//=======================================================================================
// Vehicle Settings (VS) 

// Telemetry ID 
#define VS_SYSTEM_ID 1 
#define VS_SYSTEM_ID_GCS 255 

//==================================================
// Timing 

#define VS_HEARTBEAT_TIMEOUT 100   // Max count before heartbeat timeout 
#define VS_MISSION_TIMEOUT 10      // Max count before mission protocol timeout 
#define VS_MISSION_RESEND 5        // Max tries to resend a mission message 
#define VS_NAV_DEVICE_TIMEOUT 10   // Max count before navigation devices are considered lost 
#define VS_RC_TIMEOUT 10           // Max count before RC comms are considered lost 

// These must match the interval of the timers thread that calls them 
constexpr float vs_madgwick_dt = 0.1f;   // Madgwick filter calculation interval 
constexpr float vs_kalman_dt = 0.1f;     // Position Kalman filter prediction calculation interval 

//==================================================

// Data sizes 
constexpr uint16_t vs_telemetry_buff = 1500;   // Telemetry data buffer size (bytes) 

// Propulsion and steering 
constexpr uint16_t vs_motor_pwm_off = 1500;    // PWM to turn motor(s) off - can vary between motors/ESCs 

// Navigation 
#define VS_MAG_CAL 0                           // Include magnetometer calibration correction 

//==================================================
// To be made into parameters 

constexpr float vs_accel_sx = 0.1f;          // Accelerometer uncertainty along the X-axis 
constexpr float vs_accel_sy = 0.1f;          // Accelerometer uncertainty along the X-axis 
constexpr float vs_accel_sz = 0.1f;          // Accelerometer uncertainty along the X-axis 

constexpr float vs_auto_max_pwm = 1750.0f;   // Max PWM output of motor(s) in autonomous modes 

constexpr float vs_tn_offset = 0.0f;         // Offset between true and magnetic North (magnetic declination) (degrees) 

constexpr float vs_compass_hix = 0.0f;       // Compass X-axis hard-iron offset 
constexpr float vs_compass_hiy = 0.0f;       // Compass Y-axis hard-iron offset 
constexpr float vs_compass_hiz = 0.0f;       // Compass Z-axis hard-iron offset 

constexpr float vs_compass_sidx = 1.0f;      // Compass X-axis soft-iron diagonal correction 
constexpr float vs_compass_sidy = 1.0f;      // Compass Y-axis soft-iron diagonal correction 
constexpr float vs_compass_sidz = 1.0f;      // Compass Z-axis soft-iron diagonal correction 

constexpr float vs_compass_siox = 0.0f;      // Compass X-axis soft-iron off-diagonal correction 
constexpr float vs_compass_sioy = 0.0f;      // Compass Y-axis soft-iron off-diagonal correction 
constexpr float vs_compass_sioz = 0.0f;      // Compass Z-axis soft-iron off-diagonal correction 

constexpr float vs_waypoint_radius = 5.0f;   // Vehicle acceptance distance to waypoint (m) 

constexpr float vs_madgwick_b = 0.5f;         // Madgwick filter weighted correction (beta) 

//==================================================

// The vehicle specific settings below only have an affect when the corresponding vehicle 
// type (VT) is selected. 

// Boat 
#define VS_BOAT_K1 0               // Kinematics 1 - Differential thruster - 2 propellers 
#define VS_BOAT_K2 0               // Kinematics 2 - Rudder - 1 propeller + 1 rudder 

//=======================================================================================

#endif   // _SYSTEM_CONFIG_VEHICLE_UTEST_H_ 
