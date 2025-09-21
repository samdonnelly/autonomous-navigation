/**
 * @file navigation.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Vehicle navigation 
 * 
 * @version 0.1
 * @date 2025-04-03
 * 
 * @copyright Copyright (c) 2025
 * 
 */

//=======================================================================================
// Includes 

#include "vehicle.h" 

//=======================================================================================


//=======================================================================================
// Constants 

// Real world data 
static constexpr float earth_radius = 6371.0f;   // Average radius of the Earth (km) 
static constexpr float gravity = 9.81;           // Gravity (m/s^2) 

// Directions 
static constexpr float 
heading_north = 0.0f,                                         // Heading when facing North 
heading_south_deg = 180.0f,                                   // Heading when facing South (degrees) 
heading_full_range_deg = 360.0f,                              // Range of heading (degrees) 
heading_south_rad = heading_south_deg*deg_to_rad,             // Heading when facing South (radians) 
heading_full_range_rad = heading_full_range_deg*deg_to_rad;   // Range of heading (radians) 

// Other 
static constexpr uint16_t mission_target_increment = 1;
static constexpr float _0_0f = 0.0f, _1_0f = 1.0f, _2_0f = 2.0f;

//=======================================================================================


//=======================================================================================
// Initialization 

VehicleNavigation::VehicleNavigation()
    : timers(),
      status(),
      accel(), gyro(), mag(),
      accel_uncertainty(),
      mag_hi(), mag_sid(), mag_sio(),
      true_north_offset(_0_0f),
      q0(_1_0f), q1(_0_0f), q2(_0_0f), q3(_0_0f),
      m_beta(_0_0f), m_dt(vs_madgwick_dt),
      r11(_0_0f), r12(_0_0f), r13(_0_0f), r21(_0_0f), r22(_0_0f), r23(_0_0f), r31(_0_0f), r32(_0_0f), r33(_0_0f),
      orient(),
      accel_ned(), accel_ned_uncertainty(),
      heading(_0_0f), heading_target(_0_0f),
      location_gps(), location_gps_uncertainty(),
      velocity_gps(), velocity_gps_uncertainty(),
      k_dt(vs_kalman_dt),
      k_vel(), k_pos(),
      s2_v(), s2_p(),
      velocity_filtered(),
      location_filtered(), location_previous(), location_target(),
      waypoint_distance(_0_0f), waypoint_radius(_0_0f)
{
    // The mission target sequence number gets set to its max value so it's guaranteed 
    // to be updated with mission target information when autonomous navigation starts 
    // for the first time since that many mission items are not supported. Autonomous 
    // navigation can't happen without a position lock, and by the time one is obtained, 
    // the code guarantees there will be valid mission data to obtain which is also why 
    // other mission target data doesn't need to be set initially. 
    mission_target.seq = ~RESET; 
}

//=======================================================================================


//=======================================================================================
// Navigation data handling 

/**
 * @brief Update the vehicle orientation data 
 * 
 * @param vehicle : vehicle object 
 */
void VehicleNavigation::OrientationUpdate(Vehicle &vehicle)
{
    if (vehicle.hardware.data_ready.imu_ready == FLAG_SET)
    {
        vehicle.hardware.data_ready.imu_ready = FLAG_CLEAR; 

        xSemaphoreTake(vehicle.comms_mutex, portMAX_DELAY); 
        vehicle.hardware.IMUGet(accel, gyro, mag); 
        xSemaphoreGive(vehicle.comms_mutex); 

        status.imu_connected = FLAG_SET; 
        timers.imu_connection = RESET;

        OrientationCalcs();
    }

    OrientationChecks(vehicle); 
}


/**
 * @brief Orientation data checks 
 * 
 * @param vehicle : vehicle object 
 */
void VehicleNavigation::OrientationChecks(Vehicle &vehicle)
{
    // Check for a physical device loss (no data coming in). 
    if (status.imu_connected && (timers.imu_connection++ >= VS_NAV_DEVICE_TIMEOUT))
    {
        status.imu_connected = FLAG_CLEAR; 
    }
}


/**
 * @brief Update the vehicle location data 
 * 
 * @param vehicle : vehicle object 
 */
void VehicleNavigation::LocationUpdate(Vehicle &vehicle)
{
    if (vehicle.hardware.data_ready.gps_ready == FLAG_SET)
    {
        vehicle.hardware.data_ready.gps_ready = FLAG_CLEAR; 

        xSemaphoreTake(vehicle.comms_mutex, portMAX_DELAY); 
        status.gps_connected = vehicle.hardware.GPSGet(location_gps,
                                                       location_gps_uncertainty,
                                                       velocity_gps,
                                                       velocity_gps_uncertainty); 
        xSemaphoreGive(vehicle.comms_mutex); 

        timers.gps_connection = RESET;

        KalmanPoseUpdate();
    }

    LocationChecks(vehicle); 
}


/**
 * @brief GPS data checks 
 * 
 * @param vehicle : vehicle object 
 */
void VehicleNavigation::LocationChecks(Vehicle &vehicle)
{
    // If the home location has not yet been set and the vehicle has a GPS position lock 
    // then the home locaton is set to the current location. The home location can also 
    // be set using Mission Planner either from a mission upload (home location uploaded 
    // as mission item 0) or a manual home location update command. It's important to 
    // know if Mission Planner has already set the home location so we don't overwrite 
    // it which is done by checking the home location status. 
    if ((vehicle.memory.MissionHomeLocationStatus() == false) && status.gps_connected)
    {
        vehicle.memory.MissionHomeLocationSet(location_gps.latI, 
                                              location_gps.lonI, 
                                              location_gps.alt); 
    }

    // The filtered location is set to the current location as soon as a GPS connection 
    // becomes available after previously not being available. This prevents the filtered 
    // location needing time to become accurate since it will be {0, 0, 0} on startup. 
    if (!status.gps_status_change && status.gps_connected)
    {
        status.gps_status_change = FLAG_SET; 
        location_filtered = location_gps;
        location_previous = location_gps;
    }
    else if (status.gps_status_change && !status.gps_connected)
    {
        status.gps_status_change = FLAG_CLEAR; 
    }

    // A timer is used to detect the physical loss of a GPS device. This is a case where 
    // GPS data is no longer being received, regardless of position lock or not. If no 
    // data has come in after a certain period of time then position lock is manually 
    // removed to prevent the vehicle from operating on old data (since the lack of new 
    // data prevents it from being updated). 
    if (status.gps_connected && (timers.gps_connection++ >= VS_NAV_DEVICE_TIMEOUT))
    {
        status.gps_connected = FLAG_CLEAR; 
    }
}

//=======================================================================================


//=======================================================================================
// Mission execution 

/**
 * @brief Assess the current mission target 
 * 
 * @details When an autonomous mission completes, the mission index will not start over. 
 *          Instead it will stay at the last index value and perform the action of the 
 *          most recent command until the mode, mission or mission index changes. This 
 *          happens due to the mission index not being able to exceed the mission size. 
 * 
 * @param vehicle : vehicle object 
 */
void VehicleNavigation::TargetAssess(Vehicle &vehicle)
{
    // If there's no GPS position then abort. Autonomous navigation requires a GPS 
    // position in order to function correctly. 
    if (!status.gps_connected)
    {
        return; 
    }

    // Update the mission target information if it doesn't match the set target. The 
    // target can be updated via telemetry or item completion which is why it must be 
    // checked. 
    if (mission_target.seq != vehicle.memory.MissionTargetGet())
    {
        TargetUpdate(vehicle); 
    }

    // Execute the mission command. MAVLink supports various mission commands that all 
    // get stored as mission items. Each command requires a different action. 
    switch (mission_target.command)
    {
        case MAV_CMD_NAV_WAYPOINT: 
            TargetWaypoint(vehicle); 
            break; 
        
        default: 
            // If the current mission item is not supported then it must be skipped. 
            vehicle.memory.MissionTargetSet(mission_target.seq + mission_target_increment); 
            break; 
    }
}


/**
 * @brief Mission target update 
 * 
 * @param vehicle : vehicle object 
 */
void VehicleNavigation::TargetUpdate(Vehicle &vehicle)
{
    // mission_target doesn't need to be checked to be a valid item becuase the sequence 
    // used is the current mission target which will always be within the mission size. 
    mission_target = vehicle.memory.MissionItemGet(vehicle.memory.MissionTargetGet()); 

    // Mission items provide global coordinates with latitude and longitude being 
    // integers and altitude being a float. Since location objects store latitude, 
    // longitude and altitude in both integer and float form, the remaining dimensions 
    // must be filled in by conversion from another type. 
    location_target.latI = mission_target.x; 
    location_target.lonI = mission_target.y; 
    location_target.altI = static_cast<int32_t>(mission_target.z * coordinate_scalar);
    location_target.lat = static_cast<float>(mission_target.x) / coordinate_scalar; 
    location_target.lon = static_cast<float>(mission_target.y) / coordinate_scalar; 
    location_target.alt = mission_target.z;
}


/**
 * @brief Adjust the vehicles course/heading 
 * 
 * @param vehicle : vehicle object 
 */
void VehicleNavigation::CourseCorrection(Vehicle &vehicle)
{
    // If the GPS and IMU are both connected then proceed to find the error between the 
    // vehicle heading and the target heading and use that to update the vehicle thrust 
    // and steering. If either the GPS or IMU are not connected then the vehicle is force 
    // stopped as autonomous navigation would not work otherwise. 
    (status.gps_connected && status.imu_connected) ? vehicle.AutoDrive(HeadingError()) : 
                                                     vehicle.control.ForceStop(vehicle); 
}

//=======================================================================================


//=======================================================================================
// Orientation calculations 

/**
 * @brief Determine the vehicle orientation 
 */
void VehicleNavigation::OrientationCalcs(void)
{
#if VS_MAG_CAL
    // Correct the magnetometer axis readings with calibration values. 
    MagnetometerCorrection();
#endif

    // Execute a Madgwick filter with the new IMU data to find the quaternion rotation 
    // matrix. This allows for the orientation and acceleration in the NED frame to be 
    // determined. 
    MadgwickFilter();

    // Find the roll, pitch and yaw in the NED frame relative to magnetic North. 
    AttitudeNED();

    // Find the acceleration and its uncertainty in the Earth frame relative to true North. 
    AccelNED();
    AccelUncertaintyEarth();

    // Run the prediction step of the Kalman filter to estimate vehicle position. 
    KalmanPosePredict();
}


/**
 * @brief Correct the magnetometer data with the calibration values 
 */
void VehicleNavigation::MagnetometerCorrection(void)
{
    // Correct the magnetometer axis readings with calibration values. First the hard-
    // iron offsets are subtracted from the raw axis readings, then soft-iron scale 
    // values are applied using a matrix multiplication. These correction values are 
    // parameters that can be updated. 

    // Hard-iron offsets 
    Vector<float> mag_off = 
    {
        .x = mag.x - mag_hi.x,
        .y = mag.y - mag_hi.y,
        .z = mag.z - mag_hi.z
    };

    // Soft-iron offsets 
    mag.x = (mag_sid.x*mag_off.x) + (mag_sio.x*mag_off.y) + (mag_sio.y*mag_off.z); 
    mag.y = (mag_sio.x*mag_off.x) + (mag_sid.y*mag_off.y) + (mag_sio.z*mag_off.z); 
    mag.z = (mag_sio.y*mag_off.x) + (mag_sio.z*mag_off.y) + (mag_sid.z*mag_off.z); 
}


/**
 * @brief Determine the vehicle orientation using a Madgwick filter 
 * 
 * @details Uses the new IMU data and a Madgwick filter to determine the orientation and 
 *          acceleration of the vehicle in the Earth frame. Orientation is used for 
 *          vehicle stability and acceleration is used for position estimation. 
 */
void VehicleNavigation::MadgwickFilter(void)
{
    // Madgwick filter calculation variables 
    float gx, gy, gz, ax, ay, az, mx, my, mz;
    float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, 
          _2bx, _2bz, _4bx, _4bz, 
          _2q0, _2q1, _2q2, _2q3, 
          _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    constexpr float _0_5f = 0.5f, _4_0f = 4.0f; 

	// Convert gyroscope degrees/sec to radians/sec
	gx = gyro.x * deg_to_rad;
	gy = gyro.y * deg_to_rad;
	gz = gyro.z * deg_to_rad;

	// Rate of change of quaternion from gyroscope
	qDot1 = _0_5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = _0_5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = _0_5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = _0_5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer and magnetometer measurements are valid 
    // (avoids NaN in normalisation). 
	if (!((accel.x == _0_0f) && (accel.y == _0_0f) && (accel.z == _0_0f)) && 
        !((mag.x == _0_0f) && (mag.y == _0_0f) && (mag.z == _0_0f)))
    {
		// Normalise accelerometer measurement
		recipNorm = invSqrt(accel.x * accel.x + 
                            accel.y * accel.y + 
                            accel.z * accel.z);
		ax = accel.x * recipNorm;
		ay = accel.y * recipNorm;
		az = accel.z * recipNorm;

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mag.x * mag.x + 
                            mag.y * mag.y + 
                            mag.z * mag.z);
		mx = mag.x * recipNorm;
		my = mag.y * recipNorm;
		mz = mag.z * recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = _2_0f * q0 * mx;
		_2q0my = _2_0f * q0 * my;
		_2q0mz = _2_0f * q0 * mz;
		_2q1mx = _2_0f * q1 * mx;
		_2q0 = _2_0f * q0;
		_2q1 = _2_0f * q1;
		_2q2 = _2_0f * q2;
		_2q3 = _2_0f * q3;
		_2q0q2 = _2_0f * q0 * q2;
		_2q2q3 = _2_0f * q2 * q3;
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
		_2bx = sqrtf(hx * hx + hy * hy);
		_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
		_4bx = _2_0f * _2bx;
		_4bz = _2_0f * _2bz;

        // Gradient decent algorithm corrective step
		s0 = -_2q2 * (_2_0f * q1q3 - _2q0q2 - ax) + _2q1 * (_2_0f * q0q1 + _2q2q3 - ay) - 
             _2bz * q2 * (_2bx * (_0_5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + 
             (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + 
             _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (_0_5f - q1q1 - q2q2) - mz);
		
        s1 = _2q3 * (_2_0f * q1q3 - _2q0q2 - ax) + _2q0 * (_2_0f * q0q1 + _2q2q3 - ay) - 
             _4_0f * q1 * (_1_0f - _2_0f * q1q1 - _2_0f * q2q2 - az) + _2bz * q3 * 
             (_2bx * (_0_5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + 
             (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + 
             (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (_0_5f - q1q1 - q2q2) - mz);
		
        s2 = -_2q0 * (_2_0f * q1q3 - _2q0q2 - ax) + _2q3 * (_2_0f * q0q1 + _2q2q3 - ay) - 
             _4_0f * q2 * (_1_0f - _2_0f * q1q1 - _2_0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * 
             (_2bx * (_0_5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * 
             (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * 
             (_2bx * (q0q2 + q1q3) + _2bz * (_0_5f - q1q1 - q2q2) - mz);
		
        s3 = _2q1 * (_2_0f * q1q3 - _2q0q2 - ax) + _2q2 * (_2_0f * q0q1 + _2q2q3 - ay) + 
             (-_4bx * q3 + _2bz * q1) * (_2bx * (_0_5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + 
             (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * 
             q1 * (_2bx * (q0q2 + q1q3) + _2bz * (_0_5f - q1q1 - q2q2) - mz);
		
        // Normalise step magnitude
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= m_beta * s0;
		qDot2 -= m_beta * s1;
		qDot3 -= m_beta * s2;
		qDot4 -= m_beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * m_dt;
	q1 += qDot2 * m_dt;
	q2 += qDot3 * m_dt;
	q3 += qDot4 * m_dt;

	// Normalise quaternion
	recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	// Find the elements of the rotation matrix from body to NWU frame 
	r11 = _0_5f - q2*q2 - q3*q3;
	r12 = q1*q2 - q0*q3;
	r13 = q1*q3 + q0*q2;
	r21 = q1*q2 + q0*q3;
	r22 = _0_5f - q1*q1 - q3*q3;
	r23 = q2*q3 - q0*q1;
	r31 = q1*q3 - q0*q2;
	r32 = q0*q1 + q2*q3;
	r33 = _0_5f - q1*q1 - q2*q2;
}


/**
 * @brief Find the vehicle orientation in the NED frame 
 */
void VehicleNavigation::AttitudeNED(void)
{
    // Calculate roll, pitch and yaw (radians) in the NED frame relative to magnetic 
    // North. 
    orient.x = atan2f(r32, r33);     // Roll 
	orient.y = -asinf(-_2_0f*r31);   // Pitch 
	orient.z = -atan2f(r21, r11);    // Yaw 

    // Adjust the yaw angle to be relative to true North while still being within a valid 
    // range. Roll and pitch are unaffected by this offset. 
    orient.z += true_north_offset;

    if (orient.z <= -heading_south_rad)
    {
        orient.z += heading_full_range_rad;
    }
    else if (orient.z > heading_south_rad)
    {
        orient.z -= heading_full_range_rad;
    }

    // Record of a second version of the yaw angle but in the form of the vehicle heading 
    // which is used for navigation calculations. 
    heading = orient.z*rad_to_deg;

    if (heading < heading_north)
    {
        heading += heading_full_range_deg;
    }
}


/**
 * @brief Find the acceleration of the vehicle in the NED frame 
 */
void VehicleNavigation::AccelNED(void)
{
    // Determine the acceleration (g's) in the NED frame relative to magnetic North and 
    // eliminate gravity from the vertical direction. Then adjust for magnetic 
    // declination so that it becomes relative to true North. 
    BodyToEarthAccel(accel, accel_ned);
    accel_ned.y = -accel_ned.y;
    accel_ned.z = -(accel_ned.z - gravity);
    TrueNorthEarthAccel(accel_ned);
}


/**
 * @brief Find the accelerometer uncertainty in the Earth frame 
 */
void VehicleNavigation::AccelUncertaintyEarth(void)
{
    // Determine the acceleration uncertainty (g's) in the Earth frame relative to 
    // magnetic North. Adjust for magnetic declination so that it becomes relative to 
    // true North then make sure each value is positive so it can be used to predict 
    // position estimation weights (Kalman filter). 
    BodyToEarthAccel(accel_uncertainty, accel_ned_uncertainty);
    TrueNorthEarthAccel(accel_ned_uncertainty);

    if (accel_ned_uncertainty.x < _0_0f)
    {
        accel_ned_uncertainty.x = -accel_ned_uncertainty.x;
    }
    if (accel_ned_uncertainty.y < _0_0f)
    {
        accel_ned_uncertainty.y = -accel_ned_uncertainty.y;
    }
    if (accel_ned_uncertainty.z < _0_0f)
    {
        accel_ned_uncertainty.z = -accel_ned_uncertainty.z;
    }
}


/**
 * @brief Body frame to Earth frame rotation using Madgwick quaternion 
 * 
 * @param a_xyz : body/sensor frame acceleration data (g's) 
 * @param a_ned : calculated Earth frame acceleration (g's) 
 */
void VehicleNavigation::BodyToEarthAccel(
    Vector<float> &a_xyz,
    Vector<float> &a_ned) const
{
    a_ned.x = _2_0f*(r11*a_xyz.x + r12*a_xyz.y + r13*a_xyz.z);
	a_ned.y = _2_0f*(r21*a_xyz.x + r22*a_xyz.y + r23*a_xyz.z);
	a_ned.z = _2_0f*(r31*a_xyz.x + r32*a_xyz.y + r33*a_xyz.z);
}


/**
 * @brief True North Earth frame acceleration 
 * 
 * @param acceleration : acceleration (g's) vector to rotate 
 */
void VehicleNavigation::TrueNorthEarthAccel(Vector<float> &acceleration) const
{
    // Rotate the Earth frame acceleration relative to magnetic North using a 2D rotation 
    // matrix so that it's relative to true North (magnetic declination correction). 
    const float
    eq0 = cosf(true_north_offset),
    eq1 = sinf(true_north_offset),
    eq2 = acceleration.x*eq0,
    eq3 = acceleration.y*eq1,
    eq4 = acceleration.x*eq1,
    eq5 = acceleration.y*eq0;

    acceleration.x = eq2 - eq3;
    acceleration.y = eq4 + eq5;
}


/**
 * @brief Error between current and target heading (relative to true North) 
 * 
 * @return float : heading error (degrees) 
 */
float VehicleNavigation::HeadingError(void)
{
    // Find the error between the target heading and the current true North heading. 
    // The target heading is based on true North and is found using current and target 
    // GPS coordinates. 
    float heading_error = heading_target - heading;

    if (heading_error > heading_south_deg)
    {
        heading_error -= heading_full_range_deg; 
    }
    else if (heading_error <= -heading_south_deg)
    {
        heading_error += heading_full_range_deg; 
    }

    return heading_error; 
}

//=======================================================================================


//=======================================================================================
// Position calculations 

/**
 * @brief Pose estimation Kalman filter prediction step 
 */
void VehicleNavigation::KalmanPosePredict(void)
{
    const float dt_2 = k_dt*k_dt;
    const float accel_const = 0.5f*dt_2;
    constexpr float coordinate_const = earth_radius*km_to_m*deg_to_rad;

    // Predict the new state by finding the local change in position and velocity (no 
    // global reference) using the new NED frame acceleration data. 
    const Vector<float> accel_ned_si = 
    {
        .x = accel_ned.x*gravity,   // North 
        .y = accel_ned.y*gravity,   // East 
        .z = accel_ned.z*gravity    // Down 
    };
    k_vel.x = k_vel.x + k_dt*accel_ned_si.x;
    k_pos.x = k_pos.x + k_vel.x*k_dt + accel_const*accel_ned_si.x;
    k_vel.y = k_vel.y + k_dt*accel_ned_si.y;
    k_pos.y = k_pos.y + k_vel.y*k_dt + accel_const*accel_ned_si.y;
    k_vel.z = k_vel.z + k_dt*accel_ned_si.z;
    k_pos.z = k_pos.z + k_vel.z*k_dt + accel_const*accel_ned_si.z;
    
    // Convert the local changes to a new global position using the last known position 
    // as reference. 
    location_filtered.lat = location_previous.lat + k_pos.x / coordinate_const;
    location_filtered.lon = location_previous.lon + k_pos.y / (coordinate_const*cosf(location_filtered.lat*deg_to_rad));
    location_filtered.alt = location_previous.alt + k_pos.z;

    // Convert the local changes in velocity to a more usable form. 
    velocity_filtered.sog = sqrtf(k_vel.x*k_vel.x + k_vel.y*k_vel.y);
    velocity_filtered.vvel = k_vel.z;

    // Predict the new uncertainty. This must be done each prediction step to account for 
    // error accumulation. In the Kalman filter equations, the uncertainty is calculated 
    // using the variance in the sensor measurements which is the square of the standard 
    // deviation. These calculations use uncertainty in the same format/units as the 
    // acceleration (i.e. not squared) and velocity and position uncertainty are 
    // calculated the same way as the predicted velocity and position are from above. 
    // This is done to simplify the calculation, and it still produces good results 
    // because accelerometer uncertainty is hard to determine so empirical tuning is 
    // often required anyway. The GPS uncertainty measurement is in the same units. 
    const Vector<float> accel_ned_uncertainty_si = 
    {
        .x = accel_ned_uncertainty.x*gravity,
        .y = accel_ned_uncertainty.y*gravity,
        .z = accel_ned_uncertainty.z*gravity
    };
    s2_v.x = s2_v.x + k_dt*accel_ned_uncertainty_si.x;
    s2_p.x = s2_p.x + s2_v.x*k_dt + accel_const*accel_ned_uncertainty_si.x;
    s2_v.y = s2_v.y + k_dt*accel_ned_uncertainty_si.y;
    s2_p.y = s2_p.y + s2_v.y*k_dt + accel_const*accel_ned_uncertainty_si.y;
    s2_v.z = s2_v.z + k_dt*accel_ned_uncertainty_si.z;
    s2_p.z = s2_p.z + s2_v.z*k_dt + accel_const*accel_ned_uncertainty_si.z;
}


/**
 * @brief Pose estimation Kalman filter update step 
 */
void VehicleNavigation::KalmanPoseUpdate(void)
{
    const float cog = velocity_gps.cog*deg_to_rad;

    // Find the Kalman gains for position and velocity in each NED axis. 
    const float
    K_N11 = s2_p.x / (s2_p.x + location_gps_uncertainty.lat),
    K_N22 = s2_v.x / (s2_v.x + velocity_gps_uncertainty.sog),
    K_E11 = s2_p.y / (s2_p.y + location_gps_uncertainty.lon),
    K_E22 = s2_v.y / (s2_v.y + velocity_gps_uncertainty.sog),
    K_D11 = s2_p.z / (s2_p.z + location_gps_uncertainty.alt),
    K_D22 = s2_v.z / (s2_v.z + velocity_gps_uncertainty.vvel);

    // Update the uncertainty of the estimation. 
    s2_p.x = (_1_0f - K_N11)*s2_p.x;
    s2_v.x = (_1_0f - K_N22)*s2_v.x;
    s2_p.y = (_1_0f - K_E11)*s2_p.y;
    s2_v.y = (_1_0f - K_E22)*s2_v.y;
    s2_p.z = (_1_0f - K_D11)*s2_p.z;
    s2_v.z = (_1_0f - K_D22)*s2_v.z;

    // Update the state position 
    location_filtered.lat = location_filtered.lat + (location_gps.lat - location_filtered.lat)*K_N11;
    location_filtered.lon = location_filtered.lon + (location_gps.lon - location_filtered.lon)*K_E11;
    location_filtered.alt = location_filtered.alt + (location_gps.alt - location_filtered.alt)*K_D11;
    location_filtered.latI = static_cast<int32_t>(location_filtered.lat * coordinate_scalar);
    location_filtered.lonI = static_cast<int32_t>(location_filtered.lon * coordinate_scalar);
    location_filtered.altI = static_cast<int32_t>(location_filtered.alt * altitude_scalar);
    
    // Update the state velocity. The local velocity for each axis is updated so velocity 
    // can continue to be estimated during the prediction step. The determined velocity 
    // is then converted to its global format for the user to fetch. 
    k_vel.x = k_vel.x + (velocity_gps.sog*cosf(cog) - k_vel.x)*K_N22;
    k_vel.y = k_vel.y + (velocity_gps.sog*sinf(cog) - k_vel.y)*K_E22;
    k_vel.z = k_vel.z + (velocity_gps.vvel - k_vel.z)*K_D22;
    velocity_filtered.sog = sqrtf(k_vel.x*k_vel.x + k_vel.y*k_vel.y);
    velocity_filtered.vvel = k_vel.z;

    // Save the global position so the prediction step can add local position estimates 
    // to it to estimate global position. Reset the local position estimates as the 
    // distance from last known position is now zero. 
    location_previous = location_filtered;
    k_pos = { _0_0f, _0_0f, _0_0f };
}


/**
 * @brief Find the position relative to the target waypoint 
 * 
 * @param vehicle : vehicle object 
 */
void VehicleNavigation::TargetWaypoint(Vehicle &vehicle)
{
    // Check if the current location is within the acceptance radius of the target 
    // waypoint. If it is and the mission item specifies to continue onto the next 
    // mission item, then the mission target is updated. 

    // Update the GPS position information before checking if the waypoint is hit. 
    WaypointError(); 
    
    // Check if the waypoint had been hit by comparing the distance to the waypoint 
    // with the waypoint acceptance distance. 
    if (waypoint_distance < waypoint_radius)
    {
        // Send a mission item reached message 
        vehicle.telemetry.MAVLinkMissionItemReachedSet(); 

        if (mission_target.autocontinue)
        {
            vehicle.memory.MissionTargetSet(mission_target.seq + mission_target_increment); 
        }
    }
}


/**
 * @brief Find the distance and heading to the target waypoint 
 */
void VehicleNavigation::WaypointError(void)
{
    float current_lat, current_lon, target_lat, target_lon;
    float eq0, eq1, eq2, eq3, eq4, eq5, eq6, eq7, eq8;

    // Convert the coordinates to radians so they're compatible with the math library. 
    current_lat = location_filtered.lat*deg_to_rad;
    current_lon = location_filtered.lon*deg_to_rad;
    target_lat = location_target.lat*deg_to_rad;
    target_lon = location_target.lon*deg_to_rad;

    // Break the calculations down into smaller chunks to make it more readable. This 
    // also prevents from the same calculations from being done more than once. 
    eq0 = target_lon - current_lon;
    eq1 = cos(target_lat);
    eq2 = cos(current_lat);
    eq3 = sin(target_lat);
    eq4 = sin(current_lat);
    eq5 = eq1*sin(eq0);
    eq6 = eq1*cos(eq0);
    eq7 = eq2*eq3;
    eq8 = eq7 - eq4*eq6;
    
    // Calculate the initial heading in radians between coordinates relative to true 
    // North. As you move along the path that's the shortest distance between two points 
    // on the globe, your heading relative to true North changes which is why this is 
    // just the instantaneous heading. This calculation comes from the great-circle 
    // navigation equations. Once the heading is found it's converted from radians to 
    // degrees and scaled by 10 (units: degrees*10) so its integer representation can 
    // hold one decimal place of accuracy. 
    heading_target = atan(eq5/eq8)*rad_to_deg;
    
    // Correct the calculated heading if needed. atan can produce a heading outside the 
    // needed range (0-359.9 degrees) so this correction brings the value back within 
    // range. 
    if (eq8 < 0)
    {
        heading_target += heading_south_deg;
    }
    else if (eq5 < 0)
    {
        heading_target += heading_full_range_deg;
    }

    // Calculate the surface distance (or radius - direction independent) in meters 
    // between the current and target coordinates. This distance can also be described as 
    // the distance between coordinates along their great-circle. This calculation comes 
    // from the great-circle navigation equations. 
    waypoint_distance = atan2(sqrt((eq8*eq8) + (eq5*eq5)), (eq4*eq3 + eq2*eq6))*earth_radius*km_to_m;
}

//=======================================================================================


//=======================================================================================
// Getters 

/**
 * @brief Get the vehicle location 
 * 
 * @return VehicleNavigation::Location : location data 
 */
VehicleNavigation::Location VehicleNavigation::LocationGet(void)
{
    return location_filtered; 
}


/**
 * @brief Get the accelerometer readings 
 * 
 * @return VehicleNavigation::Vector<float> : accelerometer axis data 
 */
VehicleNavigation::Vector<float> VehicleNavigation::AccelGet(void)
{
    return accel;
}


/**
 * @brief Get the gyroscope readings 
 * 
 * @return VehicleNavigation::Vector<float> : gyroscope axis data 
 */
VehicleNavigation::Vector<float> VehicleNavigation::GyroGet(void)
{
    return gyro;
}


/**
 * @brief Get the magnetometer readings 
 * 
 * @return VehicleNavigation::Vector<float> : magnetometer axis data 
 */
VehicleNavigation::Vector<float> VehicleNavigation::MagGet(void)
{
    return mag;
}


/**
 * @brief Get the orientation (roll, pitch, yaw) 
 * 
 * @return VehicleNavigation::Vector<float> : vehicle orientation (degrees) 
 */
VehicleNavigation::Vector<float> VehicleNavigation::OrientationGet(void)
{
    return orient; 
}


/**
 * @brief Get the vehicle heading 
 * 
 * @return float : heading (0-359.9 degrees) 
 */
float VehicleNavigation::HeadingGet(void)
{
    return heading; 
}


/**
 * @brief Get the target heading 
 * 
 * @return float : target heading (0-359.9 degrees) 
 */
float VehicleNavigation::HeadingTargetGet(void)
{
    return heading_target; 
}


/**
 * @brief Get the distance to the target waypoint 
 * 
 * @return float : waypoint distance (meters) 
 */
float VehicleNavigation::WaypointDistanceGet(void)
{
    return waypoint_distance; 
}

//=======================================================================================


//=======================================================================================
// Setters 

/**
 * @brief Set the accelerometer uncertainty for the X-axis 
 * 
 * @param accel_sx : X-axis hard-iron offset value (milligauss)
 */
void VehicleNavigation::AccelUncertaintyXSet(float accel_sx)
{
    accel_uncertainty.x = accel_sx; 
}


/**
 * @brief Set the accelerometer uncertainty for the Y-axis 
 * 
 * @param accel_sy : Y-axis hard-iron offset value (milligauss)
 */
void VehicleNavigation::AccelUncertaintyYSet(float accel_sy)
{
    accel_uncertainty.y = accel_sy; 
}


/**
 * @brief Set the accelerometer uncertainty for the Z-axis 
 * 
 * @param accel_sz : Z-axis hard-iron offset value (milligauss)
 */
void VehicleNavigation::AccelUncertaintyZSet(float accel_sz)
{
    accel_uncertainty.z = accel_sz; 
}


/**
 * @brief Set the magnetometer X-axis hard-iron offset 
 * 
 * @param compass_hix : X-axis hard-iron offset value (milligauss)
 */
void VehicleNavigation::MagHardIronXSet(float compass_hix)
{
    mag_hi.x = compass_hix; 
}


/**
 * @brief Set the magnetometer Y-axis hard-iron offset 
 * 
 * @param compass_hiy : Y-axis hard-iron offset value (milligauss)
 */
void VehicleNavigation::MagHardIronYSet(float compass_hiy)
{
    mag_hi.y = compass_hiy; 
}


/**
 * @brief Set the magnetometer Z-axis hard-iron offset 
 * 
 * @param compass_hiz : Z-axis hard-iron offset value (milligauss)
 */
void VehicleNavigation::MagHardIronZSet(float compass_hiz)
{
    mag_hi.z = compass_hiz; 
}


/**
 * @brief Set the magnetometer X-axis soft-iron diagonal scale value 
 * 
 * @param compass_sidx : X-axis soft-iron diagonal scale value 
 */
void VehicleNavigation::MagSoftIronDiagonalXSet(float compass_sidx)
{
    mag_sid.x = compass_sidx; 
}


/**
 * @brief Set the magnetometer Y-axis soft-iron diagonal scale value 
 * 
 * @param compass_sidy : Y-axis soft-iron diagonal scale value 
 */
void VehicleNavigation::MagSoftIronDiagonalYSet(float compass_sidy)
{
    mag_sid.y = compass_sidy; 
}


/**
 * @brief Set the magnetometer Z-axis soft-iron diagonal scale value 
 * 
 * @param compass_sidz : Z-axis soft-iron diagonal scale value 
 */
void VehicleNavigation::MagSoftIronDiagonalZSet(float compass_sidz)
{
    mag_sid.z = compass_sidz; 
}


/**
 * @brief Set the magnetometer X-axis soft-iron off-diagonal scale value 
 * 
 * @param compass_siox : X-axis soft-iron off-diagonal scale value 
 */
void VehicleNavigation::MagSoftIronOffDiagonalXSet(float compass_siox)
{
    mag_sio.x = compass_siox; 
}


/**
 * @brief Set the magnetometer Y-axis soft-iron off-diagonal scale value 
 * 
 * @param compass_sioy : Y-axis soft-iron off-diagonal scale value 
 */
void VehicleNavigation::MagSoftIronOffDiagonalYSet(float compass_sioy)
{
    mag_sio.y = compass_sioy; 
}


/**
 * @brief Set the magnetometer Z-axis soft-iron off-diagonal scale value 
 * 
 * @param compass_sioz : Z-axis soft-iron off-diagonal scale value 
 */
void VehicleNavigation::MagSoftIronOffDiagonalZSet(float compass_sioz)
{
    mag_sio.z = compass_sioz; 
}


/**
 * @brief Set the true North offset (magnetic declination) 
 * 
 * @param compass_tn : Magnetic declination (degrees) 
 */
void VehicleNavigation::TrueNorthOffsetSet(float compass_tn)
{
    true_north_offset = compass_tn*deg_to_rad; 
}


/**
 * @brief Set the Madgwick filter weighted correction factor (beta) 
 * 
 * @param madgwick_b : Madgwick filter correction (beta) 
 */
void VehicleNavigation::MadgwickBetaSet(float madgwick_b)
{
    m_beta = madgwick_b;
}


/**
 * @brief Set the waypoint acceptance radius 
 * 
 * @param wp_radius : waypoint acceptance radius (meters) 
 */
void VehicleNavigation::WaypointRadiusSet(float wp_radius)
{
    waypoint_radius = wp_radius; 
}

//=======================================================================================


//=======================================================================================
// Helper functions 

// Inverse square root 
float VehicleNavigation::invSqrt(const float &x) const
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;

	i = 0x5f3759df - (i >> 1);
	y = *(float*)&i;
	y *= (1.5f - (halfx * y * y));
	y *= (1.5f - (halfx * y * y));

	return y;
}

//=======================================================================================
