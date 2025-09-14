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

// Unit conversions 
static constexpr float rad_to_deg = 180.0f / 3.1415927f;   // Radians to degrees 
static constexpr float deg_to_rad = 3.1415927f / 180.0f;   // Degrees to radians 
static constexpr float km_to_m = 1000.0f;                  // Kilometers to meters 
static constexpr float scale_10 = 10.0f;                   // Scaling to remove 1 decimal place 

// Directions 
static constexpr float heading_north = 0.0f;               // Heading when facing North 
static constexpr float heading_south = 180.0f;             // Heading when facing South 
static constexpr float heading_full_range = 360.0f;        // Range of heading 

// Other 
static constexpr uint16_t mission_target_increment = 1;

//=======================================================================================


//=======================================================================================
// Initialization 

VehicleNavigation::VehicleNavigation()
    : waypoint_distance(RESET), 
      coordinate_lpf_gain(0.5),   // GPS coordinate low pass filter gain 
      true_north_offset(RESET), 
      heading(RESET), 
      heading_target(RESET) 
{
    timers.gps_connection = RESET; 
    timers.imu_connection = RESET; 

    status.flags = RESET; 

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

#if VS_MAG_CAL
        MagnetometerCorrection();
#endif
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
        status.gps_connected = vehicle.hardware.GPSGet(location_current); 
        xSemaphoreGive(vehicle.comms_mutex); 

        timers.gps_connection = RESET; 
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
        vehicle.memory.MissionHomeLocationSet(location_current.latI, 
                                              location_current.lonI, 
                                              location_current.alt); 
    }

    // The filtered location is set to the current location as soon as a GPS connection 
    // becomes available after previously not being available. This prevents the filtered 
    // location needing time to become accurate since it will be {0, 0, 0} on startup. 
    if (!status.gps_status_change && status.gps_connected)
    {
        status.gps_status_change = FLAG_SET; 
        location_filtered = location_current; 
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
 * @brief Determine the vehicle orientation 
 */
void VehicleNavigation::OrientationCalcs(void)
{
    // Execute a Madgwick filter with the new IMU data to find the quaternion rotation 
    // matrix. This allows for the orientation and acceleration in the NED frame to be 
    // determined. 
    MadgwickCalcs();

    // Find the roll, pitch and yaw in the NED frame relative to magnetic North. 
    OrientationNED();

    // Find the acceleration and its uncertainty in the NED frame relative to true North. 
    AccelNED();
    AccelUncertaintyNED();
}


/**
 * @brief Determine the vehicle orientation using a Madgwick filter 
 * 
 * @details Uses the new IMU data and a Madgwick filter to determine the orientation and 
 *          acceleration of the vehicle in the Earth frame. Orientation is used for 
 *          vehicle stability and acceleration is used for position estimation. 
 */
void VehicleNavigation::MadgwickCalcs(void)
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

    constexpr float _0_0f = 0.0f, _0_5f = 0.5f, _1_0f = 1.0f, _2_0f = 2.0f, _4_0f = 4.0f; 

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
	if (!((accel.x == _0_0f) && (accel.y == _0_0f) && (accel.z == _0_0f)) || 
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
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * dt;
	q1 += qDot2 * dt;
	q2 += qDot3 * dt;
	q3 += qDot4 * dt;

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
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
void VehicleNavigation::OrientationNED(void)
{
    constexpr float _2_0f = 2.0f;

    // Calculate roll, pitch and yaw (radians) in the NED frame relative to magnetic 
    // North. 
	orient.x = atan2f(r32, r33);     // Roll 
	orient.y = -asinf(-_2_0f*r31);   // Pitch 
	orient.z = -atan2f(r21, r11);    // Yaw 

    // Adjust the yaw angle to be relative to true North. Roll and pitch are unaffected 
    // by this offset. 
    orient.z += true_north_offset;

    if (orient.z < heading_north)
    {
        orient.z += heading_full_range;
    }
    else if (orient.z >= heading_full_range)
    {
        orient.z -= heading_full_range;
    }
}


/**
 * @brief Find the acceleration of the vehicle in the NED frame 
 */
void VehicleNavigation::AccelNED(void)
{
    constexpr float _2_0f = 2.0f;

    // Determine the acceleration (g's) in the NED frame relative to magnetic North 
    accel_ned.x = _2_0f*(r11*accel.x + r12*accel.y + r13*accel.z);
	accel_ned.y = -_2_0f*(r21*accel.x + r22*accel.y + r23*accel.z);
	accel_ned.z = -(_2_0f*(r31*accel.x + r32*accel.y + r33*accel.z) - gravity);

    // Rotate the NED acceleration into the true North NED frame using a 2D rotation 
    // matrix about the vertical axis. 
    const float
    eqo = true_north_offset*deg_to_rad,
    eq1 = cosf(eqo),
    eq2 = sinf(eqo),
    eq3 = accel_ned.x*eq1,
    eq4 = accel_ned.y*eq2,
    eq5 = accel_ned.x*eq2,
    eq6 = accel_ned.y*eq1;

    accel_ned.x = eq3 - eq4;
    accel_ned.y = eq5 + eq6;
}


/**
 * @brief Find the accelerometer uncertainty in the NED frame 
 */
void VehicleNavigation::AccelUncertaintyNED(void)
{
    // 
}


/**
 * @brief Error between current and target heading (relative to true North) 
 * 
 * @return int16_t : heading error (degrees*10) 
 */
int16_t VehicleNavigation::HeadingError(void)
{
    // Find the error between the target heading and the current true North heading. 
    // The target heading is based on true North and is found using current and target 
    // GPS coordinates. 
    int16_t heading_error = heading_target - heading;

    if (heading_error > HEADING_SOUTH)
    {
        heading_error -= HEADING_RANGE; 
    }
    else if (heading_error <= -HEADING_SOUTH)
    {
        heading_error += HEADING_RANGE; 
    }

    return heading_error; 
}

//=======================================================================================


//=======================================================================================
// Position calculations 

/**
 * @brief Find the position relative to the target waypoint 
 * 
 * @param vehicle : vehicle object 
 */
void VehicleNavigation::TargetWaypoint(Vehicle &vehicle)
{
    // Check if the current location is within the acceptance radius of the target 
    // waypoint. If it is and the mission item specifies to continue onto the next 
    // mission item, then the mission target is updated. The read coordinates are 
    // filtered to smooth the data. The coordinate/target heading is also found here 
    // since it can only change with new coordinate data. 

    // location_filtered.lat += (location_current.lat - location_filtered.lat)*coordinate_lpf_gain; 
    // location_filtered.lon += (location_current.lon - location_filtered.lon)*coordinate_lpf_gain; 

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
    float num, den; 

    // Convert the coordinates to radians so they're compatible with the math library. 
    current_lat = location_current.lat * deg_to_rad; 
    current_lon = location_current.lon * deg_to_rad; 
    target_lat = location_target.lat * deg_to_rad; 
    target_lon = location_target.lon * deg_to_rad; 

    // Break the calculations down into smaller chunks to make it more readable. This 
    // also prevents from the same calculations from being done twice. 
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
    num = eq1*sin(eq0); 
    den = eq7 - eq4*eq1*cos(eq0); 
    heading_target = (int16_t)(atan(num/den)*rad_to_deg*scale_10); 
    
    // Correct the calculated heading if needed. atan can produce a heading outside the 
    // needed range (0-359.9 degrees) so this correction brings the value back within 
    // range. 
    if (den < 0)
    {
        heading_target += HEADING_SOUTH; 
    }
    else if (num < 0)
    {
        heading_target += HEADING_RANGE; 
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
 * @brief Get the current vehicle location 
 * 
 * @return VehicleNavigation::Location : current location data 
 */
VehicleNavigation::Location VehicleNavigation::LocationCurrentGet(void)
{
    return location_current; 
}


/**
 * @brief Get the current accelerometer readings 
 * 
 * @return VehicleNavigation::Vector<float> : accelerometer axis data 
 */
VehicleNavigation::Vector<float> VehicleNavigation::AccelCurrentGet(void)
{
    return accel;
}


/**
 * @brief Get the current gyroscope readings 
 * 
 * @return VehicleNavigation::Vector<float> : gyroscope axis data 
 */
VehicleNavigation::Vector<float> VehicleNavigation::GyroCurrentGet(void)
{
    return gyro;
}


/**
 * @brief Get the current magnetometer readings 
 * 
 * @return VehicleNavigation::Vector<float> : magnetometer axis data 
 */
VehicleNavigation::Vector<float> VehicleNavigation::MagCurrentGet(void)
{
    return mag;
}


/**
 * @brief Get the current orientation (roll, pitch, yaw) 
 * 
 * @return VehicleNavigation::Vector<float> : vehicle orientation (degrees) 
 */
VehicleNavigation::Vector<float> VehicleNavigation::OrientationCurrentGet(void)
{
    return orient; 
}


/**
 * @brief Get the current vehicle heading 
 * 
 * @return int16_t : current vehicle heading (degrees*10) 
 */
int16_t VehicleNavigation::HeadingCurrentGet(void)
{
    return heading; 
}


/**
 * @brief Get the target vehicle heading 
 * 
 * @return int16_t : target heading (0-3599 degrees*10) 
 */
int16_t VehicleNavigation::HeadingTargetGet(void)
{
    return heading_target; 
}


/**
 * @brief Get the distance to the target waypoint 
 * 
 * @return uint16_t : waypoint distance (meters*10) 
 */
uint16_t VehicleNavigation::WaypointDistanceGet(void)
{
    return (uint16_t)(waypoint_distance*scale_10); 
}

//=======================================================================================


//=======================================================================================
// Setters 

/**
 * @brief Set the true North offset 
 * 
 * @param compass_tn : true North offset value 
 */
void VehicleNavigation::TrueNorthOffsetSet(int16_t compass_tn)
{
    true_north_offset = compass_tn; 
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
