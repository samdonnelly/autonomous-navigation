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
// Macros 

// Size and range 
#define EARTH_RADIUS 6371        // Average radius of the Earth (km) 

// Unit conversions 
#define PI 3.1415927f            // PI 
#define RAD_TO_DEG 180.0f / PI   // Radians to degrees 
#define DEG_TO_RAD PI / 180.0f   // Degrees to radians 
#define KM_TO_M 1000             // Kilometers to meters 
#define SCALE_10 10              // Scaling to remove 1 decimal place 

// Other 
#define MISSION_TARGET_INC 1     // Mission target increment 

//=======================================================================================


//=======================================================================================
// Initialization 

VehicleNavigation::VehicleNavigation()
    : waypoint_distance(RESET), 
      coordinate_lpf_gain(0.5),   // GPS coordinate low pass filter gain 
      heading(RESET), 
      heading_target(RESET), 
      true_north_offset(RESET) 
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
        vehicle.hardware.IMUGet(accel, gyro, mag, heading); 
        xSemaphoreGive(vehicle.comms_mutex); 

        status.imu_connected = FLAG_SET; 
        timers.imu_connection = RESET; 
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
            vehicle.memory.MissionTargetSet(mission_target.seq + MISSION_TARGET_INC); 
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
    location_target.alt = mission_target.z; 
    location_target.altI = (int32_t)(mission_target.z * DEGREE_DATATYPE); 
    location_target.lat = (float)mission_target.x / DEGREE_DATATYPE; 
    location_target.lon = (float)mission_target.y / DEGREE_DATATYPE; 
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
    // stopped as autonomous navigation would not work otherwise 
    (status.gps_connected && status.imu_connected) ? 
        vehicle.AutoDrive(HeadingError(mag)) : 
        vehicle.control.ForceStop(vehicle); 
}

//=======================================================================================


//=======================================================================================
// Location calculations 

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
    if (waypoint_distance < VS_WAYPOINT_RADIUS)
    {
        // Send a mission item reached message 
        vehicle.telemetry.MAVLinkMissionItemReachedSet(); 

        if (mission_target.autocontinue)
        {
            vehicle.memory.MissionTargetSet(mission_target.seq + MISSION_TARGET_INC); 
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
    current_lat= location_current.lat * DEG_TO_RAD; 
    current_lon = location_current.lon * DEG_TO_RAD; 
    target_lat = location_target.lat * DEG_TO_RAD; 
    target_lon = location_target.lon * DEG_TO_RAD; 

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
    heading_target = (int16_t)(atan(num/den)*RAD_TO_DEG*SCALE_10); 
    
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
    waypoint_distance = atan2(sqrt((eq8*eq8) + (eq5*eq5)), (eq4*eq3 + eq2*eq6))*EARTH_RADIUS*KM_TO_M; 
}

//=======================================================================================


//=======================================================================================
// Heading calculations 

/**
 * @brief Magnetic North Heading 
 * 
 * @param magnetometer : magnetometer axis data 
 * @return int16_t : heading (degrees*10) 
 */
int16_t VehicleNavigation::HeadingError(Vector<int16_t> &mag_axis)
{
    // Find the magnetic heading based on the magnetometer X and Y axis data. atan2f 
    // looks at the value and sign of X and Y to determine the correct output so axis 
    // values don't have to be checked for potential errors (ex. divide by zero). 
    int16_t mag_heading = (int16_t)(atan2f((float)mag_axis.y, (float)mag_axis.x)*RAD_TO_DEG*SCALE_10); 

    // Find the true North heading by adding the true North heading offset to the 
    // magnetic heading. After this is done, the bounds are checked to make sure the 
    // offset didn't put the heading value outside its acceptable range. 
    heading = mag_heading + true_north_offset; 
    HeadingRangeCheck(heading); 

    // Adjust the heading range. The magnetic heading is calculated within the range 
    // -180 to 180 degrees and the true North offset adjustment maintains this range. 
    // However, it's easier to find the heading error when the heading is in the range 
    // 0 to 359.9 degrees since the target heading is calculated within this range. 
    if (heading < HEADING_NORTH)
    {
        heading += HEADING_RANGE; 
    }

    // Find the error between the target heading and the current true North heading. 
    // The target heading is based on true North and is found using current and target 
    // GPS coordinates. 
    int16_t heading_error = heading_target - heading; 
    HeadingRangeCheck(heading_error); 

    return heading_error; 
}


/**
 * @brief Keep the heading within a valid range 
 * 
 * @param heading_value : heading value to check 
 */
void VehicleNavigation::HeadingRangeCheck(int16_t &heading_value)
{
    if (heading_value > HEADING_SOUTH)
    {
        heading_value -= HEADING_RANGE; 
    }
    else if (heading_value <= -HEADING_SOUTH)
    {
        heading_value += HEADING_RANGE; 
    }
}

//=======================================================================================


//=======================================================================================
// Getters and setters 

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
 * @return VehicleNavigation::Vector<int16_t> : accelerometer axis data 
 */
VehicleNavigation::Vector<int16_t> VehicleNavigation::AccelCurrentGet(void)
{
    return accel; 
}


/**
 * @brief Get the current gyroscope readings 
 * 
 * @return VehicleNavigation::Vector<int16_t> : gyroscope axis data 
 */
VehicleNavigation::Vector<int16_t> VehicleNavigation::GyroCurrentGet(void)
{
    return gyro; 
}


/**
 * @brief Get the current magnetometer readings 
 * 
 * @return VehicleNavigation::Vector<int16_t> : magnetometer axis data 
 */
VehicleNavigation::Vector<int16_t> VehicleNavigation::MagCurrentGet(void)
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
 * @brief Set the true North offset 
 * 
 * @param tn_offset : true North offset value 
 */
void VehicleNavigation::TrueNorthOffsetSet(int16_t tn_offset)
{
    true_north_offset = tn_offset; 
}


/**
 * @brief Get the distance to the target waypoint 
 * 
 * @return uint16_t : waypoint distance (meters*10) 
 */
uint16_t VehicleNavigation::WaypointDistanceGet(void)
{
    return (uint16_t)(waypoint_distance*SCALE_10); 
}

//=======================================================================================
