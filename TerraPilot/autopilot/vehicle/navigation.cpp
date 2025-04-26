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
// Notes 

// Home location access points: 
// - Vehicle updates based on first obtained GPS position 
// 	 - Check first to see if the home location has been set. Don't want to overwrite it 
//     if it's already been set. 
// 	 - If it has not been set then set the home location and set the home location flag. 
// - Mission Planner does a mission upload (home location at mission index 0) 
// 	 - Home location has been set 
// 	 - Set the home location flag 
// 	 - Mission Planner won't upload a mission without the home position set locally in 
//     the application. It then 
// 	   proceeds to send the home location as mission item 0. 
// - Mission Planner manually updates the home position 
// 	 - Home location has been set 
// 	 - Set the home location flag 
// - The vehicle loads a mission (home position at index 0) from memory 
// 	 - This can happen on startup and when the system needs to reset (without powering 
//     down) 
// 	 - Home location has not been set, only read from historical data 
// 	 - In this case, we don't want to use the loaded home position as the home location 
//     because it has not been 
// 	   explicity set, only read from recorded data. 
// 	 - Do not set the home location flag. 

// Mission Upload from Mission Planner doesn't have a means to set the home location 
// flag yet. 
// 	- It does now! 

// The filtered location won't be initialized to the first obtained GPS position now that 
// the home location flag is now in the memory module and accessed elsewhere. 
// 	- Fixed by adding a dedicated flag to monitor GPS status changes. 

//=======================================================================================


//=======================================================================================
// Macros 

// Size and range 
#define EARTH_RADIUS 6371        // Average radius of the Earth (km) 
#define HEADING_DIFF_MAX 1800    // Maximum heading difference (+/- 180 degrees * 10) 
#define HEADING_RANGE 3600       // Full heading range (360 degrees * 10) 

// Unit conversions 
#define PI 3.141592f             // PI 
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
{
    timers.gps_connection = RESET; 

    status.flags = RESET; 

    // The mission target sequence number gets set to its max value so it's guaranteed 
    // to be updated with mission target information when autonomous navigation starts 
    // for the first time since that many mission items are not supported. Autonomous 
    // navigation can't happen without a position lock, and by the time one is obtained, 
    // the code guarantees there will be valid mission data to obtain which is also why 
    // other mission target data doesn't need to be set initially. 
    mission_target.seq = ~RESET; 

    coordinate_lpf_gain = 0.5;   // Make parameter? 
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
        vehicle.hardware.IMUGet(accel, gyro, heading); 
        xSemaphoreGive(vehicle.comms_mutex); 

        status.imu_connected = FLAG_SET; 
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
            WaypointDistance(vehicle); 
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
        vehicle.AutoDrive(HeadingError(TrueNorthHeading(heading), heading_target)) : 
        vehicle.control.ForceStop(vehicle); 
}

//=======================================================================================


//=======================================================================================
// Location calculations 

/**
 * @brief Find the distance to the target waypoint 
 * 
 * @param vehicle : vehicle object 
 */
void VehicleNavigation::WaypointDistance(Vehicle &vehicle)
{
    // Check if the current location is within the acceptance radius of the target 
    // waypoint. If it is and the mission item specifies to continue onto the next 
    // mission item, then the mission target is updated. The read coordinates are 
    // filtered to smooth the data. The coordinate/target heading is also found here 
    // since it can only change with new coordinate data. 
    
    CoordinateFilter(location_current, location_filtered); 
    heading_target = GPSHeading(location_filtered, location_target); 
    
    if (GPSRadius(location_filtered, location_target) < mission_target.param2)
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
 * @brief Coordinate filter 
 * 
 * @param raw : new/raw coordinates read by the system 
 * @param filtered : previously filtered coordinates 
 */
void VehicleNavigation::CoordinateFilter(
    Location raw, 
    Location &filtered) const
{
    filtered.lat += (raw.lat - filtered.lat)*coordinate_lpf_gain; 
    filtered.lon += (raw.lon - filtered.lon)*coordinate_lpf_gain; 
}


/**
 * @brief GPS radius calculation 
 * 
 * @param current : current vehicle location 
 * @param target : target location 
 * @return float : surface distance between the current and target location 
 */
// int32_t VehicleNavigation::GPSRadius(
float VehicleNavigation::GPSRadius(
    Location current, 
    Location target)
{
    float eq0, eq1, eq2, eq3, eq4, eq5, eq6, eq7, surf_dist; 

    // Convert the coordinates to radians so they're compatible with the math library. 
    current.lat *= DEG_TO_RAD; 
    current.lon *= DEG_TO_RAD; 
    target.lat *= DEG_TO_RAD; 
    target.lon *= DEG_TO_RAD; 

    // Calculate the surface distance (or radius - direction independent) in meters 
    // between coordinates. This distance can also be described as the distance between 
    // coordinates along their great-circle. This calculation comes from the great-circle 
    // navigation equations. Once the distance is found then the returned value is scaled 
    // by 10 (units: meters*10) so its integer representation can hold one decimal place 
    // of accuracy. Equations are structured to not have repeated calculations. 
    eq0 = (target.lon - current.lon); 
    eq1 = cos(target.lat); 
    eq2 = cos(current.lat); 
    eq3 = sin(target.lat); 
    eq4 = sin(current.lat); 
    eq5 = eq1*sin(eq0); 
    eq6 = eq1*cos(eq0); 
    eq7 = eq2*eq3 - eq4*eq6; 

    surf_dist = atan2(sqrt((eq7*eq7) + (eq5*eq5)), (eq4*eq3 + eq2*eq6))*EARTH_RADIUS*KM_TO_M; 
    
    // return (int32_t)(surf_dist*SCALE_10); 
    return surf_dist; 
}


/**
 * @brief GPS heading calculation 
 * 
 * @param current : current vehicle location 
 * @param target : target location 
 * @return int16_t : heading between the current and target location (degrees*10) 
 */
int16_t VehicleNavigation::GPSHeading(
    Location current, 
    Location target)
{
    int16_t gps_heading; 
    float eq0, eq1, num, den; 

    // Convert the coordinates to radians so they're compatible with the math library. 
    current.lat *= DEG_TO_RAD; 
    current.lon *= DEG_TO_RAD; 
    target.lat *= DEG_TO_RAD; 
    target.lon *= DEG_TO_RAD; 

    // Calculate the initial heading in radians between coordinates relative to true north. 
    // As you move along the path that's the shortest distance between two points on the 
    // globe, your heading relative to true north changes which is why this is just the 
    // instantaneous heading. This calculation comes from the great-circle navigation 
    // equations. Once the heading is found it's converted from radians to degrees and 
    // scaled by 10 (units: degrees*10) so its integer representation can hold one decimal 
    // place of accuracy. 
    eq0 = cos(target.lat); 
    eq1 = (target.lon - current.lon); 
    num = eq0*sin(eq1); 
    den = cos(current.lat)*sin(target.lat) - sin(current.lat)*eq0*cos(eq1); 

    gps_heading = (int16_t)(atan(num/den)*RAD_TO_DEG*SCALE_10); 

    // Correct the calculated heading if needed. atan can produce a heading outside the 
    // needed range (0-359.9 degrees) so this correction brings the value back within range. 
    if (den < 0)
    {
        gps_heading += HEADING_DIFF_MAX; 
    }
    else if (num < 0)
    {
        gps_heading += HEADING_RANGE; 
    }

    return gps_heading; 
}

//=======================================================================================


//=======================================================================================
// Heading calculations 

/**
 * @brief True north heading 
 * 
 * @param magnetic_heading : magnetometer heading (degrees*10) 
 * @return int16_t : true north heading (degrees*10) 
 */
int16_t VehicleNavigation::TrueNorthHeading(int16_t magnetic_heading) const
{
    // Use the current heading and true north correction offset to get the true north 
    // heading. If the true north heading exceeds acceptable heading bounds (0-359.9deg 
    // or 0-3599 scaled), then shift the heading to be back within bounds. This can be 
    // done because of the circular nature of the heading (ex. 0 degrees is the same 
    // direction as 360 degrees). The returned heading is in degrees*10. 

    int16_t tn_heading = magnetic_heading + true_north_offset; 

    if (true_north_offset >= 0)
    {
        if (tn_heading >= HEADING_RANGE)
        {
            tn_heading -= HEADING_RANGE; 
        }
    }
    else 
    {
        if (tn_heading < 0)
        {
            tn_heading += HEADING_RANGE; 
        }
    }

    return tn_heading; 
}


/**
 * @brief Heading error 
 * 
 * @param current_heading : current true north heading (degrees*10) 
 * @param target_heading : desired true north heading (degrees*10) 
 * @return int16_t : difference between current and desired true north headings (degrees*10) 
 */
int16_t VehicleNavigation::HeadingError(
    int16_t current_heading, 
    int16_t target_heading)
{
    // Calculate the heading error and correct it when the heading crosses the 0/360 
    // degree boundary. For example, if the current heading is 10 degrees and the 
    // target heading is 345 degrees, the error will read as 345-10 = 335 degrees. 
    // Although not technically wrong, it makes more sense to say the error is -25 
    // degrees (-(10 + (360-345))) because that is the smaller angle between the two 
    // headings and the negative sign indicates in what direction this smaller error 
    // happens. So instead of turning 335 degrees clockwise, you can turn 25 degrees 
    // counter clockwise to correct for the error. The inflection point of the error 
    // for this correction is 180 degrees (or 1800 in degrees*10). 

    int16_t heading_error = target_heading - current_heading; 

    if (heading_error > HEADING_DIFF_MAX)
    {
        heading_error -= HEADING_RANGE; 
    }
    else if (heading_error <= -HEADING_DIFF_MAX)
    {
        heading_error += HEADING_RANGE; 
    }

    return heading_error; 
}

//=======================================================================================


//=======================================================================================
// Getters and setters 

// Get the current vehicle location 
VehicleNavigation::Location VehicleNavigation::LocationCurrentGet(void)
{
    return location_current; 
}


// Get the current accelerometer readings 
VehicleNavigation::Vector<int16_t> VehicleNavigation::AccelCurrentGet(void)
{
    return accel; 
}


// Get the current gyroscope readings 
VehicleNavigation::Vector<int16_t> VehicleNavigation::GyroCurrentGet(void)
{
    return gyro; 
}


// Get the current magnetometer readings 
VehicleNavigation::Vector<int16_t> VehicleNavigation::MagCurrentGet(void)
{
    return mag; 
}


// Get the current orientation (roll, pitch, yaw) 
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


// Get the target vehicle heading 
int16_t VehicleNavigation::HeadingTargetGet(void)
{
    return heading_target; 
}

//=======================================================================================
