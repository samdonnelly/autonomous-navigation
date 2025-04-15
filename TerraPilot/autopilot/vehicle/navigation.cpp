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

#define EARTH_RADIUS 6371         // Average radius of the Earth (km) 
#define HEADING_DIFF_MAX 1800     // Maximum heading difference (+/- 180 degrees * 10) 
#define HEADING_RANGE 3600        // Full heading range (360 degrees * 10) 

#define PI 3.141592                    // PI 
#define RAD_TO_DEG 180.0 / PI          // Radians to degrees conversion 
#define DEG_TO_RAD PI / 180.0          // Degrees to radians conversion 

#define KM_TO_M 1000    // Kilometers to meters 

#define SCALE_10 10 

//=======================================================================================


//=======================================================================================
// Initialization 

VehicleNavigation::VehicleNavigation()
{
    // 
}

//=======================================================================================


//=======================================================================================
// Position 

// Get the current vehicle location 
VehicleNavigation::Location VehicleNavigation::LocationCurrentGet(void)
{
    return current; 
}

//=======================================================================================


//=======================================================================================
// Calculations 

// Coordinate filter 
void VehicleNavigation::CoordinateFilter(
    Location new_data, 
    Location &filtered_data) const
{
    filtered_data.lat += (new_data.lat - filtered_data.lat)*coordinate_lpf_gain; 
    filtered_data.lon += (new_data.lon - filtered_data.lon)*coordinate_lpf_gain; 
}


// GPS radius calculation 
int32_t VehicleNavigation::GPSRadius(
    Location current, 
    Location target)
{
    double eq0, eq1, eq2, eq3, eq4, eq5, eq6, eq7, surf_dist; 

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
    
    return (int32_t)(surf_dist*SCALE_10); 
}


// GPS heading calculation 
int16_t VehicleNavigation::GPSHeading(
    Location current, 
    Location target)
{
    int16_t heading; 
    double eq0, eq1, num, den; 

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

    heading = (int16_t)(atan(num/den)*RAD_TO_DEG*SCALE_10); 

    // Correct the calculated heading if needed. atan can produce a heading outside the 
    // needed range (0-359.9 degrees) so this correction brings the value back within range. 
    if (den < 0)
    {
        heading += HEADING_DIFF_MAX; 
    }
    else if (num < 0)
    {
        heading += HEADING_RANGE; 
    }

    return heading; 
}


// True north heading 
int16_t VehicleNavigation::TrueNorthHeading(int16_t heading) const
{
    // Use the current heading and true north correction offset to get the true north 
    // heading. If the true north heading exceeds acceptable heading bounds (0-359.9deg 
    // or 0-3599 scaled), then shift the heading to be back within bounds. This can be 
    // done because of the circular nature of the heading (ex. 0 degrees is the same 
    // direction as 360 degrees). The returned heading is in degrees*10. 

    int16_t tn_heading = heading + true_north_offset; 

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


// Heading error 
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
