/**
 * @file control.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Vehicle control 
 * 
 * @version 0.1
 * @date 2025-04-10
 * 
 * @copyright Copyright (c) 2025
 * 
 */

//=======================================================================================
// Includes 

#include "vehicle.h" 

//=======================================================================================


//=======================================================================================
// Initialization 

// Constructor 
VehicleControl::VehicleControl()
{
    timers.rc_connection = RESET; 

    status.rc_connected = FLAG_CLEAR; 

    channels.throttle = PWM_NEUTRAL; 
    channels.roll = PWM_NEUTRAL; 
    channels.pitch = PWM_NEUTRAL; 
    channels.yaw = PWM_NEUTRAL; 
    channels.mode_control = PWM_NEUTRAL; 
    channels.mode = PWM_NEUTRAL; 
}

//=======================================================================================


//=======================================================================================
// RC data decoding 

/**
 * @brief RC data decode 
 * 
 * @param vehicle : vehicle object 
 */
void VehicleControl::DataDecode(Vehicle &vehicle)
{
    // Check if new data is available. If so then get the data. 
    if (vehicle.hardware.data_ready.rc_ready == FLAG_SET)
    {
        vehicle.hardware.data_ready.rc_ready = FLAG_CLEAR; 

        // Get a copy of the data so we don't have to hold the comms mutex throughout the 
        // whole decoding process. 
        xSemaphoreTake(vehicle.comms_mutex, portMAX_DELAY); 
        vehicle.hardware.RCGet(channels); 
        xSemaphoreGive(vehicle.comms_mutex); 

        timers.rc_connection = RESET; 
        status.rc_connected = FLAG_SET; 

        // Only check for a mode command if new data is available. 
        ModeDecode(vehicle); 
    }

    // RC data checks 
    DataChecks(); 
}


/**
 * @brief RC mode decode 
 * 
 * @param vehicle : vehicle object 
 */
void VehicleControl::ModeDecode(Vehicle &vehicle)
{
    // Check for a new mode input from the RC transmitter. 
    if (channels.mode_control > PWM_AUX_HIGH)
    {
        uint8_t rc_mode = RC_MODE3; 

        if (channels.mode < PWM_MAX_MODE2)
        {
            rc_mode = RC_MODE1; 
        }
        else if (channels.mode < PWM_MAX_MODE4)
        {
            rc_mode = RC_MODE2; 
        }

        // Map the provided mode to a state index for the vehicle and attempt to update 
        // the vehicle state using that index. 
        vehicle.MainStateRCModeMap(rc_mode); 
        vehicle.MainStateSelect(rc_mode); 
    }
}


/**
 * @brief RC data checks 
 */
void VehicleControl::DataChecks(void)
{
    // There isn't a universal way to check for a transmitter connection loss across all 
    // receivers and transmitters. The user must make sure the proper failsafes are 
    // enabled for their receiver and transmitter setup. 
    
    // Check for a physical device loss (no data coming in). 
    if (status.rc_connected && (timers.rc_connection++ >= VS_RC_TIMEOUT))
    {
        status.rc_connected = FLAG_CLEAR; 

        channels.throttle = PWM_NEUTRAL; 
        channels.roll = PWM_NEUTRAL; 
        channels.pitch = PWM_NEUTRAL; 
        channels.yaw = PWM_NEUTRAL; 
    }
}

//=======================================================================================


//=======================================================================================
// Manual control 

// Remote control 
void VehicleControl::RemoteControl(Vehicle &vehicle)
{
    if (status.rc_connected)
    {
        vehicle.ManualDrive(channels); 
    }
    else 
    {
        // Vehicle stop 
    }
}


// Vehicle stop 

//=======================================================================================
