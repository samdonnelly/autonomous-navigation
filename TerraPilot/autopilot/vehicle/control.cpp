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
// Enums 
//=======================================================================================


//=======================================================================================
// Initialization 

// Constructor 
VehicleControl::VehicleControl()
{
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

        // if ((channels.mode < PWM_MIN) || (channels.mode > PWM_MAX))
        // {
        //     return; 
        // }

        // uint8_t rc_mode = RC_MODE6; 

        // if (channels.mode < PWM_MAX_MODE1)
        // {
        //     rc_mode = RC_MODE1; 
        // }
        // else if (channels.mode < PWM_MAX_MODE2)
        // {
        //     rc_mode = RC_MODE2; 
        // }
        // else if (channels.mode < PWM_MAX_MODE3)
        // {
        //     rc_mode = RC_MODE3; 
        // }
        // else if (channels.mode < PWM_MAX_MODE4)
        // {
        //     rc_mode = RC_MODE4; 
        // }
        // else if (channels.mode < PWM_MAX_MODE5)
        // {
        //     rc_mode = RC_MODE5; 
        // }

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
    // There isn't a clear way to check for transmitter connection loss across all 
    // devices. The user must make sure the proper failsafes are enabled for their 
    // receiver and transmitter setup. 
    
    // Check for a physical device loss 
}

//=======================================================================================
