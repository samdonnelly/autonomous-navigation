/**
 * @file boat.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Boat 
 * 
 * @version 0.1
 * @date 2025-02-25
 * 
 * @copyright Copyright (c) 2025
 * 
 */

//=======================================================================================
// Includes 

#include "boat.h" 

//=======================================================================================


//=======================================================================================
// Macros 

#define BOAT_MAX_THRUST 1750U        // Make into a parameter 
#define BOAT_MAX_HEADING_ERROR 900   // Error for max turn thrust (degrees*10 - up to 1800) 
#define BOAT_NO_HEADING_ERROR 0 

//=======================================================================================


//=======================================================================================
// Global data 

Boat boat; 

//=======================================================================================


//=======================================================================================
// Setup 

// Constructor 
Boat::Boat() 
    : Vehicle(MAV_TYPE_SURFACE_BOAT), 
      main_state(MainStates::INIT_STATE)
{
    main_state_flags.flags = RESET; 
    main_state_flags.init_state = FLAG_SET; 
}


// Boat setup 
void Boat::VehicleSetup(void)
{
    main_event_info.dispatch = MainDispatch; 
    comms_event_info.dispatch = CommsDispatch; 

    periodic_timer_50ms.callback = TimerCallback50ms; 
    periodic_timer_100ms.callback = TimerCallback100ms; 
    periodic_timer_250ms.callback = TimerCallback250ms; 
    periodic_timer_1s.callback = TimerCallback1s; 
}

//=======================================================================================


//=======================================================================================
// Vehicle Drive 

// Updates to propulsion and steering don't need to be protected because they're changing 
// continuous PWM values, so there are no outgoing messages to interrupt. 

/**
 * @brief Manual drive output 
 * 
 * @param main_channels : RC transmitter/receiver channels 
 */
void Boat::ManualDrive(VehicleControl::ChannelFunctions main_channels)
{
#if VS_BOAT_K1 
    
    // With this setup the steering (roll) command has to be mapped to a thruster output. 

    uint16_t 
    left_thruster = main_channels.throttle, 
    right_thruster = main_channels.throttle; 

    // Only account for steering input if there is any throttle and steering input. 
    if ((main_channels.throttle != VehicleControl::PWM_NEUTRAL) && 
        (main_channels.roll != VehicleControl::PWM_NEUTRAL))
    {
        // Both the left and right thuster start with the value provided by the throttle 
        // input, then depending on the position of the roll input the left or right 
        // thruster will be linearly scaled back to allow for differential thrust 
        // steering. For example, a full left turn (roll) input will scale the left 
        // thruster value to neutral (zero thrust) while the right thuster will have the 
        // value of the throttle input. 

        int16_t rise = (int16_t)VehicleControl::PWM_NEUTRAL - (int16_t)main_channels.throttle; 
        int16_t run = VehicleControl::PWM_NEUTRAL - VehicleControl::PWM_LOW; 
        int16_t roll = (int16_t)VehicleControl::PWM_NEUTRAL - (int16_t)main_channels.roll; 
        int16_t thrust_diff = (rise * roll) / run; 

        if (main_channels.roll < VehicleControl::PWM_NEUTRAL)
        {
            right_thruster += (uint16_t)thrust_diff; 
        }
        else 
        {
            left_thruster += (uint16_t)(-thrust_diff); 
        }
    }

    hardware.PropulsionSet(left_thruster, right_thruster); 

#elif VS_BOAT_K2 

    // With this setup the inputs can directly be applied to the motor output and the 
    // unused DOFs set to neutral. 
    hardware.PropulsionSet(main_channels.throttle, VehicleControl::PWM_NEUTRAL); 
    hardware.SteeringSet(main_channels.roll, VehicleControl::PWM_NEUTRAL, VehicleControl::PWM_NEUTRAL); 

#endif 
}


/**
 * @brief Autonomous drive output 
 * 
 * @param heading_error : error between current and desired headings (-1799 to 1800 degrees*10) 
 */
void Boat::AutoDrive(int16_t heading_error)
{
#if VS_BOAT_K1 

    uint16_t 
    left_thruster = BOAT_MAX_THRUST, 
    right_thruster = BOAT_MAX_THRUST; 

    // If the boat is not pointing in the direction it needs to go then adjust the motor 
    // ouptut. Otherwise continue straight at the set thrust. 
    if (heading_error != BOAT_NO_HEADING_ERROR)
    {
        int16_t rise = BOAT_MAX_THRUST - VehicleControl::PWM_NEUTRAL; 
        uint16_t *thruster = nullptr; 

        // Check which direction the boat needs to turn 
        if (heading_error < BOAT_NO_HEADING_ERROR)
        {
            heading_error = -heading_error; 
            thruster = &left_thruster; 
        }
        else
        {
            thruster = &right_thruster; 
        }

        // Cap the heading error if it exceeds the set limit 
        if (heading_error > BOAT_MAX_HEADING_ERROR)
        {
            heading_error = BOAT_MAX_HEADING_ERROR; 
        }

        *thruster -= (uint16_t)((rise * heading_error) / BOAT_MAX_HEADING_ERROR); 
    }

    hardware.PropulsionSet(left_thruster, right_thruster); 
    
#elif VS_BOAT_K2 

    // In this configuration the boat can be set to the target speed and the heading 
    // error only applies to the sterring output. 
    
#endif 
}

//=======================================================================================
