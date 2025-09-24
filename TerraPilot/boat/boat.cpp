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
// Global data 

Boat boat; 

static constexpr float boat_max_heading_error = 90.0f;   // Error for max turn thrust (degrees) 
static constexpr float boat_no_heading_error = 0.0f;     // No error (degrees) 

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

        uint16_t 
        run = VehicleControl::PWM_NEUTRAL - VehicleControl::PWM_LOW, 
        rise, roll, *thruster = nullptr;

        if (main_channels.roll > VehicleControl::PWM_NEUTRAL)
        {
            roll = main_channels.roll - VehicleControl::PWM_NEUTRAL;
            thruster = &right_thruster;
        }
        else
        {
            roll = VehicleControl::PWM_NEUTRAL - main_channels.roll;
            thruster = &left_thruster;
        }

        if (main_channels.throttle > VehicleControl::PWM_NEUTRAL)
        {
            rise = main_channels.throttle - VehicleControl::PWM_NEUTRAL;
            *thruster -= roll * rise / run;
        }
        else
        {
            rise = VehicleControl::PWM_NEUTRAL - main_channels.throttle;
            *thruster += roll * rise / run;
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
void Boat::AutoDrive(float heading_error)
{
#if VS_BOAT_K1 

    uint16_t 
    left_thruster = auto_max_pwm, 
    right_thruster = auto_max_pwm; 

    // If the boat is not pointing in the direction it needs to go then adjust the motor 
    // ouptut. Otherwise continue straight at the set thrust. 
    if (heading_error != boat_no_heading_error)
    {
        float rise = static_cast<float>(auto_max_pwm - VehicleControl::PWM_NEUTRAL); 
        uint16_t *thruster = nullptr; 

        // Check which direction the boat needs to turn 
        if (heading_error < boat_no_heading_error)
        {
            heading_error = -heading_error; 
            thruster = &left_thruster; 
        }
        else
        {
            thruster = &right_thruster; 
        }

        // Cap the heading error if it exceeds the set limit 
        if (heading_error > boat_max_heading_error)
        {
            heading_error = boat_max_heading_error; 
        }

        *thruster -= static_cast<uint16_t>(rise * heading_error / boat_max_heading_error); 
    }

    hardware.PropulsionSet(left_thruster, right_thruster); 
    
#elif VS_BOAT_K2 

    // In this configuration the boat can be set to the target speed and the heading 
    // error only applies to the sterring output. 
    
#endif 
}

//=======================================================================================


//=======================================================================================
// Setters 

/**
 * @brief Set the max PWM motor output for autonomous modes 
 * 
 * @param max_pwm : max PWM (1500 - 2000) 
 */
void Boat::AutoDriveMaxPWMSet(uint16_t max_pwm)
{
    auto_max_pwm = max_pwm;
}

//=======================================================================================
