/**
 * @file boat_rc.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Boat remote control 
 * 
 * @version 0.1
 * @date 2024-06-17
 * 
 * @copyright Copyright (c) 2024
 * 
 */

//=======================================================================================
// Includes 

#include "boat.h" 

//=======================================================================================


//=======================================================================================
// User functions 

// Radio connection check 
void BoatRC::RadioConnectionCheck(Boat& boat_rc)
{
    // Shut the thrusters off if there is no radio connection 
    if (!boat_rc.radio.ConnectionStatus())
    {
        boat_rc.navigation.ThrustersOff(); 
    }
}


// Remote control thruster output 
void BoatRC::RemoteControl(void)
{
    // 

    // // Check that the command matches a valid throttle command. If it does then update 
    // // the thruster command. Throttle command filtering is done by system 1. 

    // int16_t cmd_value = (int16_t)rc_cmd_data.cmd_value; 
    // int16_t throttle = RC_MOTOR_NO_THRUST; 
    // uint8_t motor = rc_cmd_data.cmd_id[0]; 
    // uint8_t direction = rc_cmd_data.cmd_id[1]; 

    // // Determine the throttle command 
    // if (direction == RC_MOTOR_FWD_THRUST)
    // {
    //     throttle = cmd_value; 
    // }
    // else if (direction == RC_MOTOR_REV_THRUST)
    // {
    //     throttle = ~cmd_value + 1; 
    // }

    // // Determine the motor 
    // if (motor == RC_MOTOR_RIGHT_MOTOR)
    // {
    //     rc_test_thruster_output(&timeout, DEVICE_ONE, throttle); 
    // }
    // else if (motor == RC_MOTOR_LEFT_MOTOR)
    // {
    //     rc_test_thruster_output(&timeout, DEVICE_TWO, throttle); 
    // }
}

//=======================================================================================
