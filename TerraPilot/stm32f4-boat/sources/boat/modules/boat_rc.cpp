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
#include "esc_config.h" 
#include "vehicle_radio_config.h" 

//=======================================================================================


//=======================================================================================
// User functions 

// Radio connection check 
void BoatRC::RadioConnectionCheck(uint8_t radio_connection_status)
{
    // Shut the thrusters off if there is no radio connection. This function is queued 
    // periodically when in manual mode. 
    if (!radio_connection_status)
    {
        ThrustersOff(); 
    }
}


// Remote control thruster output 
void BoatRC::RemoteControl(
    int16_t throttle_value, 
    uint8_t motor_id, 
    uint8_t direction_id)
{
    // The manual control thruster commands are checked by the radio before this function 
    // is called so we don't have to verify if the parameters are valid. We check if 
    // reverse direction is requested so we can change the sign on the throttle value 
    // as negative values are not sent over the radio. The boat radio module keeps 
    // track of the radio connection and if the connection is lost then the thrusters 
    // will be forced to zero output. 

    // Determine the throttle command 
    if (direction_id == GS_RADIO_REV_DIRECTION)
    {
        throttle_value = ~throttle_value + 1; 
    }

    // Determine the motor 
    if (motor_id == GS_RADIO_RIGHT_JOYSTICK)
    {
        esc_send(DEVICE_ONE, throttle_value); 
    }
    else if (motor_id == GS_RADIO_LEFT_JOYSTICK)
    {
        esc_send(DEVICE_TWO, throttle_value); 
    }
}


// Turn thrusters off 
void BoatRC::ThrustersOff(void)
{
    esc_send(DEVICE_ONE, ESC_NO_THRUST); 
    esc_send(DEVICE_TWO, ESC_NO_THRUST); 
}

//=======================================================================================
