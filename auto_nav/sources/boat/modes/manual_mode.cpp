/**
 * @file manual_mode.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Boat manual mode 
 * 
 * @version 0.1
 * @date 2024-03-16
 * 
 * @copyright Copyright (c) 2024
 * 
 */

//=======================================================================================
// Includes 

#include "manual_mode.h" 

//=======================================================================================


//=======================================================================================
// Macros 

// Manual Control 
#define AB_MC_LEFT_MOTOR 0x4C        // "L" character that indicates left motor 
#define AB_MC_RIGHT_MOTOR 0x52       // "R" character that indicates right motor 
#define AB_MC_FWD_THRUST 0x50        // "P" (plus) - indicates forward thrust 
#define AB_MC_REV_THRUST 0x4D        // "M" (minus) - indicates reverse thrust 
#define AB_MC_NEUTRAL 0x4E           // "N" (neutral) - indicates neutral gear or zero thrust 

// Thuster 
#define AB_NO_THRUST 0               // Force thruster output to zero 

//=======================================================================================


//=======================================================================================
// Setup and teardown 

// Constructor 
boat_manual_mode::boat_manual_mode()
    : right_thruster(AB_NO_THRUST), 
      left_thruster(AB_NO_THRUST) {}


// Destructor 
boat_manual_mode::~boat_manual_mode() {}

//=======================================================================================


//=======================================================================================
// Manual mode functions 

// Autonomous mode 
void boat_manual_mode::manual_mode(
    uint8_t& manual_cmd_ready, 
    uint8_t *cmd_id, 
    uint8_t command_value)
{
    // Only attempt a throttle command update if a new data command was received 
    if (manual_cmd_ready)
    {
        manual_cmd_ready = CLEAR_BIT; 

        // Check that the command matches a valid throttle command. If it does then 
        // update the thruster command. 

        int16_t cmd_value = (int16_t)command_value; 

        if (cmd_id[0] == AB_MC_RIGHT_MOTOR)
        {
            switch (cmd_id[1])
            {
                case AB_MC_FWD_THRUST: 
                    right_thruster += (cmd_value - right_thruster) >> SHIFT_3; 
                    esc_readytosky_send(DEVICE_ONE, right_thruster); 
                    break; 
                
                case AB_MC_REV_THRUST: 
                    right_thruster += ((~cmd_value + 1) - right_thruster) >> SHIFT_3; 
                    esc_readytosky_send(DEVICE_ONE, right_thruster); 
                    break; 
                
                case AB_MC_NEUTRAL: 
                    if (cmd_value == AB_NO_THRUST)
                    {
                        right_thruster = AB_NO_THRUST; 
                        esc_readytosky_send(DEVICE_ONE, right_thruster); 
                    }
                    break; 
                
                default: 
                    break; 
            }
        }
        else if (cmd_id[0] == AB_MC_LEFT_MOTOR)
        {
            switch (cmd_id[1])
            {
                case AB_MC_FWD_THRUST: 
                    left_thruster += (cmd_value - left_thruster) >> SHIFT_3; 
                    esc_readytosky_send(DEVICE_TWO, left_thruster); 
                    break; 
                
                case AB_MC_REV_THRUST: 
                    left_thruster += ((~cmd_value + 1) - left_thruster) >> SHIFT_3; 
                    esc_readytosky_send(DEVICE_TWO, left_thruster); 
                    break; 
                
                case AB_MC_NEUTRAL: 
                    if (cmd_value == AB_NO_THRUST)
                    {
                        left_thruster = AB_NO_THRUST; 
                        esc_readytosky_send(DEVICE_TWO, left_thruster); 
                    }
                    break; 
                
                default: 
                    break; 
            }
        }
    }
}


// Autonomous mode exit 
void boat_manual_mode::manual_mode_exit(void)
{
    // Set the throttle to zero to stop the thrusters 
    esc_readytosky_send(DEVICE_ONE, AB_NO_THRUST); 
    esc_readytosky_send(DEVICE_TWO, AB_NO_THRUST); 
}

//=======================================================================================
