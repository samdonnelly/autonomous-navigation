/**
 * @file manual_mode.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Boat manual mode interface 
 * 
 * @version 0.1
 * @date 2024-03-16
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef _BOAT_MANUAL_MODE_H_ 
#define _BOAT_MANUAL_MODE_H_ 

#ifdef __cplusplus
extern "C" {
#endif

//=======================================================================================
// Includes 

#include "includes_drivers.h" 

//=======================================================================================


//=======================================================================================
// Classes 

class boat_manual_mode 
{
private:   // Private members 

    // Thrusters 
    int16_t right_thruster, left_thruster;   // Thruster throttle 

public:   // Public member functions 

    // Constructor 
    boat_manual_mode(); 

    // Destructor 
    ~boat_manual_mode(); 

    /**
     * @brief Manual mode 
     * 
     * @details While in manual control mode, if there is a valid new manual throttle 
     *          command then use the command will be translated into a thruster output. 
     *          Throttle commands get sent by the ground station. This function is only 
     *          called while in manual control mode. 
     * 
     * @param manual_cmd_ready : new manual control command flag 
     * @param cmd_id : command that identifies the thruster and direction 
     * @param command_value : throttle level 
     */
    void manual_mode(
        uint8_t& manual_cmd_ready, 
        uint8_t *cmd_id, 
        uint8_t command_value); 

    /**
     * @brief Manual mode exit 
     * 
     * @details Called when the boat is exiting manual control mode. Makes sure manual 
     *          mode is left in a safe state including turning the thrusters off. 
     */
    void manual_mode_exit(void); 
}; 

//=======================================================================================

#ifdef __cplusplus
}
#endif

#endif   // _BOAT_MANUAL_MODE_H_ 
