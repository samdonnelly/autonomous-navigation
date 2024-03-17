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

    // Autonomous mode 
    void manual_mode(
        uint8_t& manual_cmd_ready, 
        uint8_t *cmd_id, 
        uint8_t command_value); 

    // Autonomous mode exit 
    void manual_mode_exit(void); 
}; 

//=======================================================================================

#ifdef __cplusplus
}
#endif

#endif   // _BOAT_MANUAL_MODE_H_ 
