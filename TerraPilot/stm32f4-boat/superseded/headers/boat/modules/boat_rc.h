/**
 * @file boat_rc.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Boat remote control interface 
 * 
 * @version 0.1
 * @date 2024-06-17
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef _BOAT_RC_H_
#define _BOAT_RC_H_ 

//=======================================================================================
// Includes 
//=======================================================================================


//=======================================================================================
// Classes 

// Forward declare the Boat class 
class Boat; 

class BoatRC 
{
public:   // Public member functions 

    // Constructor 
    BoatRC() {}

    // Destructor 
    ~BoatRC() {} 

    // Radio connection check 
    void RadioConnectionCheck(uint8_t radio_connection_status); 

    // Remote control thruster output 
    void RemoteControl(
        int16_t throttle_value, 
        uint8_t motor_id, 
        uint8_t direction_id); 

    // Turn thrusters off 
    void ThrustersOff(void); 
}; 

//=======================================================================================

#endif   // _BOAT_RC_H_ 
