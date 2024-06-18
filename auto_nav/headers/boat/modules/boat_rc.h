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
public:   // Public members 

    // 

private:   // Private members 

    // 

public:   // Public member functions 

    // Constructor 
    BoatRC() {}

    // Destructor 
    ~BoatRC() {} 

    // Radio connection check 
    void RadioConnectionCheck(Boat& boat_rc); 

    // Remote control thruster output 
    void RemoteControl(void); 

private:   // Private member functions 

    // 
}; 

//=======================================================================================

#endif   // _BOAT_RC_H_ 
