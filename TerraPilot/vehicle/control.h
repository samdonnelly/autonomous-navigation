/**
 * @file control.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Vehicle control interface 
 * 
 * @version 0.1
 * @date 2025-03-14
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef _CONTROL_H_ 
#define _CONTROL_H_ 

//=======================================================================================
// Includes 

#include "includes.h" 

//=======================================================================================


//=======================================================================================
// Classes 

class Vehicle; 


class VehicleControl 
{
public:   // public types 

    enum PWMThresholds : uint16_t 
    {
        PWM_MIN = 800,          // PWM value must be higher than this to be valid 
        PWM_LOW = 1000,         // Normal low PWM value 
        PWM_AUX_LOW = 1200,     // Aux switch is low 
        PWM_MAX_MODE1 = 1230,   // Max PWM for mode 1 
        PWM_MAX_MODE2 = 1360,   // Max PWM for mode 2 
        PWM_MAX_MODE3 = 1490,   // Max PWM for mode 3 
        PWM_NEUTRAL = 1500,     // Neutral PWM  
        PWM_MAX_MODE4 = 1620,   // Max PWM for mode 4 
        PWM_MAX_MODE5 = 1750,   // Max PWM for mode 5 
        PWM_AUX_HIGH = 1800,    // Aux switch is high 
        PWM_HIGH = 2000,        // Normal max PWM value 
        PWM_MAX = 2200          // PWM value must be lower than this to be valid 
    };

    enum RCModes : uint8_t 
    {
        RC_MODE1,   // PWM: 0-1230 
        RC_MODE2,   // PWM: 1231-1360 
        RC_MODE3,   // PWM: 1361-1490 
        RC_MODE4,   // PWM: 1491-1620 
        RC_MODE5,   // PWM: 1621-1750 
        RC_MODE6    // PWM: 1751+ 
    };

    struct ChannelFunctions 
    {
        // Main controls - Default/neutral PWM is 1500 
        uint16_t throttle, roll, pitch, yaw; 

        // Auxiliary controls - Default/neutral PWM is 1000 
        uint16_t mode_control, mode; 
        uint16_t aux3, aux4, aux5, aux6, aux7, aux8, aux9, aux10; 
    }; 
    
private:   // private members 

    struct Timers 
    {
        uint8_t rc_connection; 
    }
    timers; 

    union Status
    {
        struct 
        {
            uint32_t rc_connected : 1; 
        }; 
        uint32_t flags; 
    }
    status; 

    // RC 
    ChannelFunctions channels; 

private:   // private methods 

    // RC data handling 
    void RCModeDecode(Vehicle &vehicle); 
    void RCDataChecks(void); 

public:   // public methods 

    // Constructor 
    VehicleControl(); 

    // RC data handling 
    void RCUpdate(Vehicle &vehicle); 
    void RemoteControl(Vehicle &vehicle); 

    // Propulsion and steering 
    void ForceStop(Vehicle &vehicle); 
}; 

//=======================================================================================

#endif   // _CONTROL_H_ 
