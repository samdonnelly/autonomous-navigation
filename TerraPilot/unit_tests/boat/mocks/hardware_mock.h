/**
 * @file hardware_mock.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Hardware mock interface 
 * 
 * @version 0.1
 * @date 2025-03-28
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef _HARDWARE_MOCK_H_ 
#define _HARDWARE_MOCK_H_ 

//=======================================================================================
// Includes 

#include "vehicle.h" 

//=======================================================================================


//=======================================================================================
// Macros 

#define HW_MOCK_CONTROL_SETPOINTS 25 

//=======================================================================================


//=======================================================================================
// Mock data 

class HardwareMock 
{
public: 

    // Actuators 
    struct ControlSetpoints 
    {
        uint16_t throttle_1, throttle_2; 
        uint16_t roll, pitch, yaw; 
    }; 

    std::array<ControlSetpoints, HW_MOCK_CONTROL_SETPOINTS> control_setpoints; 
    uint8_t throttle_setpoint_index, orientation_setpoint_index; 

public: 

    void HardwareMockInit(void); 

    void ThrottleSepointCopy(uint16_t throttle_1, uint16_t throttle_2); 
    void SteeringSepointCopy(uint16_t roll, uint16_t pitch, uint16_t yaw); 
    void ControlSetpointGet(std::array<ControlSetpoints, HW_MOCK_CONTROL_SETPOINTS> &setpoints); 
}; 

extern HardwareMock hardware_mock; 

//=======================================================================================

#endif   // _HARDWARE_MOCK_H_ 
