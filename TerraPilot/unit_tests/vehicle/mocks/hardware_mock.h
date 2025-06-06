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
// Mock data 

class HardwareMock 
{
public: 

    // Telemetry 
    uint8_t telemetry_in_buff[VS_TELEMETRY_BUFF]; 
    uint16_t telemetry_in_size; 
    uint16_t telemetry_out_size; 

public: 

    void HardwareMockInit(void); 

    // Telemetry 
    void TelemetryInAppend(uint16_t size, uint8_t *buffer); 
    void TelmetryInReset(void); 
    uint16_t TelemetryOutGetSize(void); 
}; 

extern HardwareMock hardware_mock; 

//=======================================================================================

#endif   // _HARDWARE_MOCK_H_ 
