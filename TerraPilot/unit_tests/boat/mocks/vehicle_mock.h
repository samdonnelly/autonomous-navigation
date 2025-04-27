/**
 * @file vehicle_mock.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Generic vehicle mock interface 
 * 
 * @version 0.1
 * @date 2025-04-22
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef _VEHICLE_MOCK_H_ 
#define _VEHICLE_MOCK_H_ 

//=======================================================================================
// Includes 

#include "vehicle.h" 

//=======================================================================================


//=======================================================================================
// Macros 

#define VEHICLE_MOCK_CHANNEL_BUFF 25 
#define VEHICLE_MOCK_HEADING_ERROR_BUFF 19 

//=======================================================================================


//=======================================================================================
// Mock data 

class VehicleMock 
{
public:   // public members 

    enum TestFunctionIndex : uint8_t
    {
        MANUAL_DRIVE, 
        AUTO_DRIVE 
    }
    test_function_index; 

    std::array<VehicleControl::ChannelFunctions, VEHICLE_MOCK_CHANNEL_BUFF> channels; 
    std::array<int16_t, VEHICLE_MOCK_HEADING_ERROR_BUFF> headings; 

public:   // public methods 

    VehicleMock() {} 

    // Setters 
    void TestFunctionIndexSet(TestFunctionIndex index); 
    void RCChannelSet(
        std::array<VehicleControl::ChannelFunctions, VEHICLE_MOCK_CHANNEL_BUFF> &rc_channels); 
    void HeadingErrorsSet(std::array<int16_t, VEHICLE_MOCK_HEADING_ERROR_BUFF> &heading_errors); 
};

extern VehicleMock vehicle_mock; 

//=======================================================================================

#endif   // _VEHICLE_MOCK_H_ 
