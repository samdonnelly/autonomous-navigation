/**
 * @file vehicle_hardware.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Vehicle hardware interface 
 * 
 * @details Vehicle hardware functions are not defined by the autopilot. They should be 
 *          defined within the project using this autopilot library so the project can 
 *          add a hardware specific interface. 
 * 
 * @version 0.1
 * @date 2025-02-24
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef _VEHICLE_HARDWARE_H_ 
#define _VEHICLE_HARDWARE_H_ 

//=======================================================================================
// Classes 

class VehicleHardware 
{
public:   // Public member functions 
    // Constructor 
    VehicleHardware() {}

    // Destructor 
    ~VehicleHardware() {}

private:   // Private member functions 
    // Hardware interface 
    void VH_ReceiverRead(void); 

    void VH_TelemetryRead(void); 
    void VH_TelemetryWrite(void); 
    
    void VH_GPSRead(void); 
    void VH_GPSWrite(void); 
    
    void VH_CompassRead(void); 
    void VH_CompassWrite(void); 
    
    void VH_IMURead(void); 
    void VH_IMUWrite(void); 
    
    void VH_SDCardRead(void); 
    void VH_SDCardWrite(void); 
    
    void VH_ESCWrite(void); 
    
    void VH_ADCRead(void); 
    
    void VH_LEDWrite(void); 
}; 

//=======================================================================================

#endif   // _VEHICLE_HARDWARE_H_ 
