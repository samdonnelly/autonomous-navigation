/**
 * @file boat.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Boat interface 
 * 
 * @version 0.1
 * @date 2024-01-31
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef _BOAT_H_
#define _BOAT_H_ 

#ifdef __cplusplus
extern "C" {
#endif

//=======================================================================================
// Notes 
// - Features: (top level - what the user can see/do) 
//   - Choose between throttle control (0-100% ESC/motor throttle) and speed control (GPS 
//     speed setpoint and throttle changes to match while still capping the throttle at 
//     its max). 
//   - Mission modes: continuous (starts mission over once done), single (stops after 
//     hitting the last waypoint), etc. 
// 
// - Requirements: (end goals of the features under the hood) 
//   - Coordinates stored on the SD card (not hard coded) and nothing can happen if there 
//     is no mission on the card. 
//   - Indicators (such as lights) need to be 
// 
// - Functionality: (the way features are implemented under the hood) 
//   - The ability to more easily write and read complicated files/data on the SD card 
//     needs to be added. 
// 
// - Tasks: 
//   - Read devices: (some are run as backgroud threads) 
//     - GPS 
//     - Magnetometer 
//     - RF transceiver 
//     - SD card 
//     - IMU 
//     - Analogs: battery voltages 
//     - Sonar 
//   
//   - Write devices: 
//     - RF transceiver 
//     - ESCs/motors 
//     - SD card 
//     - LEDs 
//     - Bluetooth 
//   
//   - Processes 
//     - Navigation calculations 
//     - Filters (ex. Kalman) 
//     - Log data 
//     - Motor controller 
//     - Idle - called when there is nothing else to run 
// 
// - Scheduling 
//   - Data is read from sensors on a schedule or in intervals. When then main thread 
//     needs the data it then just grabs the formatted version from the driver. 
// 
// - State machine: 
//   - State machine and RTOS can be integrated. 
//   - Event driven + input driven 
//   - Input driven needs to account for changing external inputs while that state 
//     machine is running. 
// 
// - States: 
//   - Autonomous 
//   - Manual 
//   - Loiter 
// 
// - ArduPilot 
//   - Each platform has a 1kHz timer available. This can help time when tasks are run. 
//     If tasks need to run slower then counters are used to accumulate time. Slow tasks 
//     are not called on the timer thread because they would slow it down. 
//   - Callbacks / function pointer are used to define the scheduler. 
//   - An SD card is used to store data. The filesystem within a root directory is as 
//     follows: 
//     - LOGS : flight logs and stored data 
//     - TERRAIN : terrain data 
//     - STRNG_BAK : parameter data is backup up here on every boot 
//     - scripts : LUA scripts 
//=======================================================================================


//=======================================================================================
// Includes 

#include "includes_drivers.h" 
#include "includes_cpp_drivers.h" 
#include "stm32f4xx_it.h" 

// FreeRTOS 
#include "FreeRTOS.h" 
// #include "task.h" 
#include "cmsis_os2.h" 
#include "queue.h" 
// #include "semphr.h" 
// #include "timers.h" 

//=======================================================================================


//=======================================================================================
// Macros 
//=======================================================================================


//=======================================================================================
// Boat object 

class Boat 
{
private:   // Private members 

    // States 

    // Events 

    // Flags 

public:   // Public members 

    // 

private:   // Private member functions 

    // Dispatch function(s) 
    static void BoatMainDispatch(Event event); 

    // State functions 

    // State table 

    // Helper functions 

public:   // Public member functions 

    // Constructor(s) 
    Boat() {} 

    // Destructor 
    ~Boat() {} 

    // Setup 
    void BoatSetup(void); 
}; 

extern Boat boat; 

//=======================================================================================

#ifdef __cplusplus
}
#endif

#endif   // _BOAT_H_ 
