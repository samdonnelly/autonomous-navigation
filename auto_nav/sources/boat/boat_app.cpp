/**
 * @file boat_app.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Autonomous RC boat application implementation 
 * 
 * @version 0.1
 * @date 2024-01-31
 * 
 * @copyright Copyright (c) 2024
 * 
 */

//=======================================================================================
// Includes 

#include "boat_app.h" 

//=======================================================================================


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
//     - Log data 
//     - Idle - called when there is nothing else to run 
// 
// - Scheduling 
//   - Data is read from sensors on a schedule or in intervals. When then main thread 
//     needs the data it then just grabs the formatted version from the driver. 
// 
// - States: 
//   - 
// 
// - ArduPilot 
//   - Each platform has a 1kHz timer available. This can help time when tasks are run. 
//     If tasks need to run slower then counters are used to accumulate time. Slow tasks 
//     are not called on the timer thread because they would slow it down. 
//   - Callbacks / function pointer are used to define the scheduler. 
//=======================================================================================
