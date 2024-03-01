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

#include "stm32f411xe.h"

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
// Prototypes 

/**
 * @brief 
 */
void boat_state0(void); 


/**
 * @brief 
 */
void boat_state1(void); 

//=======================================================================================


//=======================================================================================
// Classes 

class boat_state_data 
{
private:   // Private data 
    
    // State function pointer 
    typedef void (*state_ptr)(void); 

    // Data 
    uint8_t state;   // Application state 

public:   // Public data 

    // Store the states + indexes 
    enum 
    {
        STATE0,      // State 0 
        STATE1,      // State 1 
        NUM_STATES   // Keeps track of the number of states. Not a state itself. 
    }; 

    // State table 
    state_ptr state_table[NUM_STATES] = 
    {
        &boat_state0, 
        &boat_state1 
    }; 

public:   // Setup and teardown 

    // Constructor 
    boat_state_data() {} 

    // Destructor 
    ~boat_state_data() {} 

public: 
    
    // Main state machine 
    void state_machine(void); 
}; 


static boat_state_data boat; 

//=======================================================================================


//=======================================================================================
// Main application 

void boat_app(void)
{
    boat.state_machine(); 
}

//=======================================================================================


//=======================================================================================
// State machine 

// Top level state machine 
void boat_state_data::state_machine(void) 
{
    uint8_t next_state = state; 

    switch(next_state)
    {
        case STATE0: 
            next_state = STATE1; 
            break; 

        case STATE1: 
            next_state = STATE0; 
            break; 
        
        default: 
            next_state = STATE0; 
            break; 
    }

    // Execute the state and update the state record 
    state_table[next_state](); 
    state = next_state; 
}

//=======================================================================================


//=======================================================================================
// States 

// State 0 
void boat_state0(void)
{
    // 
}


// State 1 
void boat_state1(void)
{
    // 
}

//=======================================================================================