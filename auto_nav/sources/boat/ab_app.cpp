/**
 * @file project_init.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Autonomous boat application code 
 * 
 * @version 0.1
 * @date 2023-07-20
 * 
 * @copyright Copyright (c) 2023
 * 
 */

//=======================================================================================
// Includes 

#include "ab_interface.h" 
#include "ws2812_config.h" 
#include "gps_coordinates.h" 

//=======================================================================================


//=======================================================================================
// Macros 

// System info 
#define AB_NUM_STATES 8              // Number of system states 

// Timing 
#define AB_INIT_DELAY 1000000        // Init state delay (us) 

// Manual Control 
#define AB_MC_LEFT_MOTOR 0x4C        // "L" character that indicates left motor 
#define AB_MC_RIGHT_MOTOR 0x52       // "R" character that indicates right motor 
#define AB_MC_FWD_THRUST 0x50        // "P" (plus) - indicates forward thrust 
#define AB_MC_REV_THRUST 0x4D        // "M" (minus) - indicates reverse thrust 
#define AB_MC_NEUTRAL 0x4E           // "N" (neutral) - indicates neutral gear or zero thrust 

// Thrusters 
#define AB_NO_THRUST 0               // Force thruster output to zero 

//=======================================================================================


//=======================================================================================
// Function pointers 

/**
 * @brief State machine function pointer 
 */
typedef void (*ab_state_func_ptr_t)(void); 

//=======================================================================================


//=======================================================================================
// Prototypes 

/**
 * @brief Initialization state 
 * 
 * @details First state to run on startup and after a system reset. Configures the system 
 *          as needed before going to the "not ready" state. 
 */
void ab_init_state(void); 


/**
 * @brief Not ready state 
 * 
 * @details Waits for the "ready" flag to be set. This state provides an indication to 
 *          the ground station that the boat is not ready to start navigating either 
 *          manually or autonomously (ex. no position lock). 
 *          
 *          Entered from the "init" state or when the "ready" flag is cleared while in 
 *          the "ready", "manual" or "auto" states. The "ready" flag can be cleared if 
 *          the boat loses position lock while in "ready" and "auto" states, and if the 
 *          heartbeat is not seen while in "ready" and "manual" states. The state is 
 *          exited if the "ready" flag is set. 
 */
void ab_not_ready_state(void); 


/**
 * @brief Ready state 
 * 
 * @details Waits for the ground station station to choose either manual or autonomous 
 *          mode. This state provides an indication to the ground station that the 
 *          boat is ready to start navigating. 
 *          
 *          Entered from the "not ready" state once the boat is ready to start 
 *          navigating, and from the "manual" and "auto" states if the boat is commanded 
 *          to idle by the ground station. Exited when the ground station selects either 
 *          manual or autonomous mode, or if the boat no longer meets the requirements 
 *          to start navigating (ex. position lock lost). 
 */
void ab_ready_state(void); 


/**
 * @brief Manual control state 
 * 
 * @details Allows for the ground station to manually drive the boat. Thruster commands 
 *          are send by the ground station and read by the boat via radio which get 
 *          translated to thruster outputs. A loss of radio communication will remove 
 *          the boat from manual control mode and turn off the thrusters. 
 *          
 *          Entered from the "ready" state when the ground station selects manual mode. 
 *          Exited if the ground station commands the boat to idle or if radio 
 *          communication is lost. 
 */
void ab_manual_state(void); 


/**
 * @brief Autonomous navigation state 
 * 
 * @details Autonomously navigates a pre-defined waypoint mission. On startup, the first 
 *          pre-defined waypoint is the boats default target waypoint. The boat will not 
 *          start navigating these waypoints until this mode is entered. The target 
 *          location can be updated via commands from the ground station. Upon hitting 
 *          a waypoint, the boat will automatically target the next pre-defined waypoint. 
 *          
 *          Entered from the "ready" state when the ground station selects autonomous 
 *          mode. Exited if the ground station commands the boat to idle or if GNSS 
 *          position is lost. 
 */
void ab_auto_state(void); 


/**
 * @brief Low power state 
 * 
 * @details Waits for the "reset" flag to be set. This state is used when the battery 
 *          voltage is too low. It's meant to do minimal work to provide time to recover 
 *          the boat before the battery SOC drops too low. 
 *          
 *          Entered from any continuous state when the battery voltage is observed to be 
 *          too low. Exited only when the "reset" flag is set, however upon reset the 
 *          system will re-enter low power mode if battery voltage is still too low. 
 */
void ab_low_pwr_state(void); 


/**
 * @brief Fault state 
 * 
 * @details Waits for the "reset" flag to be set. This state is used when there is a 
 *          fault seen somewhere in the system that is preventing it from operating 
 *          normally. 
 *          
 *          Entered from any continuous state when a fault is seen somewhere in the 
 *          system. Exits to the "reset" state when the "reset" flag gets set. 
 */
void ab_fault_state(void); 


/**
 * @brief Reset state 
 * 
 * @details Clears fault codes and resets the systems main control loop. 
 *          
 *          Entered from the "low power" and "fault" states. Exits to the "init" state. 
 */
void ab_reset_state(void); 

//=======================================================================================


//=======================================================================================
// Control data 

// State function pointer table 
static ab_state_func_ptr_t state_table[AB_NUM_STATES] = 
{
    &ab_init_state, 
    &ab_not_ready_state, 
    &ab_ready_state, 
    &ab_manual_state, 
    &ab_auto_state, 
    &ab_low_pwr_state, 
    &ab_fault_state, 
    &ab_reset_state 
}; 

//=======================================================================================


//=======================================================================================
// Autonomous boat application 

// Boat application 
void Boat::BoatApp(void)
{
    // Autonomous boat application code here 

    ab_states_t next_state = state; 

    //==================================================
    // System checks and updates 

    // Check for critical info 
    BoatSystemCheck(); 

    // Check radio communication 
    radio_comm_check(next_state); 

    // Update the LED strobe 
    strobe(); 

    //==================================================

    //==================================================
    // System state machine 

    switch (next_state)
    {
        case AB_INIT_STATE: 
            if (!init)
            {
                next_state = AB_NOT_READY_STATE; 
            }
            break; 
        
        case AB_NOT_READY_STATE: 
            if (fault)
            {
                next_state = AB_FAULT_STATE; 
            }
            else if (low_pwr)
            {
                next_state = AB_LOW_PWR_STATE; 
            }
            else if (idle)
            {
                next_state = AB_READY_STATE; 
            }
            break; 
        
        case AB_READY_STATE: 
            if (fault)
            {
                next_state = AB_FAULT_STATE; 
            }
            else if (low_pwr)
            {
                next_state = AB_LOW_PWR_STATE; 
            }
            else if (!ready)
            {
                next_state = AB_NOT_READY_STATE; 
            }
            else if (manual)
            {
                next_state = AB_MANUAL_STATE; 
            }
            else if (autonomous)
            {
                next_state = AB_AUTO_STATE; 
            }
            break; 
        
        case AB_MANUAL_STATE: 
            if (fault)
            {
                next_state = AB_FAULT_STATE; 
            }
            else if (low_pwr)
            {
                next_state = AB_LOW_PWR_STATE; 
            }
            else if (!ready)
            {
                next_state = AB_NOT_READY_STATE; 
            }
            else if (idle)
            {
                next_state = AB_READY_STATE; 
            }
            break; 
        
        case AB_AUTO_STATE: 
            if (fault)
            {
                next_state = AB_FAULT_STATE; 
            }
            else if (low_pwr)
            {
                next_state = AB_LOW_PWR_STATE; 
            }
            else if (!ready)
            {
                next_state = AB_NOT_READY_STATE; 
            }
            else if (idle)
            {
                next_state = AB_READY_STATE; 
            }
            break; 
        
        case AB_LOW_PWR_STATE: 
            if (reset)
            {
                next_state = AB_RESET_STATE; 
            }
            break; 
        
        case AB_FAULT_STATE: 
            if (reset)
            {
                next_state = AB_RESET_STATE; 
            }
            break; 
        
        case AB_RESET_STATE: 
            if (init)
            {
                next_state = AB_INIT_STATE; 
            }
            break; 
        
        default: 
            next_state = AB_INIT_STATE; 
            break; 
    }
    
    //==================================================

    // Execute the state 
    state_table[next_state](); 

    // Update the state 
    state = next_state; 

    // Run the controllers 
    m8q_controller(); 
}

//=======================================================================================


//=======================================================================================
// System 

// System checks 
void Boat::BoatSystemCheck(void)
{
    // Device status checks 
    if (m8q_get_fault_code())
    {
        fault_code |= (SET_BIT << SHIFT_0); 
    }
    if (nrf24l01_get_status())
    {
        fault_code |= (SET_BIT << SHIFT_2); 
    }

    // System requirements check. If these conditions are not met then take the system 
    // out of an active mode. 
    ready = SET_BIT; 
    
    // GPS position lock check. If the system loses GPS position lock in manual mode then 
    // it continues on. 
    navstat = m8q_get_position_navstat_lock(); 

    if (navstat && (state != AB_MANUAL_STATE))
    {
        ready = CLEAR_BIT; 
    }
    
    // Heartbeat check. If the system loses the heartbeat in autonomous mode then it 
    // continues on. 
    if ((!connect_status()) && (state != AB_AUTO_STATE))
    {
        ready = CLEAR_BIT; 
    }
}

//=======================================================================================


//=======================================================================================
// State functions 

// Initialization state 
void ab_init_state(void)
{
    //==================================================
    // State entry 

    if (boat.state_entry)
    {
        boat.state_entry = CLEAR_BIT; 

        // Set the thruster throttle to zero to initialize the ESCs and motors. 
        esc_readytosky_send(DEVICE_ONE, AB_NO_THRUST); 
        esc_readytosky_send(DEVICE_TWO, AB_NO_THRUST); 

        // Grab the current location (if available) and set the default target waypoint. 
        boat.current.lat = m8q_get_position_lat(); 
        boat.current.lon = m8q_get_position_lon(); 
        boat.target.lat = gps_waypoints[0].lat; 
        boat.target.lon = gps_waypoints[0].lon; 
    }

    //==================================================

    // Set up the file system 

    //==================================================
    // State exit 

    // Short delay to let the system set up before moving into the next state 
    if (tim_compare(boat.timer_nonblocking, 
                    boat.state_timer.clk_freq, 
                    AB_INIT_DELAY, 
                    &boat.state_timer.time_cnt_total, 
                    &boat.state_timer.time_cnt, 
                    &boat.state_timer.time_start))
    {
        boat.state_timer.time_start = SET_BIT; 
        boat.state_entry = SET_BIT; 
        boat.init = CLEAR_BIT; 
    }

    //==================================================
}


// Not ready state 
void ab_not_ready_state(void)
{
    //==================================================
    // State entry 
    
    if (boat.state_entry)
    {
        boat.state_entry = CLEAR_BIT; 

        // Update the LED strobe colour 
        boat.strobe_colour_set(ws2812_led_not_ready); 
    }

    //==================================================

    // Wait for the system requirements to be met - ready flag will be set 

    //==================================================
    // State exit 

    if (boat.fault | boat.low_pwr | boat.ready)
    {
        boat.state_entry = SET_BIT; 
        boat.idle = SET_BIT; 
        boat.manual = CLEAR_BIT; 
        boat.autonomous = CLEAR_BIT; 

        // Make sure the LEDs are off and reset the strobe timer 
        boat.strobe_off(); 
    }

    //==================================================
}


// Ready state 
void ab_ready_state(void)
{
    //==================================================
    // State entry 

    if (boat.state_entry)
    {
        boat.state_entry = CLEAR_BIT; 

        // Update the LED strobe colour 
        boat.strobe_colour_set(ws2812_led_ready); 
    }

    //==================================================

    // Wait for an active state to be chosen 

    //==================================================
    // State exit 

    // Manual and autonomous mode exit conditions come from external commands received so 
    // they're not included here. 

    if (boat.fault | boat.low_pwr | !boat.ready)
    {
        boat.state_entry = SET_BIT; 
        boat.idle = CLEAR_BIT; 

        // Make sure the LEDs are off and reset the strobe timer 
        boat.strobe_off(); 
    }

    //==================================================
}


// Manual control mode state 
void ab_manual_state(void)
{
    int16_t cmd_value = CLEAR; 

    //==================================================
    // State entry 

    if (boat.state_entry)
    {
        boat.state_entry = CLEAR_BIT; 

        // Update the LED strobe colour 
        boat.strobe_colour_set(ws2812_led_manual_strobe); 
    }

    //==================================================

    //==================================================
    // External thruster control 

    // Only attempt a throttle command update if a new data command was received 
    if (boat.mc_data)
    {
        boat.mc_data = CLEAR_BIT; 

        // Check that the command matches a valid throttle command. If it does then update 
        // the thruster command. 
        
        cmd_value = (int16_t)boat.cmd_value; 

        if (boat.cmd_id[0] == AB_MC_RIGHT_MOTOR)
            {
                switch (boat.cmd_id[1])
                {
                    case AB_MC_FWD_THRUST: 
                        boat.right_thruster += (cmd_value - boat.right_thruster) >> SHIFT_3; 
                        esc_readytosky_send(DEVICE_ONE, boat.right_thruster); 
                        break; 
                    
                    case AB_MC_REV_THRUST: 
                        boat.right_thruster += ((~cmd_value + 1) - boat.right_thruster) >> SHIFT_3; 
                        esc_readytosky_send(DEVICE_ONE, boat.right_thruster); 
                        break; 
                    
                    case AB_MC_NEUTRAL: 
                        if (cmd_value == AB_NO_THRUST)
                        {
                            boat.right_thruster = AB_NO_THRUST; 
                            esc_readytosky_send(DEVICE_ONE, boat.right_thruster); 
                        }
                        break; 
                    
                    default: 
                        break; 
                }
            }
            else if (boat.cmd_id[0] == AB_MC_LEFT_MOTOR)
            {
                switch (boat.cmd_id[1])
                {
                    case AB_MC_FWD_THRUST: 
                        boat.left_thruster += (cmd_value - boat.left_thruster) >> SHIFT_3; 
                        esc_readytosky_send(DEVICE_TWO, boat.left_thruster); 
                        break; 
                    
                    case AB_MC_REV_THRUST: 
                        boat.left_thruster += ((~cmd_value + 1) - boat.left_thruster) >> SHIFT_3; 
                        esc_readytosky_send(DEVICE_TWO, boat.left_thruster); 
                        break; 
                    
                    case AB_MC_NEUTRAL: 
                        if (cmd_value == AB_NO_THRUST)
                        {
                            boat.left_thruster = AB_NO_THRUST; 
                            esc_readytosky_send(DEVICE_TWO, boat.left_thruster); 
                        }
                        break; 
                    
                    default: 
                        break; 
                }
            }
    }

    //==================================================

    //==================================================
    // State exit 

    // the idle (ready) state exit condition comes from an external command received 
    // so it's not included here. 

    if (boat.fault | boat.low_pwr | !boat.ready)
    {
        boat.state_entry = SET_BIT; 
        boat.manual = CLEAR_BIT; 

        // Set the throttle to zero to stop the thrusters 
        esc_readytosky_send(DEVICE_ONE, AB_NO_THRUST); 
        esc_readytosky_send(DEVICE_TWO, AB_NO_THRUST); 

        // Make sure the LEDs are off and reset the strobe timer 
        boat.strobe_off(); 
    }

    //==================================================
}


// Autonomous mode state 
void ab_auto_state(void)
{
    // static uint8_t nav_period_counter = CLEAR; 

    //==================================================
    // State entry 

    if (boat.state_entry)
    {
        boat.state_entry = CLEAR_BIT; 

        // Update the LED strobe colour 
        boat.strobe_colour_set(ws2812_led_auto_strobe); 
    }

    //==================================================

    //==================================================
    // Navigation calculations 

    boat.auto_mode(); 

    //==================================================

    //==================================================
    // State exit 

    // The idle (ready) state exit condition comes from an external command received 
    // so it's not included here. 

    if (boat.fault | boat.low_pwr | !boat.ready)
    {
        boat.state_entry = SET_BIT; 
        boat.autonomous = CLEAR_BIT; 

        // Stop auto mode 
        boat.auto_mode_exit(); 

        // Make sure the LEDs are off and reset the strobe timer 
        boat.strobe_off(); 
    }

    //==================================================
}


// Low power state 
void ab_low_pwr_state(void)
{
    //==================================================
    // State entry 

    if (boat.state_entry)
    {
        boat.state_entry = CLEAR_BIT; 

        // Put devices in low power mode 
        m8q_set_low_pwr_flag(); 
    }

    //==================================================

    // Wait for a system reset 
    // Currently the system would hang here until it's power cycled because there is no 
    // setter for the reset flag. This is ok because if the battery voltage is low there 
    // is nothing the system can and should do. 

    //==================================================
    // State exit 

    if (boat.reset)
    {
        boat.state_entry = SET_BIT; 

        // Take devices out of low power mode 
        m8q_clear_low_pwr_flag(); 
    }

    //==================================================
}


// Fault state 
void ab_fault_state(void)
{
    //==================================================
    // State entry 

    if (boat.state_entry)
    {
        boat.state_entry = CLEAR_BIT; 
    }

    //==================================================

    // Go directly to the reset state 
    boat.reset = SET_BIT; 

    //==================================================
    // State exit 

    if (boat.reset)
    {
        boat.state_entry = SET_BIT; 
        boat.fault = CLEAR_BIT; 
    }

    //==================================================
}


// Reset state 
void ab_reset_state(void)
{
    //==================================================
    // State entry 

    if (boat.state_entry)
    {
        boat.state_entry = CLEAR_BIT; 
    }

    //==================================================

    // Go directly to the init function 
    boat.init = SET_BIT; 

    // Clear fault and status codes 
    boat.fault_code = CLEAR; 
    m8q_set_reset_flag(); 
    nrf24l01_clear_status(); 

    //==================================================
    // State exit 

    if (boat.init)
    {
        boat.state_entry = SET_BIT; 
        boat.reset = CLEAR_BIT; 
    }

    //==================================================
}

//=======================================================================================
