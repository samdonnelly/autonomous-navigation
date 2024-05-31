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

#define AB_NUM_STATES 8              // Number of system states 
#define AB_INIT_DELAY 1000000        // Init state delay (us) 
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
            if (!init_flag)
            {
                next_state = AB_NOT_READY_STATE; 
            }
            break; 
        
        case AB_NOT_READY_STATE: 
            if (fault_flag)
            {
                next_state = AB_FAULT_STATE; 
            }
            else if (low_pwr_flag)
            {
                next_state = AB_LOW_PWR_STATE; 
            }
            else if (idle_flag)
            {
                next_state = AB_READY_STATE; 
            }
            break; 
        
        case AB_READY_STATE: 
            if (fault_flag)
            {
                next_state = AB_FAULT_STATE; 
            }
            else if (low_pwr_flag)
            {
                next_state = AB_LOW_PWR_STATE; 
            }
            else if (!ready_flag)
            {
                next_state = AB_NOT_READY_STATE; 
            }
            else if (manual_flag)
            {
                next_state = AB_MANUAL_STATE; 
            }
            else if (autonomous_flag)
            {
                next_state = AB_AUTO_STATE; 
            }
            break; 
        
        case AB_MANUAL_STATE: 
            if (fault_flag)
            {
                next_state = AB_FAULT_STATE; 
            }
            else if (low_pwr_flag)
            {
                next_state = AB_LOW_PWR_STATE; 
            }
            else if (!ready_flag)
            {
                next_state = AB_NOT_READY_STATE; 
            }
            else if (idle_flag)
            {
                next_state = AB_READY_STATE; 
            }
            break; 
        
        case AB_AUTO_STATE: 
            if (fault_flag)
            {
                next_state = AB_FAULT_STATE; 
            }
            else if (low_pwr_flag)
            {
                next_state = AB_LOW_PWR_STATE; 
            }
            else if (!ready_flag)
            {
                next_state = AB_NOT_READY_STATE; 
            }
            else if (idle_flag)
            {
                next_state = AB_READY_STATE; 
            }
            break; 
        
        case AB_LOW_PWR_STATE: 
            if (reset_flag)
            {
                next_state = AB_RESET_STATE; 
            }
            break; 
        
        case AB_FAULT_STATE: 
            if (reset_flag)
            {
                next_state = AB_RESET_STATE; 
            }
            break; 
        
        case AB_RESET_STATE: 
            if (init_flag)
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
    // if (nrf24l01_get_status())
    // {
    //     fault_code |= (SET_BIT << SHIFT_2); 
    // }

    // System requirements check. If these conditions are not met then take the system 
    // out of an active mode. 
    ready_flag = SET_BIT; 
    
    // GPS position lock check. If the system loses GPS position lock in manual mode then 
    // it continues on. 
    navstat = m8q_get_position_navstat_lock(); 

    if (navstat && (state != AB_MANUAL_STATE))
    {
        ready_flag = CLEAR_BIT; 
    }
    
    // Heartbeat check. If the system loses the heartbeat in autonomous mode then it 
    // continues on. 
    if ((!connect_status()) && (state != AB_AUTO_STATE))
    {
        ready_flag = CLEAR_BIT; 
    }
}

//=======================================================================================


//=======================================================================================
// State functions 

// Initialization state 
void ab_init_state(void)
{
    // State entry 
    if (boat_test.state_entry_flag)
    {
        boat_test.state_entry_flag = CLEAR_BIT; 

        // Set the thruster throttle to zero to initialize the ESCs and motors. 
        esc_readytosky_send(DEVICE_ONE, AB_NO_THRUST); 
        esc_readytosky_send(DEVICE_TWO, AB_NO_THRUST); 

        // Grab the current location (if available) and set the default target waypoint. 
        boat_test.current.lat = m8q_get_position_lat(); 
        boat_test.current.lon = m8q_get_position_lon(); 
        boat_test.target.lat = gps_waypoints[0].lat; 
        boat_test.target.lon = gps_waypoints[0].lon; 

        // Set up the file system 
    }

    // State exit 
    if (tim_compare(boat_test.timer_nonblocking, 
                    boat_test.state_timer.clk_freq, 
                    AB_INIT_DELAY, 
                    &boat_test.state_timer.time_cnt_total, 
                    &boat_test.state_timer.time_cnt, 
                    &boat_test.state_timer.time_start))
    {
        // Short delay to let the system set up before moving into the next state 

        boat_test.state_timer.time_start = SET_BIT; 
        boat_test.state_entry_flag = SET_BIT; 
        boat_test.init_flag = CLEAR_BIT; 
    }
}


// Not ready state 
void ab_not_ready_state(void)
{
    // State entry 
    if (boat_test.state_entry_flag)
    {
        boat_test.state_entry_flag = CLEAR_BIT; 

        // Update the LED strobe colour 
        boat_test.strobe_colour_set(ws2812_led_not_ready); 
    }

    // Wait for the system requirements to be met - ready flag will be set 

    // State exit 
    if (boat_test.fault_flag | boat_test.low_pwr_flag | boat_test.ready_flag)
    {
        boat_test.state_entry_flag = SET_BIT; 
        boat_test.idle_flag = SET_BIT; 
        boat_test.manual_flag = CLEAR_BIT; 
        boat_test.autonomous_flag = CLEAR_BIT; 

        // Make sure the LEDs are off and reset the strobe timer 
        boat_test.strobe_off(); 
    }
}


// Ready state 
void ab_ready_state(void)
{
    // State entry 
    if (boat_test.state_entry_flag)
    {
        boat_test.state_entry_flag = CLEAR_BIT; 

        // Update the LED strobe colour 
        boat_test.strobe_colour_set(ws2812_led_ready); 
    }

    // Wait for an active state to be chosen 

    // State exit 
    if (boat_test.fault_flag | boat_test.low_pwr_flag | !boat_test.ready_flag)
    {
        // Manual and autonomous mode exit conditions come from external commands 
        // received so they're not included here. 

        boat_test.state_entry_flag = SET_BIT; 
        boat_test.idle_flag = CLEAR_BIT; 

        // Make sure the LEDs are off and reset the strobe timer 
        boat_test.strobe_off(); 
    }
}


// Manual control mode state 
void ab_manual_state(void)
{
    // State entry 
    if (boat_test.state_entry_flag)
    {
        boat_test.state_entry_flag = CLEAR_BIT; 

        // Update the LED strobe colour 
        boat_test.strobe_colour_set(ws2812_led_manual_strobe); 
    }

    // External thruster control 
    boat_test.manual_mode(boat_test.mc_data, boat_test.cmd_id, boat_test.cmd_value); 

    // State exit 
    if (boat_test.fault_flag | boat_test.low_pwr_flag | !boat_test.ready_flag)
    {
        // The idle (ready) state exit condition comes from an external command 
        // received so it's not included here. 

        boat_test.state_entry_flag = SET_BIT; 
        boat_test.manual_flag = CLEAR_BIT; 

        // Stop manual mode and make sure the LEDs are off. 
        boat_test.manual_mode_exit(); 
        boat_test.strobe_off(); 
    }
}


// Autonomous mode state 
void ab_auto_state(void)
{
    // State entry 
    if (boat_test.state_entry_flag)
    {
        boat_test.state_entry_flag = CLEAR_BIT; 

        // Update the LED strobe colour 
        boat_test.strobe_colour_set(ws2812_led_auto_strobe); 
    }

    // Navigate the predefined waypoint mission autonomously 
    boat_test.auto_mode(); 

    // State exit 
    if (boat_test.fault_flag | boat_test.low_pwr_flag | !boat_test.ready_flag)
    {
        // The idle (ready) state exit condition comes from an external command 
        // received so it's not included here. 

        boat_test.state_entry_flag = SET_BIT; 
        boat_test.autonomous_flag = CLEAR_BIT; 

        // Stop auto mode and makes ure LEDs are off. 
        boat_test.auto_mode_exit(); 
        boat_test.strobe_off(); 
    }
}


// Low power state 
void ab_low_pwr_state(void)
{
    // State entry 
    if (boat_test.state_entry_flag)
    {
        boat_test.state_entry_flag = CLEAR_BIT; 

        // Put devices in low power mode 
        m8q_set_low_pwr_flag(); 
    }

    // Wait for a system reset. Currently the system will hang here until it's power 
    // cycled because there is no setter for the reset flag. This is ok because if the 
    // battery voltage is low the system should not be doing anything. 

    // State exit 
    if (boat_test.reset_flag)
    {
        boat_test.state_entry_flag = SET_BIT; 

        // Take devices out of low power mode 
        m8q_clear_low_pwr_flag(); 
    }
}


// Fault state 
void ab_fault_state(void)
{
    // State entry 
    if (boat_test.state_entry_flag)
    {
        boat_test.state_entry_flag = CLEAR_BIT; 
    }

    // Go directly to the reset state 
    boat_test.reset_flag = SET_BIT; 

    // State exit 
    if (boat_test.reset_flag)
    {
        boat_test.state_entry_flag = SET_BIT; 
        boat_test.fault_flag = CLEAR_BIT; 
    }
}


// Reset state 
void ab_reset_state(void)
{
    // State entry 
    if (boat_test.state_entry_flag)
    {
        boat_test.state_entry_flag = CLEAR_BIT; 
    }

    // Go directly to the init state function. Clear and fault and status codes before 
    // doing so. 
    boat_test.init_flag = SET_BIT; 
    boat_test.fault_code = CLEAR; 
    m8q_set_reset_flag(); 
    // nrf24l01_clear_status(); 

    // State exit 
    if (boat_test.init_flag)
    {
        boat_test.state_entry_flag = SET_BIT; 
        boat_test.reset_flag = CLEAR_BIT; 
    }
}

//=======================================================================================
