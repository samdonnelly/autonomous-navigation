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
#define AB_NUM_CMDS 9                // Total number of external commands available 

// Timing 
#define AB_INIT_DELAY 1000000        // Init state delay (us) 
#define AB_HB_PERIOD 500000          // Time between heartbeat checks (us) 
#define AB_HB_TIMEOUT 30             // period*timeout = time before conection lost status 

// Navigation 
#define AB_GPS_INDEX_CNT 3           // Successive index command count needed to update 

// Manual Control 
#define AB_MC_LEFT_MOTOR 0x4C        // "L" character that indicates left motor 
#define AB_MC_RIGHT_MOTOR 0x52       // "R" character that indicates right motor 
#define AB_MC_FWD_THRUST 0x50        // "P" (plus) - indicates forward thrust 
#define AB_MC_REV_THRUST 0x4D        // "M" (minus) - indicates reverse thrust 
#define AB_MC_NEUTRAL 0x4E           // "N" (neutral) - indicates neutral gear or zero thrust 

//=======================================================================================


//=======================================================================================
// Structures 

// Ground station commands 
typedef struct ab_cmds_s 
{
    char ab_cmd[AB_MAX_CMD_SIZE]; 
    void (*ab_cmd_func_ptr)(uint8_t); 
    uint8_t ab_cmd_mask; 
}
ab_cmds_t; 

//=======================================================================================


//=======================================================================================
// Clases 

// Data record instance 
Boat ab_data(TIM9, ADC1); 

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

//==================================================
// State functions 

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

//==================================================

//==================================================
// System 

/**
 * @brief System checks 
 * 
 * @details Checks for faults and loss of critical data such as GPS location and radio 
 *          communication. 
 */
void ab_system_checks(void); 

//==================================================

//==================================================
// Radio command functions 

/**
 * @brief Checks for radio communication 
 * 
 * @details Checks the radio heartbeat to see if the boat can still talk to the ground 
 *          station, and checks if there are any new commands to be read. If there is 
 *          new input then the input is checked against the predefined commands and if 
 *          a macth is found then the command function is called. 
 */
void ab_radio_comm_check(void); 


/**
 * @brief Parse the user command into an ID and value 
 * 
 * @details Takes a radio message received from the ground station and parses it into 
 *          an ID and payload. If the ID and payload are of a valid format then the 
 *          function will return true. Note that a payload is not needed for all 
 *          commands. See the 'cmd_table' for a list of available commands/IDs and the 
 *          states in which they're used. 
 * 
 * @param command_buffer : radio message string 
 * @return uint8_t : status of the message parsing 
 */
uint8_t ab_parse_cmd(uint8_t *command_buffer); 


/**
 * @brief Idle command 
 * 
 * @details Run when the ground station sends an "idle" command while in an applicable 
 *          state. This will trigger idle mode and turn the thrusters off. See the 
 *          "cmd_table" for states in which this command is valid. 
 * 
 * @param idle_cmd_value : generic idle command argument 
 */
void ab_idle_cmd(uint8_t idle_cmd_value); 


/**
 * @brief Manual control mode command 
 * 
 * @details Run when the ground station sends a "manual" command while in an applicable 
 *          state. This will trigger manual control mode. See the "cmd_table" for states 
 *          in which this command is valid. 
 * 
 * @param manual_cmd_value : generic manual mode command argument 
 */
void ab_manual_cmd(uint8_t manual_cmd_value); 


/**
 * @brief Autonomous mode command 
 * 
 * @details Run when the ground station sends an "auto" command while in an applicable 
 *          state. Puts the system in autonomous mode. See the "cmd_table" for states in 
 *          which this command is valid. 
 * 
 * @param auto_cmd_value : generic autonomous mode command argument 
 */
void ab_auto_cmd(uint8_t auto_cmd_value); 


/**
 * @brief Index update command 
 * 
 * @details Run when the ground station sends an "index" command while in an applicable 
 *          state. The payload of the index command ("index <payload>" - passed as the 
 *          'index_cmd_value' argument) contains the index used to set the target 
 *          location within the pre-defined waypoint mission. This function will verify 
 *          the index value and update the target location if the index is valid. See 
 *          the "cmd_table" for states in which this command is valid. 
 * 
 * @param index_cmd_value : generic index update command argument 
 */
void ab_index_cmd(uint8_t index_cmd_value); 


/**
 * @brief Manual throttle command 
 * 
 * @details Run when the ground station sends an "RP", "RN", "LP", or "LN" command while 
 *          in an applicable state. These indicate right or left thruster as well as 
 *          positive (forward) and negative (reverse) thrust. Each command will be 
 *          followed by a payload (ex. "RP <payload>") that indicates the thruster 
 *          command (%), however this payload is used in the "manual" state function and 
 *          not here. This function will indicate when a manual control message has been 
 *          received and will reset the heartbeat timeout counter. See the "cmd_table" 
 *          for states in which this command is valid. 
 * 
 * @param throttle_cmd_value : generic manual throttle command argument 
 */
void ab_throttle_cmd(uint8_t throttle_cmd_value); 


/**
 * @brief Heartbeat command 
 * 
 * @details Run when the ground station sends an "ping" command while in an applicable 
 *          state. This function resets the heartbeat timeout counter. The ground 
 *          station will periodically send a "ping" and the boat uses this to know if 
 *          it still has radio communication with the ground station. See the "cmd_table"
 *          for states in which this command is valid. 
 * 
 * @param hb_cmd_value : generic heartbeat command argument 
 */
void ab_hb_cmd(uint8_t hb_cmd_value); 

//==================================================

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


// Ground station command table 
static ab_cmds_t cmd_table[AB_NUM_CMDS] = 
{
    {"idle",   &ab_idle_cmd,     (SET_BIT << AB_MANUAL_STATE)    | 
                                 (SET_BIT << AB_AUTO_STATE)}, 
    {"manual", &ab_manual_cmd,   (SET_BIT << AB_READY_STATE)}, 
    {"auto",   &ab_auto_cmd,     (SET_BIT << AB_READY_STATE)}, 
    {"index",  &ab_index_cmd,    (SET_BIT << AB_READY_STATE)     | 
                                 (SET_BIT << AB_AUTO_STATE)}, 
    {"RP",     &ab_throttle_cmd, (SET_BIT << AB_MANUAL_STATE)}, 
    {"RN",     &ab_throttle_cmd, (SET_BIT << AB_MANUAL_STATE)}, 
    {"LP",     &ab_throttle_cmd, (SET_BIT << AB_MANUAL_STATE)}, 
    {"LN",     &ab_throttle_cmd, (SET_BIT << AB_MANUAL_STATE)}, 
    {"ping",   &ab_hb_cmd,       (SET_BIT << AB_INIT_STATE)      | 
                                 (SET_BIT << AB_NOT_READY_STATE) | 
                                 (SET_BIT << AB_READY_STATE)     | 
                                 (SET_BIT << AB_MANUAL_STATE)    | 
                                 (SET_BIT << AB_AUTO_STATE)      | 
                                 (SET_BIT << AB_LOW_PWR_STATE)   | 
                                 (SET_BIT << AB_FAULT_STATE)     | 
                                 (SET_BIT << AB_RESET_STATE)} 
}; 

//=======================================================================================


//=======================================================================================
// Autonomous boat application 

void ab_app(void)
{
    // Autonomous boat application code here 

    ab_states_t next_state = ab_data.state; 

    //==================================================
    // System checks and updates 

    // Check for critical info 
    ab_system_checks(); 

    // Check radio communication 
    ab_radio_comm_check(); 

    // Update the LED strobe 
    ab_data.strobe(); 

    //==================================================

    //==================================================
    // System state machine 

    switch (next_state)
    {
        case AB_INIT_STATE: 
            if (!ab_data.init)
            {
                next_state = AB_NOT_READY_STATE; 
            }
            break; 
        
        case AB_NOT_READY_STATE: 
            if (ab_data.fault)
            {
                next_state = AB_FAULT_STATE; 
            }
            else if (ab_data.low_pwr)
            {
                next_state = AB_LOW_PWR_STATE; 
            }
            else if (ab_data.idle)
            {
                next_state = AB_READY_STATE; 
            }
            break; 
        
        case AB_READY_STATE: 
            if (ab_data.fault)
            {
                next_state = AB_FAULT_STATE; 
            }
            else if (ab_data.low_pwr)
            {
                next_state = AB_LOW_PWR_STATE; 
            }
            else if (!ab_data.ready)
            {
                next_state = AB_NOT_READY_STATE; 
            }
            else if (ab_data.manual)
            {
                next_state = AB_MANUAL_STATE; 
            }
            else if (ab_data.autonomous)
            {
                next_state = AB_AUTO_STATE; 
            }
            break; 
        
        case AB_MANUAL_STATE: 
            if (ab_data.fault)
            {
                next_state = AB_FAULT_STATE; 
            }
            else if (ab_data.low_pwr)
            {
                next_state = AB_LOW_PWR_STATE; 
            }
            else if (!ab_data.ready)
            {
                next_state = AB_NOT_READY_STATE; 
            }
            else if (ab_data.idle)
            {
                next_state = AB_READY_STATE; 
            }
            break; 
        
        case AB_AUTO_STATE: 
            if (ab_data.fault)
            {
                next_state = AB_FAULT_STATE; 
            }
            else if (ab_data.low_pwr)
            {
                next_state = AB_LOW_PWR_STATE; 
            }
            else if (!ab_data.ready)
            {
                next_state = AB_NOT_READY_STATE; 
            }
            else if (ab_data.idle)
            {
                next_state = AB_READY_STATE; 
            }
            break; 
        
        case AB_LOW_PWR_STATE: 
            if (ab_data.reset)
            {
                next_state = AB_RESET_STATE; 
            }
            break; 
        
        case AB_FAULT_STATE: 
            if (ab_data.reset)
            {
                next_state = AB_RESET_STATE; 
            }
            break; 
        
        case AB_RESET_STATE: 
            if (ab_data.init)
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
    ab_data.state = next_state; 

    // Run the controllers 
    m8q_controller(); 
}

//=======================================================================================


//=======================================================================================
// State functions 

// Initialization state 
void ab_init_state(void)
{
    //==================================================
    // State entry 

    if (ab_data.state_entry)
    {
        ab_data.state_entry = CLEAR_BIT; 

        // Set the thruster throttle to zero to initialize the ESCs and motors. 
        esc_readytosky_send(DEVICE_ONE, AB_NO_THRUST); 
        esc_readytosky_send(DEVICE_TWO, AB_NO_THRUST); 

        // Grab the current location (if available) and set the default target waypoint. 
        ab_data.current.lat = m8q_get_position_lat(); 
        ab_data.current.lon = m8q_get_position_lon(); 
        ab_data.target.lat = gps_waypoints[0].lat; 
        ab_data.target.lon = gps_waypoints[0].lon; 
    }

    //==================================================

    // Set up the file system 

    //==================================================
    // State exit 

    // Short delay to let the system set up before moving into the next state 
    if (tim_compare(ab_data.timer_nonblocking, 
                    ab_data.delay_timer.clk_freq, 
                    AB_INIT_DELAY, 
                    &ab_data.delay_timer.time_cnt_total, 
                    &ab_data.delay_timer.time_cnt, 
                    &ab_data.delay_timer.time_start))
    {
        ab_data.delay_timer.time_start = SET_BIT; 
        ab_data.state_entry = SET_BIT; 
        ab_data.init = CLEAR_BIT; 
    }

    //==================================================
}


// Not ready state 
void ab_not_ready_state(void)
{
    //==================================================
    // State entry 
    
    if (ab_data.state_entry)
    {
        ab_data.state_entry = CLEAR_BIT; 

        // Update the LED strobe colour 
        ab_data.strobe_colour_set(ws2812_led_not_ready); 
    }

    //==================================================

    // Wait for the system requirements to be met - ready flag will be set 

    //==================================================
    // State exit 

    if (ab_data.fault | ab_data.low_pwr | ab_data.ready)
    {
        ab_data.state_entry = SET_BIT; 
        ab_data.idle = SET_BIT; 
        ab_data.manual = CLEAR_BIT; 
        ab_data.autonomous = CLEAR_BIT; 

        // Make sure the LEDs are off and reset the strobe timer 
        ab_data.strobe_off(); 
    }

    //==================================================
}


// Ready state 
void ab_ready_state(void)
{
    //==================================================
    // State entry 

    if (ab_data.state_entry)
    {
        ab_data.state_entry = CLEAR_BIT; 

        // Update the LED strobe colour 
        ab_data.strobe_colour_set(ws2812_led_ready); 
    }

    //==================================================

    // Wait for an active state to be chosen 

    //==================================================
    // State exit 

    // Manual and autonomous mode exit conditions come from external commands received so 
    // they're not included here. 

    if (ab_data.fault | ab_data.low_pwr | !ab_data.ready)
    {
        ab_data.state_entry = SET_BIT; 
        ab_data.idle = CLEAR_BIT; 

        // Make sure the LEDs are off and reset the strobe timer 
        ab_data.strobe_off(); 
    }

    //==================================================
}


// Manual control mode state 
void ab_manual_state(void)
{
    int16_t cmd_value = CLEAR; 

    //==================================================
    // State entry 

    if (ab_data.state_entry)
    {
        ab_data.state_entry = CLEAR_BIT; 

        // Update the LED strobe colour 
        ab_data.strobe_colour_set(ws2812_led_manual_strobe); 
    }

    //==================================================

    //==================================================
    // External thruster control 

    // Only attempt a throttle command update if a new data command was received 
    if (ab_data.mc_data)
    {
        ab_data.mc_data = CLEAR_BIT; 

        // Check that the command matches a valid throttle command. If it does then update 
        // the thruster command. 
        
        cmd_value = (int16_t)ab_data.cmd_value; 

        if (ab_data.cmd_id[0] == AB_MC_RIGHT_MOTOR)
            {
                switch (ab_data.cmd_id[1])
                {
                    case AB_MC_FWD_THRUST: 
                        ab_data.right_thruster += (cmd_value - ab_data.right_thruster) >> SHIFT_3; 
                        esc_readytosky_send(DEVICE_ONE, ab_data.right_thruster); 
                        break; 
                    
                    case AB_MC_REV_THRUST: 
                        ab_data.right_thruster += ((~cmd_value + 1) - ab_data.right_thruster) >> SHIFT_3; 
                        esc_readytosky_send(DEVICE_ONE, ab_data.right_thruster); 
                        break; 
                    
                    case AB_MC_NEUTRAL: 
                        if (cmd_value == AB_NO_THRUST)
                        {
                            ab_data.right_thruster = AB_NO_THRUST; 
                            esc_readytosky_send(DEVICE_ONE, ab_data.right_thruster); 
                        }
                        break; 
                    
                    default: 
                        break; 
                }
            }
            else if (ab_data.cmd_id[0] == AB_MC_LEFT_MOTOR)
            {
                switch (ab_data.cmd_id[1])
                {
                    case AB_MC_FWD_THRUST: 
                        ab_data.left_thruster += (cmd_value - ab_data.left_thruster) >> SHIFT_3; 
                        esc_readytosky_send(DEVICE_TWO, ab_data.left_thruster); 
                        break; 
                    
                    case AB_MC_REV_THRUST: 
                        ab_data.left_thruster += ((~cmd_value + 1) - ab_data.left_thruster) >> SHIFT_3; 
                        esc_readytosky_send(DEVICE_TWO, ab_data.left_thruster); 
                        break; 
                    
                    case AB_MC_NEUTRAL: 
                        if (cmd_value == AB_NO_THRUST)
                        {
                            ab_data.left_thruster = AB_NO_THRUST; 
                            esc_readytosky_send(DEVICE_TWO, ab_data.left_thruster); 
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

    if (ab_data.fault | ab_data.low_pwr | !ab_data.ready)
    {
        ab_data.state_entry = SET_BIT; 
        ab_data.manual = CLEAR_BIT; 

        // Set the throttle to zero to stop the thrusters 
        esc_readytosky_send(DEVICE_ONE, AB_NO_THRUST); 
        esc_readytosky_send(DEVICE_TWO, AB_NO_THRUST); 

        // Make sure the LEDs are off and reset the strobe timer 
        ab_data.strobe_off(); 
    }

    //==================================================
}


// Autonomous mode state 
void ab_auto_state(void)
{
    // static uint8_t nav_period_counter = CLEAR; 

    //==================================================
    // State entry 

    if (ab_data.state_entry)
    {
        ab_data.state_entry = CLEAR_BIT; 

        // Update the LED strobe colour 
        ab_data.strobe_colour_set(ws2812_led_auto_strobe); 
    }

    //==================================================

    //==================================================
    // Navigation calculations 

    ab_data.auto_mode(); 

    //==================================================

    //==================================================
    // State exit 

    // The idle (ready) state exit condition comes from an external command received 
    // so it's not included here. 

    if (ab_data.fault | ab_data.low_pwr | !ab_data.ready)
    {
        ab_data.state_entry = SET_BIT; 
        ab_data.autonomous = CLEAR_BIT; 

        // Stop auto mode 
        ab_data.auto_mode_exit(); 

        // Make sure the LEDs are off and reset the strobe timer 
        ab_data.strobe_off(); 
    }

    //==================================================
}


// Low power state 
void ab_low_pwr_state(void)
{
    //==================================================
    // State entry 

    if (ab_data.state_entry)
    {
        ab_data.state_entry = CLEAR_BIT; 

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

    if (ab_data.reset)
    {
        ab_data.state_entry = SET_BIT; 

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

    if (ab_data.state_entry)
    {
        ab_data.state_entry = CLEAR_BIT; 
    }

    //==================================================

    // Go directly to the reset state 
    ab_data.reset = SET_BIT; 

    //==================================================
    // State exit 

    if (ab_data.reset)
    {
        ab_data.state_entry = SET_BIT; 
        ab_data.fault = CLEAR_BIT; 
    }

    //==================================================
}


// Reset state 
void ab_reset_state(void)
{
    //==================================================
    // State entry 

    if (ab_data.state_entry)
    {
        ab_data.state_entry = CLEAR_BIT; 
    }

    //==================================================

    // Go directly to the init function 
    ab_data.init = SET_BIT; 

    // Clear fault and status codes 
    ab_data.fault_code = CLEAR; 
    m8q_set_reset_flag(); 
    nrf24l01_clear_status(); 

    //==================================================
    // State exit 

    if (ab_data.init)
    {
        ab_data.state_entry = SET_BIT; 
        ab_data.reset = CLEAR_BIT; 
    }

    //==================================================
}

//=======================================================================================


//=======================================================================================
// System 

// System checks 
void ab_system_checks(void)
{
    //==================================================
    // Device status checks 

    if (m8q_get_fault_code())
    {
        ab_data.fault_code |= (SET_BIT << SHIFT_0); 
    }
    if (nrf24l01_get_status())
    {
        ab_data.fault_code |= (SET_BIT << SHIFT_2); 
    }

    //==================================================

    //==================================================
    // System requirements check 

    // If these conditions are not met (excluding radio connection when in autonomous 
    // mode) then take the system out of an active mode. 

    ab_data.ready = SET_BIT; 

    // Voltages 
    // Set low power flag is voltage is below a threshold (but not zero because that means 
    // the voltage source is not present). 
    // If the low power flag gets set then the threshold to clear the flag has to be higher 
    // than the one used to set the flag. 
    
    // GPS position lock check 
    // If the system loses GPS position lock in manual mode then it continues on. 
    ab_data.navstat = m8q_get_position_navstat_lock(); 

    if (ab_data.navstat && (ab_data.state != AB_MANUAL_STATE))
    {
        ab_data.ready = CLEAR_BIT; 
    }
    
    // Heartbeat check 
    // If the system loses the heartbeat in autonomous mode then it continues on. 
    if ((!ab_data.connect) && (ab_data.state != AB_AUTO_STATE))
    {
        ab_data.ready = CLEAR_BIT; 
    }

    //==================================================
}

//=======================================================================================


//=======================================================================================
// Radio command functions 

// Checks for radio communication 
void ab_radio_comm_check(void)
{
    // Heartbeat check 
    // Increment the timeout counter periodically until the timeout limit at which 
    // point the system assumes to have lost radio connection. Connection status is 
    // re-established once a HB command is seen. 
    if (tim_compare(ab_data.timer_nonblocking, 
                    ab_data.hb_timer.clk_freq, 
                    AB_HB_PERIOD, 
                    &ab_data.hb_timer.time_cnt_total, 
                    &ab_data.hb_timer.time_cnt, 
                    &ab_data.hb_timer.time_start))
    {
        if (ab_data.hb_timeout >= AB_HB_TIMEOUT)
        {
            ab_data.connect = CLEAR_BIT; 
        }
        else 
        {
            ab_data.hb_timeout++; 
        }
    }
    
    // External command check 
    // Check if a payload has been received 
    if (nrf24l01_data_ready_status(ab_data.pipe))
    {
        // Payload has been received. Read the payload from the device RX FIFO. 
        nrf24l01_receive_payload(ab_data.read_buff, ab_data.pipe); 

        // Validate the input - parse into an ID and value if valid 
        if (ab_parse_cmd(&ab_data.read_buff[1]))
        {
            // Valid input - compare the ID to each of the available pre-defined commands 
            for (uint8_t i = CLEAR; i < AB_NUM_CMDS; i++) 
            {
                // Check that the command is available for the state before comparing it 
                // against the ID. 
                if (cmd_table[i].ab_cmd_mask & (SET_BIT << ab_data.state))
                {
                    // Command available. Compare with the ID. 
                    if (str_compare(cmd_table[i].ab_cmd, (char *)ab_data.cmd_id, BYTE_0)) 
                    {
                        // ID matched to a command. Execute the command. 
                        (cmd_table[i].ab_cmd_func_ptr)(ab_data.cmd_value); 
                        break; 
                    }
                }
            }
        }

        memset((void *)ab_data.read_buff, CLEAR, sizeof(ab_data.read_buff)); 
    }
}


// Parse the ground station command into an ID and value 
uint8_t ab_parse_cmd(uint8_t *command_buffer)
{
    uint8_t id_flag = SET_BIT; 
    uint8_t id_index = CLEAR; 
    uint8_t data = CLEAR; 
    uint8_t cmd_value[AB_MAX_CMD_SIZE]; 
    uint8_t value_size = CLEAR; 

    // Initialize data 
    memset((void *)ab_data.cmd_id, CLEAR, sizeof(ab_data.cmd_id)); 
    ab_data.cmd_value = CLEAR; 
    memset((void *)cmd_value, CLEAR, sizeof(cmd_value)); 

    // Parse the command into an ID and value 
    for (uint8_t i = CLEAR; command_buffer[i] != NULL_CHAR; i++)
    {
        data = command_buffer[i]; 

        if (id_flag)
        {
            // cmd ID parsing 

            id_index = i; 

            // Check that the command byte is within range 
            if ((data >= A_LO_CHAR && data <= Z_LO_CHAR) || 
                (data >= A_UP_CHAR && data <= Z_UP_CHAR))
            {
                // Valid character byte seen 
                ab_data.cmd_id[i] = data; 
            }
            else if (data >= ZERO_CHAR && data <= NINE_CHAR)
            {
                // Valid digit character byte seen 
                id_flag = CLEAR_BIT; 
                ab_data.cmd_id[i] = NULL_CHAR; 
                cmd_value[i-id_index] = data; 
                value_size++; 
            }
            else 
            {
                // Valid data not seen 
                return FALSE; 
            }
        }
        else 
        {
            // cmd value parsing 

            if (data >= ZERO_CHAR && data <= NINE_CHAR)
            {
                // Valid digit character byte seen 
                cmd_value[i-id_index] = data; 
                value_size++; 
            }
            else 
            {
                // Valid data not seen 
                return FALSE; 
            }
        }
    }

    // Calculate the cmd value 
    for (uint8_t i = CLEAR; i < value_size; i++)
    {
        ab_data.cmd_value += (uint8_t)char_to_int(cmd_value[i], value_size-i-1); 
    }

    return TRUE; 
}


// Idle command 
void ab_idle_cmd(uint8_t idle_cmd_value)
{
    ab_data.idle = SET_BIT; 
    ab_data.state_entry = SET_BIT; 
    ab_data.manual = CLEAR_BIT; 
    ab_data.autonomous = CLEAR_BIT; 

    // Stop thrusters and reset timer 
    ab_data.auto_mode_exit(); 

    // Make sure the LEDs are off and reset the strobe timer 
    ab_data.strobe_off(); 
}


// Manual control mode command 
void ab_manual_cmd(uint8_t manual_cmd_value)
{
    ab_data.manual = SET_BIT; 
    ab_data.state_entry = SET_BIT; 
    ab_data.idle = CLEAR_BIT; 

    // Make sure the LEDs are off and reset the strobe timer 
    ab_data.strobe_off(); 
}


// Autonomous mode command 
void ab_auto_cmd(uint8_t auto_cmd_value)
{
    ab_data.autonomous = SET_BIT; 
    ab_data.state_entry = SET_BIT; 
    ab_data.idle = CLEAR_BIT; 

    // Make sure the LEDs are off and reset the strobe timer 
    ab_data.strobe_off(); 
}


// Index update command 
void ab_index_cmd(uint8_t index_cmd_value)
{
    static uint8_t index_check = CLEAR; 
    static uint8_t index_last = CLEAR; 

    // Compare the previous index command to the new index command. The radio messages 
    // between the ground station and boat are poor meaning a complete and correct 
    // message often does not get transmitted and received successfully. This can lead 
    // to the index not being updated to the desired value and therefore the boat moving 
    // to a target it's not supposed to. To combat this, the index has to been seen 
    // successively at least "AB_GPS_INDEX_CNT" times before the index will be updated. 
    if (index_cmd_value != index_last)
    {
        index_last = index_cmd_value; 
        index_check = SET_BIT; 
    }
    else 
    {
        index_check++; 
    }

    // Check that the index is within bounds and seen the last AB_GPS_INDEX_CNT times before 
    // updating the index (filters noise). 
    if ((index_cmd_value < NUM_GPS_WAYPOINTS) && (index_check >= AB_GPS_INDEX_CNT))
    {
        ab_data.waypoint_index = index_cmd_value; 
        ab_data.target.lat = gps_waypoints[ab_data.waypoint_index].lat; 
        ab_data.target.lon = gps_waypoints[ab_data.waypoint_index].lon; 
        index_check = CLEAR; 
    }
}


// Manual throttle command 
void ab_throttle_cmd(uint8_t throttle_cmd_value)
{
    ab_data.mc_data = SET_BIT; 
    ab_data.connect = SET_BIT; 
    ab_data.hb_timeout = CLEAR; 
}


// Heartbeat command 
void ab_hb_cmd(uint8_t hb_cmd_value)
{
    ab_data.connect = SET_BIT; 
    ab_data.hb_timeout = CLEAR; 
}

//=======================================================================================
