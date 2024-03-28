/**
 * @file radio_comm.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief AB radio control 
 * 
 * @version 0.1
 * @date 2024-03-14
 * 
 * @copyright Copyright (c) 2024
 * 
 */

//=======================================================================================
// Includes 

#include "radio_comm.h" 
#include "ab_interface.h" 
#include "led_control.h" 
#include "gps_coordinates.h" 

//=======================================================================================


//=======================================================================================
// Macros 

// System 
#define AB_NUM_CMDS 9                // Total number of external commands available 

// Timing 
#define AB_HB_PERIOD 500000          // Time between heartbeat checks (us) 
#define AB_HB_TIMEOUT 30             // period*timeout = time before conection lost status 

// Navigation 
#define AB_GPS_INDEX_CNT 3           // Successive index command count needed to update 

//=======================================================================================


//=======================================================================================
// Structs 

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
// Prototypes 

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

//=======================================================================================


//=======================================================================================
// Data 

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
// Setup and teardown 

// Constructor 
boat_radio_comms::boat_radio_comms(TIM_TypeDef *timer)
    : timer_nonblocking(timer), 
      hb_timeout(CLEAR), 
      pipe(NRF24L01_DP_1), 
      connect_flag(CLEAR), 
      cmd_value(CLEAR) 
{
    // Timing 
    memset((void *)&hb_timer, CLEAR, sizeof(hb_timer)); 
    hb_timer.time_start = SET_BIT; 

    // Payload 
    memset((void *)read_buff, CLEAR, sizeof(read_buff)); 
    memset((void *)cmd_id, CLEAR, sizeof(cmd_id)); 
}

// Destructor 
boat_radio_comms::~boat_radio_comms() {}

//=======================================================================================


//=======================================================================================
// Radio communication functions 

// Checks for radio communication 
void boat_radio_comms::radio_comm_check(uint8_t state)
{
    // Heartbeat check 
    // Increment the timeout counter periodically until the timeout limit at which 
    // point the system assumes to have lost radio connection. Connection status is 
    // re-established once a HB command is seen. 
    if (tim_compare(timer_nonblocking, 
                    hb_timer.clk_freq, 
                    AB_HB_PERIOD, 
                    &hb_timer.time_cnt_total, 
                    &hb_timer.time_cnt, 
                    &hb_timer.time_start))
    {
        if (hb_timeout >= AB_HB_TIMEOUT)
        {
            connect_flag = CLEAR_BIT; 
        }
        else 
        {
            hb_timeout++; 
        }
    }
    
    // External command check 
    // Check if a payload has been received 
    if (nrf24l01_data_ready_status(pipe))
    {
        // Payload has been received. Read the payload from the device RX FIFO. 
        nrf24l01_receive_payload(read_buff, pipe); 

        // Validate the input - parse into an ID and value if valid 
        if (command_parse(&read_buff[1]))
        {
            // Valid input - compare the ID to each of the available pre-defined commands 
            for (uint8_t i = CLEAR; i < AB_NUM_CMDS; i++) 
            {
                // Check that the command is available for the state before comparing it 
                // against the ID. 
                if (cmd_table[i].ab_cmd_mask & (SET_BIT << state))
                {
                    // Command available. Compare with the ID. 
                    if (str_compare(cmd_table[i].ab_cmd, (char *)cmd_id, BYTE_0)) 
                    {
                        // ID matched to a command. Execute the command. 
                        (cmd_table[i].ab_cmd_func_ptr)(cmd_value); 
                        connect_flag = SET_BIT; 
                        hb_timeout = CLEAR; 
                        break; 
                    }
                }
            }
        }

        memset((void *)read_buff, CLEAR, sizeof(read_buff)); 
    }
}


// Parse the ground station command into an ID and value 
uint8_t boat_radio_comms::command_parse(uint8_t *cmd_buff)
{
    uint8_t id_flag = SET_BIT; 
    uint8_t id_index = CLEAR; 
    uint8_t data = CLEAR; 
    uint8_t cmd_value_str[AB_MAX_CMD_SIZE]; 
    uint8_t value_size = CLEAR; 

    // Initialize data 
    memset((void *)cmd_id, CLEAR, sizeof(cmd_id)); 
    cmd_value = CLEAR; 
    memset((void *)cmd_value_str, CLEAR, sizeof(cmd_value_str)); 

    // Parse the command into an ID and value 
    for (uint8_t i = CLEAR; cmd_buff[i] != NULL_CHAR; i++)
    {
        data = cmd_buff[i]; 

        if (id_flag)
        {
            // cmd ID parsing 

            id_index = i; 

            // Check that the command byte is within range 
            if ((data >= A_LO_CHAR && data <= Z_LO_CHAR) || 
                (data >= A_UP_CHAR && data <= Z_UP_CHAR))
            {
                // Valid character byte seen 
                cmd_id[i] = data; 
            }
            else if (data >= ZERO_CHAR && data <= NINE_CHAR)
            {
                // Valid digit character byte seen 
                id_flag = CLEAR_BIT; 
                cmd_id[i] = NULL_CHAR; 
                cmd_value_str[i-id_index] = data; 
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
                cmd_value_str[i-id_index] = data; 
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
        cmd_value += (uint8_t)char_to_int(cmd_value_str[i], value_size-i-1); 
    }

    return TRUE; 
}


// Connection status 
uint8_t boat_radio_comms::connect_status(void)
{
    return connect_flag; 
}

//=======================================================================================


//=======================================================================================
// Command functions 

// Idle command 
void ab_idle_cmd(uint8_t idle_cmd_value)
{
    boat_test.idle_flag = SET_BIT; 
    boat_test.state_entry_flag = SET_BIT; 
    boat_test.manual_flag = CLEAR_BIT; 
    boat_test.autonomous_flag = CLEAR_BIT; 

    // Stop thrusters and make sure the LEDs are off. 
    boat_test.auto_mode_exit(); 
    boat_test.manual_mode_exit(); 
    boat_test.strobe_off(); 
}


// Manual control mode command 
void ab_manual_cmd(uint8_t manual_cmd_value)
{
    boat_test.manual_flag = SET_BIT; 
    boat_test.state_entry_flag = SET_BIT; 
    boat_test.idle_flag = CLEAR_BIT; 

    // Make sure the LEDs are off and reset the strobe timer 
    boat_test.strobe_off(); 
}


// Autonomous mode command 
void ab_auto_cmd(uint8_t auto_cmd_value)
{
    boat_test.autonomous_flag = SET_BIT; 
    boat_test.state_entry_flag = SET_BIT; 
    boat_test.idle_flag = CLEAR_BIT; 

    // Make sure the LEDs are off and reset the strobe timer 
    boat_test.strobe_off(); 
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
        boat_test.waypoint_index = index_cmd_value; 
        boat_test.target.lat = gps_waypoints[boat_test.waypoint_index].lat; 
        boat_test.target.lon = gps_waypoints[boat_test.waypoint_index].lon; 
        index_check = CLEAR; 
    }
}


// Manual throttle command 
void ab_throttle_cmd(uint8_t throttle_cmd_value)
{
    boat_test.mc_data = SET_BIT; 
}


// Heartbeat command 
void ab_hb_cmd(uint8_t hb_cmd_value)
{
    // This function is here to provide a match for a valid command check. It's meant to 
    // verify that there is a radio connection but the connection flag gets set when any 
    // command matches. 
}

//=======================================================================================
