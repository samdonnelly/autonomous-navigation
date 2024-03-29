/**
 * @file project_init.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Ground station initialization code 
 * 
 * @version 0.1
 * @date 2023-07-20
 * 
 * @copyright Copyright (c) 2023
 * 
 */

//=======================================================================================
// Includes 

#include "gs_interface.h" 
#include "stm32f4xx_it.h" 

//=======================================================================================


//=======================================================================================
// Macros 

// System info 
#define GS_NUM_STATES 3              // Total number of system states 

// Timing 
#define GS_HB_PERIOD 500000          // Time between heartbeat checks (us) 
#define GS_MC_PERIOD 50000           // Time between throttle command sends (us) 
#define GS_CMD_PERIOD 250000         // Time between command sends checks (us) 

// User commands 
#define GS_MAX_USER_INPUT 30         // Max data size for user input (bytes) 
#define GS_NUM_CMDS 7                // Total number of external commands available 

// Data sizes 
#define GS_ADC_BUFF_SIZE 2           // Number of ADCs used 
#define GS_CMD_SEND_COUNT 10         // Number of times a command gets sent 

// Thrusters 
#define GS_MC_LEFT_MOTOR 0x4C        // "L" character that indicates left motor 
#define GS_MC_RIGHT_MOTOR 0x52       // "R" character that indicates right motor 
#define GS_MC_FWD_THRUST 0x50        // "P" (plus) - indicates forward thrust 
#define GS_MC_REV_THRUST 0x4D        // "M" (minus) - indicates reverse thrust 
#define GS_MC_NEUTRAL 0x4E           // "N" (neutral) - indicates neutral gear or zero thrust 
#define GS_NO_THRUST 0               // Force thruster output to zero 
#define GS_ADC_REV_LIM 100           // ADC value reverse command limit 
#define GS_ADC_FWD_LIM 155           // ADC value forward command limit 

//=======================================================================================


//=======================================================================================
// Enums 

// Ground station states 
typedef enum {
    GS_HB_STATE,             // State 0: heartbeat 
    GS_MC_STATE,             // State 1: manual control 
    GS_CMD_STATE,            // State 2: command send 
} gs_states_t; 

//=======================================================================================


//=======================================================================================
// Structures 

// User commands 
typedef struct gs_cmds_s 
{
    char gs_cmd[GS_MAX_USER_INPUT]; 
    void (*gs_cmd_func_ptr)(uint8_t); 
    uint8_t gs_cmd_mask; 
}
gs_cmds_t; 


// Data record for the system 
typedef struct gs_data_s 
{
    // System information 
    gs_states_t state;                            // State machine state 
    USART_TypeDef *uart;                          // UART port used for user interface 

    // Timing information 
    TIM_TypeDef *timer_nonblocking;                // Timer used for non-blocking delays 
    tim_compare_t delay_timer;                     // Delay timing info 

    // User commands and payload data 
    uint8_t user_buff[GS_MAX_USER_INPUT];          // Circular buffer (CB) that stores user inputs 
    uint8_t buff_index;                            // CB index used for parsing commands 
    uint8_t cmd_buff[GS_MAX_USER_INPUT];           // Stores a user command parsed from the CB 
    uint8_t cmd_id[GS_MAX_USER_INPUT];             // Stores the ID of the user command 
    uint8_t cmd_value;                             // Stores the value of the user command 
    uint8_t hb_msg[NRF24L01_MAX_PAYLOAD_LEN];      // Heartbeat message 
    uint8_t write_buff[NRF24L01_MAX_PAYLOAD_LEN];  // Data sent to PRX from PTX device 

    // System data 
    uint16_t adc_buff[GS_ADC_BUFF_SIZE];           // ADC buffer - thruster potentiometers 
    uint8_t cmd_send_index;                        // Command send counter 

    // Control flags 
    uint8_t led_state   : 1;                       // LED state (on/off) 
    uint8_t state_entry : 1;                       // State entry flag 
    uint8_t hb          : 1;                       // Heartbeat state flag 
    uint8_t mc          : 1;                       // Manual control mode state flag 
    uint8_t cmd         : 1;                       // Command send state flag 
}
gs_data_t; 

//=======================================================================================


//=======================================================================================
// Function pointers 

/**
 * @brief State machine function pointer 
 */
typedef void (*gs_state_func_ptr)(void); 

//=======================================================================================


//=======================================================================================
// Function prototypes 

/**
 * @brief Heartbeat state 
 * 
 * @details Periodically sends a ping to the PRX device on the vehicle so the vehicle 
 *          can see if it still has radio connection to the ground station. 
 *          
 *          Default system state. Enters/exits from/to the manual control and command 
 *          send states. 
 */
void gs_hb_state(void); 


/**
 * @brief Manual control state 
 * 
 * @details Sends formatted ADC potentiometer readings periodically for throttle control 
 *          when the vehicle is in manual control mode. 
 *          
 *          Enters/exits from/to the heartbeat state. 
 */
void gs_mc_state(void); 


/**
 * @brief Command send state 
 * 
 * @details Periodically sends the chosen command to the PRX device. This is done a set 
 *          number of times then defaults back to the heartbeat state. This is done to 
 *          make sure the message is received by the vehicle. 
 *          
 *          Enters/exits from/to the heartbeat state. 
 */
void gs_cmd_send_state(void); 


/**
 * @brief Heartbeat command 
 * 
 * @details Called when the user enters a heatbeat command. Puts the ground station into 
 *          the heatbeat state. 
 * 
 * @param hb_cmd_value : not used 
 */
void gs_hb_cmd(uint8_t hb_cmd_value); 


/**
 * @brief Manual control mode command 
 * 
 * @details Called when the user enters a manual control command. Puts the ground station 
 *          into the manual control state. 
 * 
 * @param mc_cmd_value : not used 
 */
void gs_mc_mode_cmd(uint8_t mc_cmd_value); 


/**
 * @brief Command send 
 * 
 * @details Called when the user enters a command to be sent to the boat (see 
 *          'cmd_table'). puts the ground station into the send command state. 
 * 
 * @param cmd_value : not used 
 */
void gs_cmd_send(uint8_t cmd_value); 


/**
 * @brief RF power output command 
 * 
 * @details Called when the user enters a power output command. Sets the power output 
 *          in the ground station RF module. The status of the operation gets displayed 
 *          in the serial terminal. 
 * 
 * @param rf_pwr : power output setting to set 
 */
void gs_pwr_cmd(uint8_t rf_pwr); 


/**
 * @brief Provides a prompt for the user at the serial terminal 
 */
void gs_user_prompt(void); 


/**
 * @brief Parse the user command into an ID and value 
 * 
 * @details Parses the user input at the serial terminal into an ID and command which 
 *          is then used to match against the list of available commands (see 
 *          'cmd_table'). 
 * 
 * @param command_buffer : buffer that contains the user input 
 * @return uint8_t : status of the parsing - true if a valid command is entered 
 */
uint8_t gs_parse_cmd(uint8_t *command_buffer); 


/**
 * @brief ADC to ESC command mapping 
 * 
 * @details Maps the ADC input to a throttle command which then gets sent to the vehicle. 
 *          This is used during manual control mode only. 
 * 
 * @param adc_val : ADC value read 
 * @return int16_t : throttle command calculated 
 */
int16_t gs_adc_mapping(uint16_t adc_val); 

//=======================================================================================


//=======================================================================================
// Global variables 

// Data record instance 
static gs_data_t gs_data; 


// Function pointer state table 
static gs_state_func_ptr state_table[GS_NUM_STATES] = 
{
    &gs_hb_state, 
    &gs_mc_state, 
    &gs_cmd_send_state 
}; 


// User commands 
static gs_cmds_t cmd_table[GS_NUM_CMDS] = 
{
    {"hb",     &gs_hb_cmd,      (SET_BIT << GS_MC_STATE)}, 
    {"mc",     &gs_mc_mode_cmd, (SET_BIT << GS_HB_STATE)}, 
    {"idle",   &gs_cmd_send,    (SET_BIT << GS_HB_STATE)}, 
    {"manual", &gs_cmd_send,    (SET_BIT << GS_HB_STATE)}, 
    {"auto",   &gs_cmd_send,    (SET_BIT << GS_HB_STATE)}, 
    {"index",  &gs_cmd_send,    (SET_BIT << GS_HB_STATE)}, 
    {"rfpwr",  &gs_pwr_cmd,     (SET_BIT << GS_HB_STATE) | (SET_BIT << GS_MC_STATE)} 
}; 

//=======================================================================================


//=======================================================================================
// Main functions 

// Ground station application initializzation 
void gs_app_init(
    TIM_TypeDef *timer_nonblocking, 
    DMA_Stream_TypeDef *adc_dma_stream, 
    ADC_TypeDef *adc, 
    DMA_Stream_TypeDef *uart_dma_stream, 
    USART_TypeDef *uart)
{
    // Ground station application code initialization 

    //==================================================
    // System configuration 

    // Configure the DMA stream for the ADC 
    dma_stream_config(
        adc_dma_stream, 
        (uint32_t)(&adc->DR), 
        (uint32_t)gs_data.adc_buff, 
        (uint16_t)GS_ADC_BUFF_SIZE); 

    // Configure the DMA stream for the UART 
    dma_stream_config(
        uart_dma_stream, 
        (uint32_t)(&uart->DR), 
        (uint32_t)gs_data.user_buff, 
        (uint16_t)GS_MAX_USER_INPUT); 
    
    //==================================================

    //==================================================
    // Data record initialization 

    // System information 
    gs_data.state = GS_HB_STATE; 
    gs_data.uart = uart; 

    // Timing information 
    gs_data.timer_nonblocking = timer_nonblocking; 
    gs_data.delay_timer.clk_freq = tim_get_pclk_freq(timer_nonblocking); 
    gs_data.delay_timer.time_cnt_total = CLEAR; 
    gs_data.delay_timer.time_cnt = CLEAR; 
    gs_data.delay_timer.time_start = SET_BIT; 

    // User commands and payload data 
    memset((void *)gs_data.user_buff, CLEAR, sizeof(gs_data.user_buff)); 
    gs_data.buff_index = CLEAR; 
    memset((void *)gs_data.cmd_buff, CLEAR, sizeof(gs_data.cmd_buff)); 
    memset((void *)gs_data.cmd_id, CLEAR, sizeof(gs_data.cmd_id)); 
    gs_data.cmd_value = CLEAR; 
    memset((void *)gs_data.hb_msg, CLEAR, sizeof(gs_data.hb_msg)); 
    strcpy((char *)gs_data.hb_msg, "ping"); 
    memset((void *)gs_data.write_buff, CLEAR, sizeof(gs_data.write_buff)); 

    // System data 
    memset((void *)gs_data.adc_buff, CLEAR, sizeof(gs_data.adc_buff)); 
    gs_data.cmd_send_index = GS_CMD_SEND_COUNT; 

    // Control flags 
    gs_data.led_state = CLEAR_BIT; 
    gs_data.state_entry = CLEAR_BIT; 
    gs_data.hb = SET_BIT; 
    gs_data.mc = CLEAR_BIT; 
    gs_data.cmd = CLEAR_BIT; 

    //==================================================

    // Provide an initial user prompt 
    gs_user_prompt(); 
}


// Ground station application  
void gs_app(void)
{
    // Ground station application code 

    // Local variables 
    gs_states_t next_state = gs_data.state; 

    //==================================================
    // User input command check 

    // Check for user serial terminal input 
    if (handler_flags.usart2_flag)
    {
        // Reset the USART2 interrupt flag 
        handler_flags.usart2_flag = CLEAR; 

        // Copy the new contents in the circular buffer to the user input buffer 
        cb_parse(
            gs_data.user_buff, 
            gs_data.cmd_buff, 
            &gs_data.buff_index, 
            GS_MAX_USER_INPUT); 

        // Validate the input - parse into an ID and value if valid 
        if (gs_parse_cmd(gs_data.cmd_buff))
        {
            // Valid input - compare the ID to each of the available pre-defined commands 
            for (uint8_t i = CLEAR; i < GS_NUM_CMDS; i++) 
            {
                // Check that the command is available for the "state" before comparing it 
                // against the ID. 
                if (cmd_table[i].gs_cmd_mask & (SET_BIT << gs_data.state))
                {
                    // Command available. Compare with the ID. 
                    if (str_compare(cmd_table[i].gs_cmd, (char *)gs_data.cmd_id, BYTE_0)) 
                    {
                        // ID matched to a command. Execute the command. 
                        (cmd_table[i].gs_cmd_func_ptr)(gs_data.cmd_value); 
                        break; 
                    }
                }
            }
        }

        gs_user_prompt(); 
    }

    //==================================================

    //==================================================
    // System state machine 

    switch (next_state)
    {
        case GS_HB_STATE: 
            if (gs_data.cmd)
            {
                next_state = GS_CMD_STATE; 
            }
            else if (gs_data.mc)
            {
                next_state = GS_MC_STATE; 
            }
            break; 
        
        case GS_MC_STATE: 
            if (gs_data.hb)
            {
                next_state = GS_HB_STATE; 
            }
            break; 

        case GS_CMD_STATE: 
            if (gs_data.hb)
            {
                next_state = GS_HB_STATE; 
            }
            break; 
        
        default: 
            next_state = GS_HB_STATE; 
            break; 
    }

    // Execute the state 
    state_table[next_state](); 

    // Update the state 
    gs_data.state = next_state; 
    
    //==================================================
}

//=======================================================================================


//=======================================================================================
// State functions 

// Heartbeat state 
void gs_hb_state(void)
{
    static uint8_t led_state = GPIO_LOW; 

    // Send a heartbeat message to the boat periodically 

    // Periodically send a ping to the PRX device 
    if (tim_compare(gs_data.timer_nonblocking, 
                    gs_data.delay_timer.clk_freq, 
                    GS_HB_PERIOD, 
                    &gs_data.delay_timer.time_cnt_total, 
                    &gs_data.delay_timer.time_cnt, 
                    &gs_data.delay_timer.time_start))
    {
        // Try sending out a payload and toggle the led if it was sent 
        if (nrf24l01_send_payload(gs_data.hb_msg))
        {
            led_state = GPIO_HIGH - led_state; 
            gpio_write(GPIOA, GPIOX_PIN_5, (gpio_pin_state_t)led_state); 
        } 
    }
}


// Heartbeat state 
void gs_mc_state(void)
{
    static uint8_t led_state = GPIO_LOW; 
    static uint8_t thruster = CLEAR; 
    char side = CLEAR; 
    char sign = GS_MC_FWD_THRUST; 
    int16_t throttle = CLEAR; 

    // Send formatted ADC potentiometer readings periodically for throttle control 

    // Periodically send the throttle command which is calculated from the ADC input - alternate 
    // between left and right side throttle commands for each send. 
    if (tim_compare(gs_data.timer_nonblocking, 
                    gs_data.delay_timer.clk_freq, 
                    GS_MC_PERIOD, 
                    &gs_data.delay_timer.time_cnt_total, 
                    &gs_data.delay_timer.time_cnt, 
                    &gs_data.delay_timer.time_start))
    {
        // time_start flag does not need to be set again because this timer runs 
        // continuously. 

        // Choose between right and left thruster 
        side = (thruster) ? GS_MC_LEFT_MOTOR : GS_MC_RIGHT_MOTOR; 

        // Read the ADC input and format the value for writing to the payload 
        throttle = gs_adc_mapping(gs_data.adc_buff[thruster]); 

        if (throttle == GS_NO_THRUST)
        {
            sign = GS_MC_NEUTRAL; 
        }
        else if (throttle < GS_NO_THRUST)
        {
            // If the throttle is negative then change the value to positive and set the sign 
            // in the payload as negative. This helps on the receiving end. 
            throttle = ~throttle + 1; 
            sign = GS_MC_REV_THRUST; 
        }

        // Format the payload with the thruster specifier and the throttle then send the 
        // payload. 
        snprintf(
            (char *)gs_data.write_buff, 
            NRF24L01_MAX_PAYLOAD_LEN, 
            "%c%c%d", 
            side, sign, throttle); 

        if (nrf24l01_send_payload(gs_data.write_buff))
        {
            led_state = GPIO_HIGH - led_state; 
            gpio_write(GPIOA, GPIOX_PIN_5, (gpio_pin_state_t)led_state); 
        } 

        // Toggle the thruster flag 
        thruster = SET_BIT - thruster; 
    }
}


// Command send state 
void gs_cmd_send_state(void)
{
    // Send a command X number of times 

    // Periodically send the command to the PRX device 
    if (tim_compare(gs_data.timer_nonblocking, 
                    gs_data.delay_timer.clk_freq, 
                    GS_CMD_PERIOD, 
                    &gs_data.delay_timer.time_cnt_total, 
                    &gs_data.delay_timer.time_cnt, 
                    &gs_data.delay_timer.time_start))
    {
        // Try sending out a payload and toggle the led if it was sent 
        if (nrf24l01_send_payload(gs_data.cmd_buff))
        {
            gs_data.led_state = GPIO_HIGH - gs_data.led_state; 
            gpio_write(GPIOA, GPIOX_PIN_5, (gpio_pin_state_t)gs_data.led_state); 
        } 

        gs_data.cmd_send_index--; 
    }

    // State exit 
    if (!gs_data.cmd_send_index)
    {
        gs_data.state_entry = SET_BIT; 
        gs_data.delay_timer.time_start = SET_BIT; 
        gs_data.cmd_send_index = GS_CMD_SEND_COUNT; 
        gs_data.hb = SET_BIT; 
        gs_data.cmd = CLEAR_BIT; 

        // Heartbeat state message 
        uart_sendstring(gs_data.uart, "\r\n\nHB state\r\n"); 
        gs_user_prompt(); 
    }
}

//=======================================================================================


//=======================================================================================
// Command functions 

// Heartbeat command 
void gs_hb_cmd(uint8_t hb_cmd_value)
{
    gs_data.hb = SET_BIT; 
    gs_data.mc = CLEAR_BIT; 
    gs_data.delay_timer.time_start = SET_BIT; 
    uart_sendstring(gs_data.uart, "\r\nHB state\r\n"); 
}


// Manual control mode command 
void gs_mc_mode_cmd(uint8_t mc_cmd_value)
{
    // gs_hb_state_exit(); 
    gs_data.mc = SET_BIT; 
    gs_data.hb = CLEAR_BIT; 
    gs_data.delay_timer.time_start = SET_BIT; 
    uart_sendstring(gs_data.uart, "\r\nMC state\r\n"); 
}


// Command send 
void gs_cmd_send(uint8_t cmd_value)
{
    // gs_hb_state_exit(); 
    gs_data.cmd = SET_BIT; 
    gs_data.hb = CLEAR_BIT; 
    gs_data.delay_timer.time_start = SET_BIT; 
    uart_sendstring(gs_data.uart, "\r\ncommand sending...\r\n"); 
}


// RF power output command 
void gs_pwr_cmd(uint8_t rf_pwr)
{
    // Check that input is within bounds 
    if (rf_pwr <= (uint8_t)NRF24L01_RF_PWR_0DBM)
    {
        nrf24l01_set_rf_pwr((nrf24l01_rf_pwr_t)rf_pwr); 

        if (nrf24l01_get_rf_pwr() == (nrf24l01_rf_pwr_t)rf_pwr)
        {
            uart_sendstring(gs_data.uart, "\r\nSuccess.\r\n"); 
        }
    }
}

//=======================================================================================


//=======================================================================================
// Other functions 

// User serial terminal prompt 
void gs_user_prompt(void)
{
    uart_sendstring(gs_data.uart, "\r\n>>> "); 
}

//=======================================================================================


//=======================================================================================
// Test functions 

// Parse the user command into an ID and value 
uint8_t gs_parse_cmd(uint8_t *command_buffer)
{
    // Local variables 
    uint8_t id_flag = SET_BIT; 
    uint8_t id_index = CLEAR; 
    uint8_t data = CLEAR; 
    uint8_t cmd_value[GS_MAX_USER_INPUT]; 
    uint8_t value_size = CLEAR; 

    // Initialize data 
    memset((void *)gs_data.cmd_id, CLEAR, sizeof(gs_data.cmd_id)); 
    gs_data.cmd_value = CLEAR; 
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
                gs_data.cmd_id[i] = data; 
            }
            else if (data >= ZERO_CHAR && data <= NINE_CHAR)
            {
                // Valid digit character byte seen 
                id_flag = CLEAR_BIT; 
                gs_data.cmd_id[i] = NULL_CHAR; 
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
        gs_data.cmd_value += (uint8_t)char_to_int(cmd_value[i], value_size-i-1); 
    }

    return TRUE; 
}


// ADC to ESC command mapping 
int16_t gs_adc_mapping(uint16_t adc_val)
{
    // Local variables 
    int16_t throttle_cmd = CLEAR;   // Assume 0% throttle and change if different 

    // Check if there is a forward or reverse throttle command 
    if (adc_val > GS_ADC_FWD_LIM)
    {
        // Forward 
        throttle_cmd = (int16_t)adc_val - GS_ADC_FWD_LIM; 
    }
    else if (adc_val < GS_ADC_REV_LIM)
    {
        // Reverse 
        throttle_cmd = (int16_t)adc_val - GS_ADC_REV_LIM; 
    }

    return throttle_cmd; 
}

//=======================================================================================
