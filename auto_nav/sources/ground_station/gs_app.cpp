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

#include "gs_app.h"

//=======================================================================================


//=======================================================================================
// Function prototypes 

/**
 * @brief Heartbeat state 
 * 
 * @details 
 */
void gs_hb_state(void); 


/**
 * @brief Manual control state 
 * 
 * @details 
 */
void gs_mc_state(void); 


/**
 * @brief Command send state 
 * 
 * @details 
 */
void gs_cmd_send_state(void); 


// /**
//  * @brief Idle command state 
//  * 
//  * @details 
//  */
// void gs_idle_cmd_state(void); 


// /**
//  * @brief Manual control command state 
//  * 
//  * @details 
//  */
// void gs_manual_cmd_state(void); 


// /**
//  * @brief Autonomous command state 
//  * 
//  * @details 
//  */
// void gs_auto_cmd_state(void); 


// /**
//  * @brief Waypoint index update command state 
//  * 
//  * @details 
//  */
// void gs_index_cmd_state(void); 


/**
 * @brief Heartbeat command 
 * 
 * @param hb_cmd_value 
 */
void gs_hb_cmd(
    uint8_t hb_cmd_value); 


/**
 * @brief Manual control mode command 
 * 
 * @param mc_cmd_value 
 */
void gs_mc_mode_cmd(
    uint8_t mc_cmd_value); 


/**
 * @brief Command send S
 * 
 * @param idle_cmd_value 
 */
void gs_cmd_send(
    uint8_t idle_cmd_value); 


// /**
//  * @brief Idle command 
//  * 
//  * @param idle_cmd_value 
//  */
// void gs_idle_cmd(
//     uint8_t idle_cmd_value); 


// /**
//  * @brief Manual control mode command 
//  * 
//  * @param manual_cmd_value 
//  */
// void gs_manual_cmd(
//     uint8_t manual_cmd_value); 


// /**
//  * @brief Autonomous mode command 
//  * 
//  * @param auto_cmd_value 
//  */
// void gs_auto_cmd(
//     uint8_t auto_cmd_value); 


// /**
//  * @brief Index update command 
//  * 
//  * @param index_cmd_value 
//  */
// void gs_index_cmd(
//     uint8_t index_cmd_value); 


/**
 * @brief Heartbeat state exit 
 * 
 * @details 
 */
void gs_state_exit(void); 


// /**
//  * @brief Heartbeat state exit 
//  * 
//  * @details 
//  */
// void gs_hb_state_exit(void); 


/**
 * @brief Parse the user command into an ID and value 
 * 
 * @details 
 * 
 * @param command_buffer 
 * @return uint8_t 
 */
uint8_t gs_parse_cmd(
    uint8_t *command_buffer); 


/**
 * @brief ADC to ESC command mapping 
 * 
 * @details 
 * 
 * @param adc_val 
 * @return int16_t 
 */
int16_t gs_adc_mapping(
    uint16_t adc_val); 

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
    // &gs_idle_cmd_state, 
    // &gs_manual_cmd_state, 
    // &gs_auto_cmd_state, 
    // &gs_index_cmd_state
}; 


// User commands 
static gs_cmds_t cmd_table[GS_NUM_CMDS] = 
{
    {"hb",     &gs_hb_cmd,      0xFF}, 
    {"mc",     &gs_mc_mode_cmd, 0x38}, 
    // {"idle",   &gs_idle_cmd,    0x3F}, 
    // {"manual", &gs_manual_cmd,  0x3F}, 
    // {"auto",   &gs_auto_cmd,    0x3F}, 
    // {"index",  &gs_index_cmd,   0x38} 
    {"idle",   &gs_cmd_send,    0x3F}, 
    {"manual", &gs_cmd_send,    0x3F}, 
    {"auto",   &gs_cmd_send,    0x3F}, 
    {"index",  &gs_cmd_send,    0x38} 
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

    // 
    gs_data.waypoint_index = CLEAR; 

    // 
    gs_data.cmd_send_index = GS_SEND_COUNT; 

    // Control flags 
    gs_data.led_state = CLEAR_BIT; 
    gs_data.state_entry = CLEAR_BIT; 
    gs_data.hb = SET_BIT; 
    gs_data.mc = CLEAR_BIT; 
    // gs_data.idle = CLEAR_BIT; 
    // gs_data.manual = CLEAR_BIT; 
    // gs_data.autonomous = CLEAR_BIT; 
    // gs_data.index = CLEAR_BIT; 

    //==================================================

    // Provide an initial user prompt 
    uart_sendstring(USART2, "\r\n>>> "); 
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
                    if (str_compare(
                            cmd_table[i].gs_cmd, 
                            (char *)gs_data.cmd_id, 
                            BYTE_0)) 
                    {
                        // ID matched to a command. Execute the command. 
                        (cmd_table[i].gs_cmd_func_ptr)(gs_data.cmd_value); 
                        break; 
                    }
                }
            }
        }

        uart_sendstring(USART2, "\r\n>>> "); 
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
            // if (gs_data.idle)
            // {
            //     next_state = GS_IDLE_CMD_STATE; 
            // }
            // else if (gs_data.manual)
            // {
            //     next_state = GS_MANUAL_CMD_STATE; 
            // }
            // else if (gs_data.autonomous)
            // {
            //     next_state = GS_AUTO_CMD_STATE; 
            // }
            // else if (gs_data.index)
            // {
            //     next_state = GS_INDEX_CMD_STATE; 
            // }
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

        // case GS_IDLE_CMD_STATE: 
        //     if (gs_data.hb)
        //     {
        //         next_state = GS_HB_STATE; 
        //     }
        //     break; 

        // case GS_MANUAL_CMD_STATE: 
        //     if (gs_data.hb)
        //     {
        //         next_state = GS_HB_STATE; 
        //     }
        //     break; 

        // case GS_AUTO_CMD_STATE: 
        //     if (gs_data.hb)
        //     {
        //         next_state = GS_HB_STATE; 
        //     }
        //     break; 

        // case GS_INDEX_CMD_STATE: 
        //     if (gs_data.hb)
        //     {
        //         next_state = GS_HB_STATE; 
        //     }
        //     break; 
        
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
    // Local variables 
    static uint8_t led_state = GPIO_LOW; 

    //==================================================
    // State entry 

    if (gs_data.state_entry)
    {
        gs_data.state_entry = CLEAR_BIT; 
    }

    //==================================================

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

    //==================================================
    // State exit 

    // State exit contolled by user inputs and command functions so there isn't one here. 

    //==================================================
}


// Heartbeat state 
void gs_mc_state(void)
{
    // Local variables 
    static uint8_t led_state = GPIO_LOW; 
    static uint8_t thruster = CLEAR; 
    char side = CLEAR; 
    char sign = GS_MC_FWD_THRUST; 
    int16_t throttle = CLEAR; 

    //==================================================
    // State entry 

    if (gs_data.state_entry)
    {
        gs_data.state_entry = CLEAR_BIT; 
    }

    //==================================================

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

    //==================================================
    // State exit 

    // State exit contolled by user inputs and command functions so there isn't one here. 

    //==================================================
}


// Command send state 
void gs_cmd_send_state(void)
{
    //==================================================
    // State entry 

    if (gs_data.state_entry)
    {
        gs_data.state_entry = CLEAR_BIT; 
    }

    //==================================================

    // Send the idle state command X number of times 

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

    //==================================================
    // State exit 

    if (!gs_data.cmd_send_index)
    {
        gs_data.state_entry = SET_BIT; 
        gs_data.delay_timer.time_start = SET_BIT; 
        gs_data.cmd_send_index = GS_SEND_COUNT; 
    }

    //==================================================
}


// // Heartbeat state 
// void gs_idle_cmd_state(void)
// {
//     //==================================================
//     // State entry 

//     if (gs_data.state_entry)
//     {
//         gs_data.state_entry = CLEAR_BIT; 
//     }

//     //==================================================

//     // Send the idle state command X number of times 

//     // Periodically send the command to the PRX device 
//     if (tim_compare(gs_data.timer_nonblocking, 
//                     gs_data.delay_timer.clk_freq, 
//                     GS_CMD_PERIOD, 
//                     &gs_data.delay_timer.time_cnt_total, 
//                     &gs_data.delay_timer.time_cnt, 
//                     &gs_data.delay_timer.time_start))
//     {
//         // Try sending out a payload and toggle the led if it was sent 
//         if (nrf24l01_send_payload(gs_data.cmd_buff))
//         {
//             gs_data.led_state = GPIO_HIGH - gs_data.led_state; 
//             gpio_write(GPIOA, GPIOX_PIN_5, (gpio_pin_state_t)gs_data.led_state); 
//         } 

//         gs_data.cmd_send_index--; 
//     }

//     //==================================================
//     // State exit 

//     if (!gs_data.cmd_send_index)
//     {
//         gs_data.state_entry = SET_BIT; 
//         gs_data.delay_timer.time_start = SET_BIT; 
//         gs_data.cmd_send_index = GS_SEND_COUNT; 
//     }

//     //==================================================
// }


// // Heartbeat state 
// void gs_manual_cmd_state(void)
// {
//     //==================================================
//     // State entry 

//     if (gs_data.state_entry)
//     {
//         gs_data.state_entry = CLEAR_BIT; 
//     }

//     //==================================================

//     // Send the manual control state command X number of times 

//     // Periodically send the command to the PRX device 
//     if (tim_compare(gs_data.timer_nonblocking, 
//                     gs_data.delay_timer.clk_freq, 
//                     GS_CMD_PERIOD, 
//                     &gs_data.delay_timer.time_cnt_total, 
//                     &gs_data.delay_timer.time_cnt, 
//                     &gs_data.delay_timer.time_start))
//     {
//         // Try sending out a payload and toggle the led if it was sent 
//         if (nrf24l01_send_payload(gs_data.cmd_id))
//         {
//             gs_data.led_state = GPIO_HIGH - gs_data.led_state; 
//             gpio_write(GPIOA, GPIOX_PIN_5, (gpio_pin_state_t)gs_data.led_state); 
//         } 

//         gs_data.cmd_send_index--; 
//     }

//     //==================================================
//     // State exit 

//     if (!gs_data.cmd_send_index)
//     {
//         gs_data.state_entry = SET_BIT; 
//         gs_data.delay_timer.time_start = SET_BIT; 
//         gs_data.cmd_send_index = GS_SEND_COUNT; 
//     }

//     //==================================================
// }


// // Heartbeat state 
// void gs_auto_cmd_state(void)
// {
//     //==================================================
//     // State entry 

//     if (gs_data.state_entry)
//     {
//         gs_data.state_entry = CLEAR_BIT; 
//     }

//     //==================================================

//     // Send the auto state command X number of times 

//     // Periodically send the command to the PRX device 
//     if (tim_compare(gs_data.timer_nonblocking, 
//                     gs_data.delay_timer.clk_freq, 
//                     GS_CMD_PERIOD, 
//                     &gs_data.delay_timer.time_cnt_total, 
//                     &gs_data.delay_timer.time_cnt, 
//                     &gs_data.delay_timer.time_start))
//     {
//         // Try sending out a payload and toggle the led if it was sent 
//         if (nrf24l01_send_payload(gs_data.cmd_id))
//         {
//             gs_data.led_state = GPIO_HIGH - gs_data.led_state; 
//             gpio_write(GPIOA, GPIOX_PIN_5, (gpio_pin_state_t)gs_data.led_state); 
//         } 

//         gs_data.cmd_send_index--; 
//     }

//     //==================================================
//     // State exit 

//     if (!gs_data.cmd_send_index)
//     {
//         gs_data.state_entry = SET_BIT; 
//         gs_data.delay_timer.time_start = SET_BIT; 
//         gs_data.cmd_send_index = GS_SEND_COUNT; 
//     }

//     //==================================================
// }


// // Heartbeat state 
// void gs_index_cmd_state(void)
// {
//     //==================================================
//     // State entry 

//     if (gs_data.state_entry)
//     {
//         gs_data.state_entry = CLEAR_BIT; 
//     }

//     //==================================================

//     // Send an updated waypoint index command X number of times 

//     // Periodically send the command to the PRX device 
//     if (tim_compare(gs_data.timer_nonblocking, 
//                     gs_data.delay_timer.clk_freq, 
//                     GS_CMD_PERIOD, 
//                     &gs_data.delay_timer.time_cnt_total, 
//                     &gs_data.delay_timer.time_cnt, 
//                     &gs_data.delay_timer.time_start))
//     {
//         // Try sending out a payload and toggle the led if it was sent 
//         if (nrf24l01_send_payload(gs_data.cmd_id))
//         {
//             gs_data.led_state = GPIO_HIGH - gs_data.led_state; 
//             gpio_write(GPIOA, GPIOX_PIN_5, (gpio_pin_state_t)gs_data.led_state); 
//         } 

//         gs_data.cmd_send_index--; 
//     }

//     //==================================================
//     // State exit 

//     if (!gs_data.cmd_send_index)
//     {
//         gs_data.state_entry = SET_BIT; 
//         gs_data.delay_timer.time_start = SET_BIT; 
//         gs_data.cmd_send_index = GS_SEND_COUNT; 
//     }

//     //==================================================
// }

//=======================================================================================


//=======================================================================================
// Command functions 

// Heartbeat command 
void gs_hb_cmd(
    uint8_t hb_cmd_value)
{
    gs_data.hb = SET_BIT; 
    gs_data.mc = CLEAR_BIT; 
    gs_state_exit(); 
}


// Manual control mode command 
void gs_mc_mode_cmd(
    uint8_t mc_cmd_value)
{
    // gs_hb_state_exit(); 
    gs_data.mc = SET_BIT; 
    gs_data.hb = CLEAR_BIT; 
    gs_state_exit(); 
}


// Command send 
void gs_cmd_send(
    uint8_t idle_cmd_value)
{
    // gs_hb_state_exit(); 
    gs_data.cmd = SET_BIT; 
    gs_data.hb = CLEAR_BIT; 
    gs_state_exit(); 
}


// // Idle command 
// void gs_idle_cmd(
//     uint8_t idle_cmd_value)
// {
//     gs_data.idle = SET_BIT; 
//     gs_hb_state_exit(); 
// }


// // Manual control mode command 
// void gs_manual_cmd(
//     uint8_t manual_cmd_value)
// {
//     gs_data.manual = SET_BIT; 
//     gs_hb_state_exit(); 
// }


// // Autonomous mode command 
// void gs_auto_cmd(
//     uint8_t auto_cmd_value)
// {
//     gs_data.autonomous = SET_BIT; 
//     gs_hb_state_exit(); 
// }


// // Index update command 
// void gs_index_cmd(
//     uint8_t index_cmd_value)
// {
//     gs_data.waypoint_index = index_cmd_value; 
//     gs_data.index = SET_BIT; 
//     gs_hb_state_exit(); 
// }

//=======================================================================================


//=======================================================================================
// Other functions 

// Heartbeat state exit 
void gs_state_exit(void)
{
    gs_data.delay_timer.time_start = SET_BIT; 
    gs_data.state_entry = SET_BIT; 
}


// // Heartbeat state exit 
// void gs_hb_state_exit(void)
// {
//     gs_data.hb = CLEAR_BIT; 
//     gs_data.delay_timer.time_start = SET_BIT; 
//     gs_data.state_entry = SET_BIT; 
// }

//=======================================================================================


//=======================================================================================
// Test functions 

// Parse the user command into an ID and value 
uint8_t gs_parse_cmd(
    uint8_t *command_buffer)
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
int16_t gs_adc_mapping(
    uint16_t adc_val)
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
