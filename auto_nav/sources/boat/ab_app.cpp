/**
 * @file project_init.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Autonomous boat initialization code 
 * 
 * @version 0.1
 * @date 2023-07-20
 * 
 * @copyright Copyright (c) 2023
 * 
 */

//=======================================================================================
// Includes 

#include "ab_app.h"

//=======================================================================================


//=======================================================================================
// Function prototypes 

/**
 * @brief 
 * 
 * @details 
 */
void ab_init_state(void); 


/**
 * @brief 
 * 
 * @details 
 */
void ab_not_ready_state(void); 


/**
 * @brief 
 * 
 * @details 
 */
void ab_ready_state(void); 


/**
 * @brief 
 * 
 * @details 
 */
void ab_manual_state(void); 


/**
 * @brief 
 * 
 * @details 
 */
void ab_auto_state(void); 


/**
 * @brief 
 * 
 * @details 
 */
void ab_low_pwr_state(void); 


/**
 * @brief 
 * 
 * @details 
 */
void ab_fault_state(void); 


/**
 * @brief 
 * 
 * @details 
 */
void ab_reset_state(void); 


/**
 * @brief 
 * 
 * @details 
 * 
 * @param idle_cmd_value : 
 */
void ab_idle_cmd(
    uint8_t idle_cmd_value); 


/**
 * @brief 
 * 
 * @details 
 * 
 * @param manual_cmd_value : 
 */
void ab_manual_cmd(
    uint8_t manual_cmd_value); 


/**
 * @brief 
 * 
 * @details 
 * 
 * @param auto_cmd_value : 
 */
void ab_auto_cmd(
    uint8_t auto_cmd_value); 


/**
 * @brief 
 * 
 * @details 
 * 
 * @param index_cmd_value : 
 */
void ab_index_cmd(
    uint8_t index_cmd_value); 


/**
 * @brief 
 * 
 * @details 
 * 
 * @param throttle_cmd_value : 
 */
void ab_throttle_cmd(
    uint8_t throttle_cmd_value); 


/**
 * @brief 
 * 
 * @details 
 * 
 * @param hb_cmd_value : 
 */
void ab_hb_cmd(
    uint8_t hb_cmd_value); 


/**
 * @brief GPS heading calculation 
 * 
 * @details 
 */
void ab_gps_rad(void); 


/**
 * @brief GPS heading calculation 
 * 
 * @details 
 */
void ab_gps_heading(void); 


/**
 * @brief Get the true North heading 
 * 
 * @details 
 */
void ab_heading(void); 


/**
 * @brief Heading error 
 * 
 * @details 
 */
void ab_heading_error(void); 


/**
 * @brief Parse the user command into an ID and value 
 * 
 * @details 
 * 
 * @param command_buffer 
 * @return uint8_t 
 */
uint8_t ab_parse_cmd(
    uint8_t *command_buffer); 

//=======================================================================================


//=======================================================================================
// Global variables 

// Data record instance 
static ab_data_t ab_data; 

// Function pointer table 
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

// Command table 
static ab_cmds_t cmd_table[AB_NUM_CMDS] = 
{
    {"idle",   &ab_idle_cmd,     (SET_BIT << AB_MANUAL_STATE) | (SET_BIT << AB_AUTO_STATE)}, 
    {"manual", &ab_manual_cmd,   (SET_BIT << AB_READY_STATE)}, 
    {"auto",   &ab_auto_cmd,     (SET_BIT << AB_READY_STATE)}, 
    {"index",  &ab_index_cmd,    (SET_BIT << AB_READY_STATE)  | (SET_BIT << AB_AUTO_STATE)}, 
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
                                 (SET_BIT << AB_RESET_STATE)}   // Available in all states 
}; 

// GPS coordinate table 

const static ab_waypoints_t waypoint_table[AB_NUM_COORDINATES] = 
{
    {50.962742, -114.064726},    // Index 0 
    {50.962174, -114.066598},    // Index 1 
    {50.961458, -114.064688},    // Index 2 
    {50.962255, -114.063186},    // Index 3 
    // {0.000000, 0.000000},    // Index 4 
    // {0.000000, 0.000000},    // Index 5 
    // {0.000000, 0.000000},    // Index 6 
    // {0.000000, 0.000000},    // Index 7 
    // {0.000000, 0.000000},    // Index 8 
    // {0.000000, 0.000000}     // Index 9 
}; 

//=======================================================================================


//=======================================================================================
// Main functions 

// Autonomous boat application initialization 
void ab_app_init(
    TIM_TypeDef *timer_nonblocking, 
    DMA_Stream_TypeDef *adc_dma_stream, 
    ADC_TypeDef *adc, 
    nrf24l01_data_pipe_t pipe_num)
{
    // Autonomous boat application code initialization 

    //==================================================
    // System configuration 

    // Configure the DMA stream 
    dma_stream_config(
        adc_dma_stream, 
        (uint32_t)(&adc->DR), 
        (uint32_t)ab_data.adc_buff, 
        (uint16_t)AB_ADC_BUFF_SIZE); 
    
    //==================================================

    //==================================================
    // Data record initialization 
    
    // System information 
    ab_data.state = AB_INIT_STATE; 
    ab_data.adc = adc; 
    ab_data.pipe = pipe_num; 
    ab_data.fault_code = CLEAR; 

    // Timing information 
    ab_data.timer_nonblocking = timer_nonblocking; 
    ab_data.delay_timer.clk_freq = tim_get_pclk_freq(timer_nonblocking); 
    ab_data.delay_timer.time_cnt_total = CLEAR; 
    ab_data.delay_timer.time_cnt = CLEAR; 
    ab_data.delay_timer.time_start = SET_BIT; 
    ab_data.hb_timer.clk_freq = tim_get_pclk_freq(timer_nonblocking); 
    ab_data.hb_timer.time_cnt_total = CLEAR; 
    ab_data.hb_timer.time_cnt = CLEAR; 
    ab_data.hb_timer.time_start = SET_BIT; 
    ab_data.hb_timeout = CLEAR; 

    // System data 
    memset((void *)ab_data.adc_buff, CLEAR, sizeof(ab_data.adc_buff)); 
    memset((void *)ab_data.led_data, CLEAR, sizeof(ab_data.led_data)); 
    ws2812_send(DEVICE_ONE, ab_data.led_data); 

    // Payload data 
    memset((void *)ab_data.read_buff, CLEAR, sizeof(ab_data.read_buff)); 
    memset((void *)ab_data.cmd_id, CLEAR, sizeof(ab_data.cmd_id)); 
    ab_data.cmd_value = CLEAR; 
    memset((void *)ab_data.hb_msg, CLEAR, sizeof(ab_data.hb_msg)); 

    // Navigation data 
    ab_data.waypoint_index = CLEAR; 
    ab_data.waypoint.lat = waypoint_table[0].lat; 
    ab_data.waypoint.lon = waypoint_table[0].lon; 
    ab_data.location.lat = m8q_get_lat(); 
    ab_data.location.lon = m8q_get_long(); 
    ab_data.waypoint_rad = CLEAR; 
    ab_data.heading_desired = CLEAR; 
    ab_data.heading_actual = CLEAR; 
    ab_data.heading_error = CLEAR; 

    // Thrusters 
    ab_data.right_thruster = AB_NO_THRUST; 
    ab_data.left_thruster = AB_NO_THRUST; 

    // Control flags 
    ab_data.connect = CLEAR_BIT; 
    ab_data.mc_data = CLEAR_BIT; 
    ab_data.state_entry = SET_BIT; 
    ab_data.init = SET_BIT; 
    ab_data.ready = CLEAR_BIT; 
    ab_data.idle = CLEAR_BIT; 
    ab_data.manual = CLEAR_BIT; 
    ab_data.autonomous = CLEAR_BIT; 
    ab_data.low_pwr = CLEAR_BIT; 
    ab_data.fault = CLEAR_BIT; 
    ab_data.reset = CLEAR_BIT; 
    
    //==================================================
}


// Autonomous boat application  
void ab_app(void)
{
    // Autonomous boat application code 

    // Local variables 
    ab_states_t next_state = ab_data.state; 

    //==================================================
    // Device status checks 

    if (m8q_get_fault_code())
    {
        ab_data.fault_code |= (SET_BIT << SHIFT_0); 
    }
    if (lsm303agr_get_status())
    {
        ab_data.fault_code |= (SET_BIT << SHIFT_1); 
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
    
    // // GPS position lock check 
    // If the system loses GPS position lock in manual mode then it continues on. 
    if (((m8q_get_navstat() & M8Q_NAVSTAT_D2) != M8Q_NAVSTAT_D2) && 
        (ab_data.state != AB_MANUAL_STATE))
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

    //==================================================
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

    //==================================================

    //==================================================
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
    // Local variables 

    //==================================================
    // State entry 

    if (ab_data.state_entry)
    {
        ab_data.state_entry = CLEAR_BIT; 

        uart_sendstring(USART2, "\r\ninit\r\n"); 

        // Set the thruster throttle to zero to initialize the ESCs and motors. 
        esc_readytosky_send(DEVICE_ONE, AB_NO_THRUST); 
        esc_readytosky_send(DEVICE_TWO, AB_NO_THRUST); 
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
    // Local variables 
    static uint8_t led_counter = CLEAR; 

    //==================================================
    // State entry 
    
    if (ab_data.state_entry)
    {
        ab_data.state_entry = CLEAR_BIT; 

        uart_sendstring(USART2, "\r\nnot_ready\r\n"); 
    }

    //==================================================

    // Wait for the system requirements to be met - ready flag will be set 

    //==================================================
    // External feedback 

    // TODO LEDs can't operate because they use the same timer as the ESCs but with a 
    //      different prescalar. 

    // Toggle an LED to indicate the state to the user 
    if (tim_compare(ab_data.timer_nonblocking, 
                    ab_data.delay_timer.clk_freq, 
                    AB_READY_PERIOD, 
                    &ab_data.delay_timer.time_cnt_total, 
                    &ab_data.delay_timer.time_cnt, 
                    &ab_data.delay_timer.time_start))
    {
        if (!led_counter)
        {
            // ab_data.led_data[WS2812_LED_3] = ab_led_clear; 
            // ab_data.led_data[WS2812_LED_4] = ab_led_clear; 
            // ws2812_send(DEVICE_ONE, ab_data.led_data); 
            led_counter++; 
        }
        else if (led_counter >= AB_READY_TIMEOUT)
        {
            // ab_data.led_data[WS2812_LED_3] = ab_led3_not_ready; 
            // ab_data.led_data[WS2812_LED_4] = ab_led4_not_ready; 
            // ws2812_send(DEVICE_ONE, ab_data.led_data); 
            led_counter = CLEAR; 
        }
        else 
        {
            led_counter++; 
        }
    }
    
    //==================================================

    //==================================================
    // State exit 

    if (ab_data.fault | ab_data.low_pwr | ab_data.ready)
    {
        ab_data.state_entry = SET_BIT; 
        ab_data.delay_timer.time_start = SET_BIT; 
        ab_data.idle = SET_BIT; 
        ab_data.manual = CLEAR_BIT; 
        ab_data.autonomous = CLEAR_BIT; 
        led_counter = CLEAR; 

        // Make sure LEDs are off 
        ab_data.led_data[WS2812_LED_3] = ab_led_clear; 
        ab_data.led_data[WS2812_LED_4] = ab_led_clear; 
        ws2812_send(DEVICE_ONE, ab_data.led_data); 
    }

    //==================================================
}


// Ready state 
void ab_ready_state(void)
{
    // Local variables 
    static uint8_t led_counter = CLEAR; 

    //==================================================
    // State entry 

    if (ab_data.state_entry)
    {
        ab_data.state_entry = CLEAR_BIT; 

        uart_sendstring(USART2, "\r\nready\r\n"); 
    }

    //==================================================

    // Wait for an active state to be chosen 

    //==================================================
    // External feedback 

    // TODO LEDs can't operate because they use the same timer as the ESCs but with a 
    //      different prescalar. 

    // Toggle an LED to indicate the state to the user 
    if (tim_compare(ab_data.timer_nonblocking, 
                    ab_data.delay_timer.clk_freq, 
                    AB_READY_PERIOD, 
                    &ab_data.delay_timer.time_cnt_total, 
                    &ab_data.delay_timer.time_cnt, 
                    &ab_data.delay_timer.time_start))
    {
        if (!led_counter)
        {
            // ab_data.led_data[WS2812_LED_3] = ab_led_clear; 
            // ab_data.led_data[WS2812_LED_4] = ab_led_clear; 
            // ws2812_send(DEVICE_ONE, ab_data.led_data); 
            led_counter++; 
        }
        else if (led_counter >= AB_READY_TIMEOUT)
        {
            // ab_data.led_data[WS2812_LED_3] = ab_led3_ready; 
            // ab_data.led_data[WS2812_LED_4] = ab_led4_ready; 
            // ws2812_send(DEVICE_ONE, ab_data.led_data); 
            led_counter = CLEAR; 
        }
        else 
        {
            led_counter++; 
        }
    }
    
    //==================================================

    //==================================================
    // State exit 

    // Manual and autonomous mode exit conditions come from external commands received so 
    // they're not included here. 

    if (ab_data.fault | ab_data.low_pwr | !ab_data.ready)
    {
        ab_data.state_entry = SET_BIT; 
        ab_data.delay_timer.time_start = SET_BIT; 
        ab_data.idle = CLEAR_BIT; 
        led_counter = CLEAR; 

        // Make sure LEDs are off 
        ab_data.led_data[WS2812_LED_3] = ab_led_clear; 
        ab_data.led_data[WS2812_LED_4] = ab_led_clear; 
        ws2812_send(DEVICE_ONE, ab_data.led_data); 
    }

    //==================================================
}


// Manual control mode state 
void ab_manual_state(void)
{
    // Local variables 
    int16_t cmd_value = CLEAR; 

    //==================================================
    // State entry 

    if (ab_data.state_entry)
    {
        ab_data.state_entry = CLEAR_BIT; 

        uart_sendstring(USART2, "\r\nmanual\r\n"); 
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
                        ab_data.right_thruster += (cmd_value - 
                                                        ab_data.right_thruster) >> SHIFT_3; 
                        esc_readytosky_send(DEVICE_ONE, ab_data.right_thruster); 
                        break; 
                    case AB_MC_REV_THRUST: 
                        ab_data.right_thruster += ((~cmd_value + 1) - 
                                                        ab_data.right_thruster) >> SHIFT_3; 
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
                        ab_data.left_thruster += (cmd_value - 
                                                        ab_data.left_thruster) >> SHIFT_3; 
                        esc_readytosky_send(DEVICE_TWO, ab_data.left_thruster); 
                        break; 
                    case AB_MC_REV_THRUST: 
                        ab_data.left_thruster += ((~cmd_value + 1) - 
                                                        ab_data.left_thruster) >> SHIFT_3; 
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
        ab_data.right_thruster = AB_NO_THRUST; 
        ab_data.left_thruster = AB_NO_THRUST; 
        esc_readytosky_send(DEVICE_ONE, AB_NO_THRUST); 
        esc_readytosky_send(DEVICE_TWO, AB_NO_THRUST); 
    }

    //==================================================
}


// Autonomous mode state 
void ab_auto_state(void)
{
    // Local variables 
    static uint8_t nav_period_flag = SET_BIT; 

    //==================================================
    // State entry 

    if (ab_data.state_entry)
    {
        ab_data.state_entry = CLEAR_BIT; 

        uart_sendstring(USART2, "\r\nauto\r\n"); 
    }

    //==================================================

    //==================================================
    // Navigation calculations 

    // Update the navigation calculations 
    if (tim_compare(ab_data.timer_nonblocking, 
                    ab_data.delay_timer.clk_freq, 
                    AB_NAV_UPDATE, 
                    &ab_data.delay_timer.time_cnt_total, 
                    &ab_data.delay_timer.time_cnt, 
                    &ab_data.delay_timer.time_start))
    {
        // Update the current location and desired heading every other period (can't be 
        // faster than once per second) 
        if (nav_period_flag)
        {
            // Get the updated location 
            ab_data.location.lat = m8q_get_lat(); 
            ab_data.location.lon = m8q_get_long(); 

            // Update GPS radius and desired heading. 
            ab_gps_rad(); 
            ab_gps_heading(); 

            // If the device is close enough to a waypoint then the next waypoint in the 
            // mission is selected. 
            if (ab_data.waypoint_rad < AB_WAYPOINT_RAD)
            {
                // Adjust waypoint index. If the end of the waypoint mission is reached 
                // then start over from the beginning. 
                if (++ab_data.waypoint_index >= AB_NUM_COORDINATES)
                {
                    ab_data.waypoint_index = CLEAR; 
                }

                // Update the target waypoint 
                ab_data.waypoint.lat = waypoint_table[ab_data.waypoint_index].lat; 
                ab_data.waypoint.lon = waypoint_table[ab_data.waypoint_index].lon; 
            }
        }

        nav_period_flag = SET_BIT - nav_period_flag; 

        // Update the current heading and calculate the thruster output every period 

        // Update the magnetometer data 
        lsm303agr_m_read(); 

        // Get the true North heading from the magnetometer 
        ab_heading(); 
        
        // Use the GPS heading and the magnetometer heading to get a heading error 
        ab_heading_error(); 

        // Calculate the thruster command 
        // throttle = (base throttle) + error*slope 
        ab_data.right_thruster = AB_AUTO_BASE_SPEED - ab_data.heading_error*ESC_MAX_THROTTLE / 
                                                      (AB_AUTO_MAX_ER_P + AB_AUTO_MAX_ER_N); 
        ab_data.left_thruster = AB_AUTO_BASE_SPEED +  ab_data.heading_error*ESC_MAX_THROTTLE / 
                                                      (AB_AUTO_MAX_ER_P + AB_AUTO_MAX_ER_N); 

        esc_readytosky_send(DEVICE_ONE, ab_data.right_thruster); 
        esc_readytosky_send(DEVICE_TWO, ab_data.left_thruster); 
    }

    //==================================================

    //==================================================
    // State exit 

    // the idle (ready) state exit condition comes from an external command received 
    // so it's not included here. 

    if (ab_data.fault | ab_data.low_pwr | !ab_data.ready)
    {
        ab_data.state_entry = SET_BIT; 
        ab_data.autonomous = CLEAR_BIT; 

        // Set the throttle to zero to stop the thrusters 
        ab_data.right_thruster = AB_NO_THRUST; 
        ab_data.left_thruster = AB_NO_THRUST; 
        esc_readytosky_send(DEVICE_ONE, AB_NO_THRUST); 
        esc_readytosky_send(DEVICE_TWO, AB_NO_THRUST); 
    }

    //==================================================
}


// Low power state 
void ab_low_pwr_state(void)
{
    // Local variables 

    //==================================================
    // State entry 

    if (ab_data.state_entry)
    {
        ab_data.state_entry = CLEAR_BIT; 

        uart_sendstring(USART2, "\r\nlow_pwr\r\n"); 

        // Put devices in low power mode 
        m8q_set_low_pwr_flag(); 
    }

    //==================================================

    // Wait for a system reset 
    // Currently the system would hang here until it's power cycled because 
    // there is no setter for the reset flag. This is ok because if the 
    // battery voltage is low there is nothing the system can and should 
    // do anyway. 

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
    // Local variables 

    //==================================================
    // State entry 

    if (ab_data.state_entry)
    {
        ab_data.state_entry = CLEAR_BIT; 

        uart_sendstring(USART2, "\r\nfault\r\n"); 
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
    // Local variables 

    //==================================================
    // State entry 

    if (ab_data.state_entry)
    {
        ab_data.state_entry = CLEAR_BIT; 

        uart_sendstring(USART2, "\r\nreset\r\n"); 
    }

    //==================================================

    // Go directly to the init function 
    ab_data.init = SET_BIT; 

    // Clear fault and status codes 
    ab_data.fault_code = CLEAR; 
    m8q_set_reset_flag(); 
    lsm303agr_clear_status(); 
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
// Command functions 

// Idle command 
void ab_idle_cmd(
    uint8_t idle_cmd_value)
{
    ab_data.idle = SET_BIT; 
    ab_data.state_entry = SET_BIT; 
    ab_data.manual = CLEAR_BIT; 
    ab_data.autonomous = CLEAR_BIT; 

    // Set the throttle to zero to stop the thrusters 
    ab_data.right_thruster = AB_NO_THRUST; 
    ab_data.left_thruster = AB_NO_THRUST; 
    esc_readytosky_send(DEVICE_ONE, AB_NO_THRUST); 
    esc_readytosky_send(DEVICE_TWO, AB_NO_THRUST); 
}


// Manual control mode command 
void ab_manual_cmd(
    uint8_t manual_cmd_value)
{
    ab_data.manual = SET_BIT; 
    ab_data.state_entry = SET_BIT; 
    ab_data.delay_timer.time_start = SET_BIT; 
    ab_data.idle = CLEAR_BIT; 

    // Make sure LEDs are off 
    ab_data.led_data[WS2812_LED_3] = ab_led_clear; 
    ab_data.led_data[WS2812_LED_4] = ab_led_clear; 
    ws2812_send(DEVICE_ONE, ab_data.led_data); 
}


// Autonomous mode command 
void ab_auto_cmd(
    uint8_t auto_cmd_value)
{
    ab_data.autonomous = SET_BIT; 
    ab_data.state_entry = SET_BIT; 
    ab_data.delay_timer.time_start = SET_BIT; 
    ab_data.idle = CLEAR_BIT; 

    // Make sure LEDs are off 
    ab_data.led_data[WS2812_LED_3] = ab_led_clear; 
    ab_data.led_data[WS2812_LED_4] = ab_led_clear; 
    ws2812_send(DEVICE_ONE, ab_data.led_data); 
}


// Index update command 
void ab_index_cmd(
    uint8_t index_cmd_value)
{
    // Local variables 
    static uint8_t index_check = CLEAR; 
    static uint8_t index_last = CLEAR; 

    // Compare the previous index command to the new index command 
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
    if ((index_cmd_value < AB_NUM_COORDINATES) && (index_check >= AB_GPS_INDEX_CNT))
    {
        ab_data.waypoint_index = index_cmd_value; 
        index_check = CLEAR; 
        uart_sendstring(USART2, "\r\nindex="); 
        uart_send_integer(USART2, (int16_t)ab_data.waypoint_index); 
        uart_send_new_line(USART2); 
    }
}


// Manual throttle command 
void ab_throttle_cmd(
    uint8_t throttle_cmd_value)
{
    ab_data.mc_data = SET_BIT; 
    ab_data.connect = SET_BIT; 
    ab_data.hb_timeout = CLEAR; 
}


// Heartbeat command 
void ab_hb_cmd(
    uint8_t hb_cmd_value)
{
    ab_data.connect = SET_BIT; 
    ab_data.hb_timeout = CLEAR; 
}

//=======================================================================================


//=======================================================================================
// Application functions 

// GPS coordinate radius check - calculate surface distance and compare to threshold 
void ab_gps_rad(void)
{
    // Local variables 
    static double surf_dist = CLEAR; 
    double eq1, eq2, eq3, eq4, eq5; 
    double deg_to_rad = 3.14159/180; 
    double pi_over_2 = 3.14159/2.0; 
    double earth_rad = 6371; 
    double km_to_m = 1000; 

    // Convert coordinates to radians 
    ab_data.location.lat *= deg_to_rad; 
    ab_data.location.lon *= deg_to_rad; 
    ab_data.waypoint.lat *= deg_to_rad; 
    ab_data.waypoint.lon *= deg_to_rad; 

    eq1 = cos(pi_over_2 - ab_data.waypoint.lat)*
          sin(ab_data.waypoint.lon - ab_data.location.lon); 
    eq2 = cos(pi_over_2 - ab_data.location.lat)*sin(pi_over_2 - ab_data.waypoint.lat); 
    eq3 = sin(pi_over_2 - ab_data.location.lat)*cos(pi_over_2 - ab_data.waypoint.lat)* 
          cos(ab_data.waypoint.lon - ab_data.location.lon); 
    eq4 = sin(pi_over_2 - ab_data.location.lat)*sin(pi_over_2 - ab_data.waypoint.lat); 
    eq5 = cos(pi_over_2 - ab_data.location.lat)*cos(pi_over_2 - ab_data.waypoint.lat)*
          cos(ab_data.waypoint.lon - ab_data.location.lon); 

    // atan2 is used because it produces an angle between +/-180 (pi). The central angle 
    // should always be positive and never greater than 180. 
    // Calculate the radius using a low pass filter to smooth the data. 
    surf_dist += ((atan2(sqrt((eq2 - eq3)*(eq2 - eq3) + (eq1*eq1)), (eq4 + eq5)) * 
                 earth_rad*km_to_m) - surf_dist)*0.25; 
    ab_data.waypoint_rad = (uint16_t)(surf_dist*10); 
}


// GPS heading calculation 
void ab_gps_heading(void)
{
    // Local variables 
    static double heading_temp = CLEAR; 
    double num, den; 
    double deg_to_rad = 3.14159/180; 

    // Convert coordinates to radians 
    ab_data.location.lat *= deg_to_rad; 
    ab_data.location.lon *= deg_to_rad; 
    ab_data.waypoint.lat *= deg_to_rad; 
    ab_data.waypoint.lon *= deg_to_rad; 

    // Calculate the numerator and denominator of the atan calculation 
    num = cos(ab_data.waypoint.lat)*sin(ab_data.waypoint.lon-ab_data.location.lon); 
    den = cos(ab_data.location.lat)*sin(ab_data.waypoint.lat) - sin(ab_data.location.lat)*
          cos(ab_data.waypoint.lat)*cos(ab_data.waypoint.lon-ab_data.location.lon); 

    // Calculate the heading between coordinates. 
    // A low pass filter is used to smooth the data. 
    // heading_temp += (atan(num/den) - heading_temp)*0.5; 
    heading_temp = atan(num/den); 

    // Convert heading to degrees 
    ab_data.heading_desired = (int16_t)(heading_temp*10/deg_to_rad); 

    // Correct the calculated heading if needed 
    if (den < 0)
    {
        ab_data.heading_desired += LSM303AGR_M_HEAD_DIFF; 
    }
    else if (num < 0)
    {
        ab_data.heading_desired += LSM303AGR_M_HEAD_MAX; 
    }
}


// Get the true North heading 
void ab_heading(void)
{
    // Get the magnetometer heading and add the true North correction 
    ab_data.heading_actual = lsm303agr_m_get_heading() + AB_TN_COR; 

    // Adjust the true North heading if the corrected headed exceeds heading bounds 
    if (AB_TN_COR >= 0)
    {
        if (ab_data.heading_actual >= LSM303AGR_M_HEAD_MAX)
        {
            ab_data.heading_actual -= LSM303AGR_M_HEAD_MAX; 
        }
    }
    else 
    {
        if (ab_data.heading_actual < 0)
        {
            ab_data.heading_actual += LSM303AGR_M_HEAD_MAX; 
        }
    }
}


// Heading error 
void ab_heading_error(void)
{
    // Calculate the heading error 
    ab_data.heading_error = ab_data.heading_desired - ab_data.heading_actual; 

    // Correct the error for when the heading crosses the 0/360 degree boundary 
    if (ab_data.heading_error > LSM303AGR_M_HEAD_DIFF)
    {
        ab_data.heading_error -= LSM303AGR_M_HEAD_MAX; 
    }
    else if (ab_data.heading_error < -LSM303AGR_M_HEAD_DIFF)
    {
        ab_data.heading_error += LSM303AGR_M_HEAD_MAX; 
    }
}


// Parse the user command into an ID and value 
uint8_t ab_parse_cmd(
    uint8_t *command_buffer)
{
    // Local variables 
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

//=======================================================================================
