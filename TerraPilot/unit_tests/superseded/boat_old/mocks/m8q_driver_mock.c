/**
 * @file m8q_driver_mock.c
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief M8Q driver mock 
 * 
 * @version 0.1
 * @date 2024-06-12
 * 
 * @copyright Copyright (c) 2024
 * 
 */

//=======================================================================================
// Includes 

#include "m8q_driver_mock.h" 

//=======================================================================================


//=======================================================================================
// Macros 
//=======================================================================================


//=======================================================================================
// Variables 
//=======================================================================================


//=======================================================================================
// Driver functions 

// Device initialization 
M8Q_STATUS m8q_init(
    I2C_TypeDef *i2c, 
    const char *config_msgs, 
    uint8_t msg_num, 
    uint8_t max_msg_size, 
    uint16_t data_buff_limit)
{
    return 0; 
}


// Low power pin initialization 
M8Q_STATUS m8q_pwr_pin_init(
    GPIO_TypeDef *gpio, 
    pin_selector_t pwr_save_pin)
{
    return 0; 
}


// TX ready pin initialization 
M8Q_STATUS m8q_txr_pin_init(
    GPIO_TypeDef *gpio, 
    pin_selector_t tx_ready_pin)
{
    return 0; 
}


// Read and store relevant message data 
M8Q_STATUS m8q_read_data(void)
{
    return 0; 
}


// Read and return the data stream contents 
M8Q_STATUS m8q_read_ds(
    uint8_t *data_buff, 
    uint16_t buff_size)
{
    return 0; 
}


// Read data stream size 
M8Q_STATUS m8q_read_ds_size(uint16_t *data_size)
{
    return 0; 
}


// Return the ACK/NAK message counter status 
uint16_t m8q_get_ack_status(void)
{
    return 0; 
}


// Send a message to the device 
M8Q_STATUS m8q_send_msg(
    const char *write_msg, 
    uint8_t max_msg_size)
{
    return 0; 
}


// Get TX ready status 
GPIO_STATE m8q_get_tx_ready(void)
{
    return 0; 
}


// Enter low power mode 
void m8q_set_low_pwr(void)
{
    // 
}


// Exit low power mode 
void m8q_clear_low_pwr(void)
{
    // 
}


// Get latitude coordinate 
double m8q_get_position_lat(void)
{
    return 0; 
}


// Get latitude coordinate string 
M8Q_STATUS m8q_get_position_lat_str(
    uint8_t *lat_str, 
    uint8_t lat_str_len)
{
    return 0; 
}


// Get North/South hemisphere 
uint8_t m8q_get_position_NS(void)
{
    return 0; 
}


// Get longitude coordinate 
double m8q_get_position_lon(void)
{
    return 0; 
}


// Get longitude coordinate string 
M8Q_STATUS m8q_get_position_lon_str(
    uint8_t *lon_str, 
    uint8_t lon_str_len)
{
    return 0; 
}


// Get East/West hemisphere 
uint8_t m8q_get_position_EW(void)
{
    return 0; 
}


// Get navigation status 
uint16_t m8q_get_position_navstat(void)
{
    return 0; 
}


// Get acceptable navigation status 
uint8_t m8q_get_position_navstat_lock(void)
{
    return 0; 
}


// Get UTC time 
M8Q_STATUS m8q_get_time_utc_time(
    uint8_t *utc_time, 
    uint8_t utc_time_len)
{
    return 0; 
}


// Get UTC date 
M8Q_STATUS m8q_get_time_utc_date(
    uint8_t *utc_date, 
    uint8_t utc_date_len)
{
    return 0; 
}

//=======================================================================================


//=======================================================================================
// Mock functions 
//=======================================================================================

