/**
 * @file nrf24l01_driver_mock.c
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief nRF24L01 driver mock 
 * 
 * @version 0.1
 * @date 2024-04-12
 * 
 * @copyright Copyright (c) 2024
 * 
 */

//=======================================================================================
// Includes 

#include "nrf24l01_driver_mock.h" 

//=======================================================================================


//=======================================================================================
// Macros 

#define MAX_BUFF_SIZE 32 

//=======================================================================================


//=======================================================================================
// Variables 

typedef struct 
{
    uint8_t read_buff[MAX_BUFF_SIZE]; 
    uint8_t send_buff[MAX_BUFF_SIZE]; 
} 
nrf24l01_mock; 

static nrf24l01_mock mock_data; 

//=======================================================================================


//=======================================================================================
// Driver functions 

// nRF24L01 initialization 
NRF24L01_STATUS nrf24l01_init(
    SPI_TypeDef *spi, 
    GPIO_TypeDef *gpio_ss, 
    pin_selector_t ss_pin, 
    GPIO_TypeDef *gpio_en, 
    pin_selector_t en_pin, 
    TIM_TypeDef *timer, 
    uint8_t rf_ch_freq, 
    nrf24l01_data_rate_t data_rate, 
    nrf24l01_rf_pwr_t rf_pwr)
{
    return NRF24L01_OK; 
}


// Configure a devices PTX settings 
NRF24L01_STATUS nrf24l01_ptx_config(const uint8_t *tx_addr)
{
    return NRF24L01_OK; 
}


// Configure a devices PRX settings 
NRF24L01_STATUS nrf24l01_prx_config(
    const uint8_t *rx_addr, 
    nrf24l01_data_pipe_t pipe_num)
{
    return NRF24L01_OK; 
}


// Data available status 
DATA_PIPE nrf24l01_data_ready_status(void)
{
    return NRF24L01_DP_1; 
}


// Receive payload 
NRF24L01_STATUS nrf24l01_receive_payload(uint8_t *read_buff)
{
    if (read_buff == NULL)
    {
        return NRF24L01_INVALID_PTR; 
    }

    memcpy((void *)read_buff, (void *)mock_data.read_buff, sizeof(mock_data.read_buff)); 

    return NRF24L01_OK; 
}


// Send payload 
NRF24L01_STATUS nrf24l01_send_payload(const uint8_t *data_buff)
{
    memcpy((void *)mock_data.send_buff, (void *)data_buff, sizeof(data_buff)); 
    return NRF24L01_OK; 
}


// RF_CH register update 
NRF24L01_STATUS nrf24l01_rf_ch_read(void)
{
    return NRF24L01_OK; 
}


// Get RF_CH channel 
uint8_t nrf24l01_get_rf_ch(void)
{
    return 0; 
}


// Set RF_CH channel 
void nrf24l01_set_rf_ch(uint8_t rf_ch_freq)
{
    // 
}


// RF_CH register write 
NRF24L01_STATUS nrf24l01_rf_ch_write(void)
{
    return NRF24L01_OK; 
}


// RF_SETUP register read 
NRF24L01_STATUS nrf24l01_rf_setup_read(void)
{
    return NRF24L01_OK; 
}


// Get RF_SETUP data rate 
nrf24l01_data_rate_t nrf24l01_get_rf_setup_dr(void)
{
    return NRF24L01_DR_250KBPS; 
}


// Get RF_SETUP power output 
nrf24l01_rf_pwr_t nrf24l01_get_rf_setup_pwr(void)
{
    return NRF24L01_RF_PWR_0DBM; 
}


// Set RF_SETUP data rate 
void nrf24l01_set_rf_setup_dr(nrf24l01_data_rate_t rate)
{
    // 
}


// Set RF_SETUP power output 
void nrf24l01_set_rf_setup_pwr(nrf24l01_rf_pwr_t rf_pwr)
{
    // 
}


// RF_SETUP register write 
NRF24L01_STATUS nrf24l01_rf_setup_write(void)
{
    return NRF24L01_OK; 
}


// CONFIG register read 
NRF24L01_STATUS nrf24l01_config_read(void)
{
    return NRF24L01_OK; 
}


// Get power mode 
nrf24l01_pwr_mode_t nrf24l01_get_config_pwr_mode(void)
{
    return NRF24L01_PWR_UP; 
}


// Get active mode 
nrf24l01_mode_select_t nrf24l01_get_config_mode(void)
{
    return NRF24L01_RX_MODE; 
}


// Power down 
NRF24L01_STATUS nrf24l01_pwr_down(void)
{
    return NRF24L01_OK; 
}


// Power up 
NRF24L01_STATUS nrf24l01_pwr_up(void)
{
    return NRF24L01_OK; 
}

//=======================================================================================


//=======================================================================================
// Mock functions 

// Set the data to be read 
void nrf24l01_mock_set_read_data(
    uint8_t *read_buff_set, 
    uint8_t buff_size) 
{
    memset((void *)mock_data.read_buff, CLEAR, sizeof(mock_data.read_buff)); 
    
    if (read_buff_set != NULL)
    {
        memcpy((void *)mock_data.read_buff, (void *)read_buff_set, buff_size); 
    }
}


// Get the sent data 
void nrf24l01_mock_get_send_data(uint8_t *send_buff_get)
{
    if (send_buff_get != NULL)
    {
        memcpy((void *)send_buff_get, (void *)mock_data.send_buff, sizeof(mock_data.send_buff)); 
    }
}

//=======================================================================================
