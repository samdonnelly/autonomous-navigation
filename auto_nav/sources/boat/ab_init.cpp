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

#include "ab_init.h"
#include "ab_includes_app.h"

//=======================================================================================


//=======================================================================================
// Global variables 

// Magnetometer directional offsets to correct for heading errors 
static int16_t mag_offsets[AB_MAG_NUM_DIRS]; 

// RF module data pipe address 
static uint8_t pipe_addr_buff[NRF24l01_ADDR_WIDTH] = {0xB3, 0xB4, 0xB5, 0xB6, 0x05}; 

//=======================================================================================


//=======================================================================================
// Functions 

// Autonomous boat initialization 
void ab_init(void)
{
    // Autonomous boat initialization code 

    //===================================================
    // General setup 

    // Initialize GPIO ports 
    gpio_port_init(); 

    //===================================================

    //===================================================
    // Timers 

    // General purpose 1us counter 
    tim_9_to_11_counter_init(
        TIM9, 
        TIM_84MHZ_1US_PSC, 
        0xFFFF,  // Max ARR value 
        TIM_UP_INT_DISABLE); 
    tim_enable(TIM9); 

    //===================================================

    //===================================================
    // I2C 

    // Screen, GPS, IMU and magnetometer 
    i2c_init(
        I2C1, 
        PIN_9, 
        GPIOB, 
        PIN_8, 
        GPIOB, 
        I2C_MODE_SM,
        I2C_APB1_42MHZ,
        I2C_CCR_SM_42_100,
        I2C_TRISE_1000_42);

    //===================================================

    //===================================================
    // SPI 

    // RF module 
    spi_init(
        SPI2, 
        GPIOB,   // GPIO port for SCK pin 
        PIN_10,  // SCK pin 
        GPIOC,   // GPIO port for data (MISO/MOSI) pins 
        PIN_2,   // MISO pin 
        PIN_3,   // MOSI pin 
        SPI_BR_FPCLK_16, 
        SPI_CLOCK_MODE_0); 
    
    // SD card (if possible) 

    //===================================================

    //===================================================
    // UART 

    // Initialize UART - serial terminal - for testing 
    uart_init(
        USART2, 
        GPIOA, 
        PIN_3, 
        PIN_2, 
        UART_FRAC_42_9600, 
        UART_MANT_42_9600, 
        UART_DMA_DISABLE, 
        UART_DMA_DISABLE); 

    // Bluetooth module 
    uart_init(
        USART1, 
        GPIOA, 
        PIN_10, 
        PIN_9, 
        UART_FRAC_84_115200, 
        UART_MANT_84_115200, 
        UART_DMA_DISABLE, 
        UART_DMA_DISABLE); 

    //===================================================

    //===================================================
    // ADC 

    // Two batteries and one PSU (if possible) 

    // Initialize the ADC port (called once) 
    adc_port_init(
        ADC1, 
        ADC1_COMMON, 
        ADC_PCLK2_4, 
        ADC_RES_8, 
        ADC_EOC_EACH, 
        ADC_SCAN_ENABLE, 
        ADC_CONT_DISABLE, 
        ADC_DMA_ENABLE, 
        ADC_DDS_ENABLE, 
        ADC_EOC_INT_DISABLE, 
        ADC_OVR_INT_DISABLE); 

    // Initialize the ADC pins and channels 
    adc_pin_init(ADC1, GPIOA, PIN_6, ADC_CHANNEL_6, ADC_SMP_15); 
    adc_pin_init(ADC1, GPIOA, PIN_7, ADC_CHANNEL_7, ADC_SMP_15); 
    adc_pin_init(ADC1, GPIOA, PIN_4, ADC_CHANNEL_4, ADC_SMP_15); 

    // Set the ADC conversion sequence 
    adc_seq(ADC1, ADC_CHANNEL_6, ADC_SEQ_1); 
    adc_seq(ADC1, ADC_CHANNEL_7, ADC_SEQ_2); 
    adc_seq(ADC1, ADC_CHANNEL_4, ADC_SEQ_3); 

    // Set the sequence length (called once and only for more than one channel) 
    adc_seq_len_set(ADC1, (adc_seq_num_t)AB_ADC_BUFF_SIZE); 

    // Turn the ADC on 
    adc_on(ADC1); 

    //===================================================

    //===================================================
    // DMA 

    // Initialize the DMA stream 
    dma_stream_init(
        DMA2, 
        DMA2_Stream0, 
        DMA_CHNL_0, 
        DMA_DIR_PM, 
        DMA_CM_ENABLE,
        DMA_PRIOR_VHI, 
        DMA_ADDR_INCREMENT, 
        DMA_ADDR_FIXED, 
        DMA_DATA_SIZE_HALF, 
        DMA_DATA_SIZE_HALF); 

    // The DMA stream gets configured in the application code init (below) 
    // The stream gets enabled after the application code init 

    //===================================================

    //===================================================
    // Screen 

    // Must come before setup of other devices on the same I2C bus 

    // Initialize then set to low power mode - not needed for this application 
    hd44780u_init(I2C1, TIM9, PCF8574_ADDR_HHH); 
    hd44780u_clear(); 
    hd44780u_display_off(); 
    hd44780u_backlight_off(); 

    //===================================================

    //===================================================
    // GPS 

    // Get the configuration messages to send to the device 
    char m8q_config_messages[M8Q_CONFIG_MSG_NUM][M8Q_CONFIG_MSG_MAX_LEN]; 
    m8q_config_copy(m8q_config_messages); 

    // Driver 
    m8q_init(
        I2C1, 
        GPIOC, 
        PIN_10, 
        PIN_11, 
        M8Q_CONFIG_MSG_NUM, 
        M8Q_CONFIG_MSG_MAX_LEN, 
        (uint8_t *)m8q_config_messages[0]); 

    // Controller 
    m8q_controller_init(TIM9); 

    //===================================================

    //===================================================
    // IMU 

    // Initialize then set to low power mode - not needed for this application 
    mpu6050_init(
        DEVICE_ONE, 
        I2C1, 
        MPU6050_ADDR_1,
        AB_IMU_STBY_MASK, 
        MPU6050_DLPF_CFG_1,
        AB_IMU_SMPLRT_DIVIDER,
        MPU6050_AFS_SEL_4,
        MPU6050_FS_SEL_500); 
    mpu6050_low_pwr_config(DEVICE_ONE, MPU6050_SLEEP_MODE_ENABLE); 
    
    //===================================================

    //===================================================
    // Magnetometer 

    // Magnetometer reading error offsets (expresses as degrees*10) 
    // See the test code for calibration steps 
    mag_offsets[0] = -160;     // N  (0/360deg) direction heading offset 
    mag_offsets[1] = 32;       // NE (45deg) direction heading offset 
    mag_offsets[2] = 215;      // E  (90deg) direction heading offset 
    mag_offsets[3] = 385;      // SE (135deg) direction heading offset 
    mag_offsets[4] = 435;      // S  (180deg) direction heading offset 
    mag_offsets[5] = 20;       // SW (225deg) direction heading offset 
    mag_offsets[6] = -450;     // W  (270deg) direction heading offset 
    mag_offsets[7] = -365;     // NW (315deg) direction heading offset 

    // Driver init 
    lsm303agr_init(
        I2C1, 
        mag_offsets, 
        LSM303AGR_M_ODR_10, 
        LSM303AGR_M_MODE_CONT, 
        LSM303AGR_CFG_DISABLE, 
        LSM303AGR_CFG_DISABLE, 
        LSM303AGR_CFG_DISABLE, 
        LSM303AGR_CFG_DISABLE); 
    
    //===================================================

    //===================================================
    // Bluetooth 

    // Initialize then set to low power mode - not needed for this application 
    hc05_init(
        USART1, 
        TIM9, 
        GPIOA,          // AT pin GPIO 
        PIN_8,          // AT pin 
        GPIOA,          // EN pin GPIO 
        PIN_12,         // EN pin 
        GPIOA,          // STATE pin GPIO 
        PIN_11);        // STATE pin 
    hc05_off(); 

    //===================================================

    //===================================================
    // RF module 

    // General setup common to all device - must be called once during setup 
    nrf24l01_init(
        SPI2,    // SPI port to use 
        GPIOC,   // Slave select pin GPIO port 
        PIN_1,   // Slave select pin number 
        GPIOC,   // Enable pin (CE) GPIO port 
        PIN_0,   // Enable pin (CE) number 
        TIM9);   // General purpose timer port 

    // Set the devices initial communication parameters - can be updated as needed 
    nrf24l01_set_rf_channel(AB_RF_CH_FREQ); 
    nrf24l01_set_rf_dr(NRF24L01_DR_2MBPS); 
    nrf24l01_set_rf_pwr(NRF24L01_RF_PWR_6DBM); 

    // Configure the devices PRX settings 
    nrf24l01_prx_config(pipe_addr_buff, NRF24L01_DP_1); 

    //===================================================

    //===================================================
    // SD card (if possible) 
    //===================================================

    //===================================================
    // LEDs 

    // The timer port (not just the channel) used for the LEDs must be different than the timer 
    // used for the ESCs because they run at different speeds and the WS2812 driver turns the 
    // timer on and off for sending. 

    // WS2812 (Neopixel LEDs) 
    ws2812_init(
        DEVICE_ONE, 
        TIM4, 
        TIM_CHANNEL_2, 
        GPIOB, 
        PIN_7); 
    
    //===================================================

    //===================================================
    // ESCs/Motors 

    // ESC 1 - right 
    esc_readytosky_init(
        DEVICE_ONE, 
        TIM3, 
        TIM_CHANNEL_4, 
        GPIOB, 
        PIN_1, 
        TIM_84MHZ_1US_PSC, 
        AB_ESC_PERIOD, 
        AB_R_ESC_FWD_SPD_LIM, 
        AB_R_ESC_REV_SPD_LIM); 

    // ESC 2 - left 
    esc_readytosky_init(
        DEVICE_TWO, 
        TIM3, 
        TIM_CHANNEL_3, 
        GPIOB, 
        PIN_0, 
        TIM_84MHZ_1US_PSC, 
        AB_ESC_PERIOD, 
        AB_L_ESC_FWD_SPD_LIM, 
        AB_L_ESC_REV_SPD_LIM); 

    // Enable the PWM timer - TIM3 set up in the ESC init 
    tim_enable(TIM3); 

    //===================================================

    //===================================================
    // System setup 

    // Application code setup 
    ab_app_init(
        TIM9, 
        DMA2_Stream0, 
        ADC1, 
        NRF24L01_DP_1); 

    // Enable the DMA stream - done after the application code initialization so the 
    // DMA can finish being set up. 
    dma_stream_enable(DMA2_Stream0); 

    //===================================================
}

//=======================================================================================
