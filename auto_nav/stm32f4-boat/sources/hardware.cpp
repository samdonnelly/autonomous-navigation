/**
 * @file vehicle_hardware.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Vehicle hardware 
 * 
 * @details This file is added by the project (not the autopilot) to define the vehicle 
 *          hardware functions. These functions provide a hardware specific interface 
 *          that's unique to each project that uses the autopilot. The autopilot 
 *          purposely does not define these because their definition will change 
 *          depending on what the user wants to use. 
 * 
 * @version 0.1
 * @date 2025-02-27
 * 
 * @copyright Copyright (c) 2025
 * 
 */

//=======================================================================================
// Include 

// Autopilot 
#include "vehicle.h" 

// Project 
#include "stm32f4xx_it.h" 
#include "includes_drivers.h" 
#include "esc_config.h" 
#include "lsm303agr_config.h" 
#include "m8q_config.h" 
#include "mpu6050_config.h" 
#include "nrf24l01_config.h" 
#include "ws2812_config.h" 

//=======================================================================================


//=======================================================================================
// Macros 

// Buffer sizes 
#define SERIAL_MSG_BUFF_SIZE 1000 
#define ADC_BUFF_SIZE 3 

//=======================================================================================


//=======================================================================================
// Hardware data 

class Hardware 
{
public:   // public members 

    // UART/Serial 
    struct SerialData 
    {
        USART_TypeDef *uart; 
        DMA_Stream_TypeDef *dma_stream; 
        uint8_t cb[SERIAL_MSG_BUFF_SIZE];              // Circular buffer populated by DMA 
        cb_index_t cb_index;                           // Circular buffer indexing info 
        dma_index_t dma_index;                         // DMA transfer indexing info 
        uint8_t data_in_buff[SERIAL_MSG_BUFF_SIZE];    // Buffer that stores latest UART input 
        uint16_t data_in_index;                        // Data input buffer index 
        uint8_t data_out_buff[SERIAL_MSG_BUFF_SIZE];   // Buffer that stores outgoing data 
        uint16_t data_out_size;                        // Size of the outgoing data 
    }
    telemetry_data, rc_data; 

    // Serial debug 
    USART_TypeDef *user_uart; 

    // ADC 
    ADC_TypeDef *adc; 
    DMA_Stream_TypeDef *adc_dma_stream; 
    uint16_t adc_buff[ADC_BUFF_SIZE];     // ADC buffer - battery and PSU voltage 

    // Timers 
    TIM_TypeDef *generic_timer; 
    TIM_TypeDef *esc_timer; 

public:   // public methods 

    // Helper functions 
    void SerialDataInit(
        SerialData &serial_data, 
        USART_TypeDef *uart, 
        DMA_Stream_TypeDef *dma_stream); 
};

static Hardware hardware; 

//=======================================================================================


//=======================================================================================
// Initialization 

void VehicleHardware::HardwareSetup(void)
{
    //==================================================
    // General 

    // Initialize GPIO ports 
    gpio_port_init(); 

    // UART/Serial data 
    hardware.SerialDataInit(hardware.telemetry_data, USART1, DMA2_Stream2); 
    // hardware.SerialDataInit(hardware.rc_data, USARTX, DMAX_StreamX); 

    // Serial debug data 
    hardware.user_uart = USART2; 

    // DMA data 
    hardware.adc = ADC1; 
    hardware.adc_dma_stream = DMA2_Stream0; 
    memset((void *)hardware.adc_buff, CLEAR, sizeof(hardware.adc_buff)); 

    // Timers 
    hardware.generic_timer = TIM9; 
    hardware.esc_timer = TIM3; 

    //==================================================

    //==================================================
    // Timers 

    // General purpose 1us counter 
    tim_9_to_11_counter_init(
        hardware.generic_timer, 
        TIM_84MHZ_1US_PSC, 
        0xFFFF,  // Max ARR value 
        TIM_UP_INT_DISABLE); 
    tim_enable(hardware.generic_timer); 

    //==================================================

    //==================================================
    // UART 
    
    // UART1 init - SiK radio module 
    uart_init(
        hardware.telemetry_data.uart, 
        GPIOA, 
        PIN_10, 
        PIN_9, 
        UART_FRAC_84_57600, 
        UART_MANT_84_57600, 
        UART_DMA_DISABLE, 
        UART_DMA_ENABLE); 

    // UART1 interrupt init - SiK radio module - IDLE line (RX) interrupts 
    uart_interrupt_init(
        hardware.telemetry_data.uart, 
        UART_INT_DISABLE, 
        UART_INT_DISABLE, 
        UART_INT_DISABLE, 
        UART_INT_DISABLE, 
        UART_INT_ENABLE, 
        UART_INT_DISABLE, 
        UART_INT_DISABLE); 

    // UART2 init - Serial terminal 
    uart_init(
        hardware.user_uart, 
        GPIOA, 
        PIN_3, 
        PIN_2, 
        UART_FRAC_42_9600, 
        UART_MANT_42_9600, 
        UART_DMA_DISABLE, 
        UART_DMA_DISABLE); 

    // // UART6 init - RC receiver 
    // uart_init(
    //     rc_data.uart, 
    //     GPIOX, 
    //     PIN_X, 
    //     PIN_X, 
    //     UART_FRAC_42_9600, 
    //     UART_MANT_42_9600, 
    //     UART_DMA_DISABLE, 
    //     UART_DMA_ENABLE); 

    // // UART6 interrupt init - RC receiver - IDLE line (RX) interrupts 
    // uart_interrupt_init(
    //     rc_data.uart, 
    //     UART_INT_DISABLE, 
    //     UART_INT_DISABLE, 
    //     UART_INT_DISABLE, 
    //     UART_INT_DISABLE, 
    //     UART_INT_ENABLE, 
    //     UART_INT_DISABLE, 
    //     UART_INT_DISABLE); 

    //==================================================

    //==================================================
    // SPI 
    
    // // SPI 2 init - SD card 
    // spi_init(
    //     SPI2, 
    //     GPIOB,               // GPIO port for SCK pin 
    //     PIN_10,              // SCK pin 
    //     GPIOB,               // GPIO port for data (MISO/MOSI) pins 
    //     PIN_14,              // MISO pin 
    //     PIN_15,              // MOSI pin 
    //     SPI_BR_FPCLK_8, 
    //     SPI_CLOCK_MODE_0); 

    // // SD card slave select pin setup 
    // spi_ss_init(GPIOB, PIN_12); 

    //==================================================

    //==================================================
    // I2C 
    
    // // I2C: GPS, IMU and magnetometer 
    // i2c_init(
    //     I2C1, 
    //     PIN_9, 
    //     GPIOB, 
    //     PIN_8, 
    //     GPIOB, 
    //     I2C_MODE_SM,
    //     I2C_APB1_42MHZ,
    //     I2C_CCR_SM_42_100,
    //     I2C_TRISE_1000_42);
    
    //==================================================

    //==================================================
    // ADC 

    // // Main and auxiliary battery voltages and 5V PSU voltage 

    // // Initialize the ADC port 
    // adc1_clock_enable(RCC); 
    // adc_port_init(
    //     hardware.adc, 
    //     ADC1_COMMON, 
    //     ADC_PCLK2_4, 
    //     ADC_RES_8, 
    //     ADC_PARAM_ENABLE,      // ADC_EOC_EACH 
    //     ADC_PARAM_DISABLE,     // ADC_EOC_INT_DISABLE 
    //     ADC_PARAM_ENABLE,      // ADC_SCAN_ENABLE 
    //     ADC_PARAM_DISABLE,     // ADC_CONT_DISABLE 
    //     ADC_PARAM_ENABLE,      // ADC_DMA_ENABLE 
    //     ADC_PARAM_ENABLE,      // ADC_DDS_ENABLE 
    //     ADC_PARAM_DISABLE);    // ADC_OVR_INT_DISABLE 

    // // Initialize the ADC pins and channels 
    // adc_pin_init(hardware.adc, GPIOA, PIN_6, ADC_CHANNEL_6, ADC_SMP_15);   // Main battery voltage 
    // adc_pin_init(hardware.adc, GPIOA, PIN_7, ADC_CHANNEL_7, ADC_SMP_15);   // Auxiliary battery voltage 
    // adc_pin_init(hardware.adc, GPIOA, PIN_4, ADC_CHANNEL_4, ADC_SMP_15);   // 5V PSU voltage 

    // // Set the ADC conversion sequence 
    // adc_seq(hardware.adc, ADC_CHANNEL_6, ADC_SEQ_1);   // Main battery voltage 
    // adc_seq(hardware.adc, ADC_CHANNEL_7, ADC_SEQ_2);   // Auxiliary battery voltage 
    // adc_seq(hardware.adc, ADC_CHANNEL_4, ADC_SEQ_3);   // 5V PSU voltage 

    // // Set the sequence length (called once and only for more than one channel) 
    // adc_seq_len_set(hardware.adc, (adc_seq_num_t)ADC_BUFF_SIZE); 

    // // Turn the ADC on 
    // adc_on(hardware.adc); 

    //==================================================

    //==================================================
    // DMA 
    
    // DMA2 stream init - UART1 - SiK radio module 
    dma_stream_init(
        DMA2, 
        hardware.telemetry_data.dma_stream, 
        DMA_CHNL_4, 
        DMA_DIR_PM, 
        DMA_CM_ENABLE,
        DMA_PRIOR_HI, 
        DMA_DBM_DISABLE, 
        DMA_ADDR_INCREMENT,   // Increment the buffer pointer to fill the buffer 
        DMA_ADDR_FIXED,       // No peripheral increment - copy from DR only 
        DMA_DATA_SIZE_BYTE, 
        DMA_DATA_SIZE_BYTE); 
        
    // DMA2 stream config - UART1 - SiK radio module 
    dma_stream_config(
        hardware.telemetry_data.dma_stream, 
        (uint32_t)(&hardware.telemetry_data.uart->DR), 
        (uint32_t)hardware.telemetry_data.cb, 
        (uint32_t)NULL, 
        (uint16_t)SERIAL_MSG_BUFF_SIZE); 

    // // DMAX stream init - UART6 - RC receiver 
    // dma_stream_init(
    //     DMAX, 
    //     hardware.rc_data.dma_stream, 
    //     DMA_CHNL_4, 
    //     DMA_DIR_PM, 
    //     DMA_CM_ENABLE,
    //     DMA_PRIOR_HI, 
    //     DMA_DBM_DISABLE, 
    //     DMA_ADDR_INCREMENT,   // Increment the buffer pointer to fill the buffer 
    //     DMA_ADDR_FIXED,       // No peripheral increment - copy from DR only 
    //     DMA_DATA_SIZE_BYTE, 
    //     DMA_DATA_SIZE_BYTE); 
    
    // // DMAX stream config - UART6 - RC receiver 
    // dma_stream_config(
    //     hardware.rc_data.dma_stream, 
    //     (uint32_t)(&hardware.rc_data.uart->DR), 
    //     (uint32_t)hardware.rc_data.cb, 
    //     (uint32_t)NULL, 
    //     (uint16_t)SERIAL_MSG_BUFF_SIZE); 

    // // DMA2 stream init - ADC1 - Voltages 
    // dma_stream_init(
    //     DMA2, 
    //     hardware.adc_dma_stream, 
    //     DMA_CHNL_0, 
    //     DMA_DIR_PM, 
    //     DMA_CM_ENABLE,
    //     DMA_PRIOR_VHI, 
    //     DMA_DBM_DISABLE, 
    //     DMA_ADDR_INCREMENT, 
    //     DMA_ADDR_FIXED, 
    //     DMA_DATA_SIZE_HALF, 
    //     DMA_DATA_SIZE_HALF); 

    // // DMA2 stream config - ADC1 - Voltages 
    // dma_stream_config(
    //     hardware.adc_dma_stream, 
    //     (uint32_t)(&hardware.adc->DR), 
    //     (uint32_t)hardware.adc_buff, 
    //     (uint32_t)NULL, 
    //     (uint16_t)ADC_BUFF_SIZE); 

    // Enable DMA streams 
    dma_stream_enable(hardware.telemetry_data.dma_stream);   // UART1 - Sik radio 
    // dma_stream_enable(hardware.rc_data.dma_stream);          // UART6 - RC receiver 
    // dma_stream_enable(hardware.adc_dma_stream);              // ADC1 - Voltages 

    //==================================================

    //==================================================
    // Interrupts 

    // Initialize interrupt handler flags 
    int_handler_init(); 

    // Enable the interrupt handlers 
    nvic_config(USART1_IRQn, EXTI_PRIORITY_0);   // UART1 - SiK radio 
    // nvic_config(USART6_IRQn, EXTI_PRIORITY_1);   // UART6 - RC receiver 
    // nvic_config(ADC_IRQn, EXTI_PRIORITY_2);      // ADC1? 

    //==================================================

    //==================================================
    // GPS 

    // // SAM-M8Q module driver init 
    // m8q_init(
    //     I2C1, 
    //     &m8q_config_msgs[0][0], 
    //     M8Q_CONFIG_MSG_NUM, 
    //     M8Q_CONFIG_MSG_MAX_LEN, 
    //     CLEAR); 

    // // Set up low power and TX ready pins 
    // m8q_pwr_pin_init(GPIOC, PIN_10); 
    // m8q_txr_pin_init(GPIOC, PIN_11); 

    // // SAM-M8Q module controller init 
    // m8q_controller_init(hardware.generic_timer); 

    //==================================================

    //==================================================
    // IMUs 

    // // MPU-6050 module driver init 
    // mpu6050_init(
    //     DEVICE_ONE, 
    //     I2C1, 
    //     MPU6050_ADDR_1,
    //     mpu6050_standby_mask, 
    //     MPU6050_DLPF_CFG_1,
    //     mpu6050_sample_rate_divider,
    //     MPU6050_AFS_SEL_4,
    //     MPU6050_FS_SEL_500); 

    // // LSM303AGR module driver init 
    // lsm303agr_m_init(
    //     I2C1, 
    //     lsm303agr_config_dir_offsets, 
    //     lsm303agr_lpf_gain, 
    //     LSM303AGR_M_ODR_10, 
    //     LSM303AGR_M_MODE_CONT, 
    //     LSM303AGR_CFG_DISABLE, 
    //     LSM303AGR_CFG_DISABLE, 
    //     LSM303AGR_CFG_DISABLE, 
    //     LSM303AGR_CFG_DISABLE); 

    //==================================================

    //==================================================
    // Radios 

    sik_init(hardware.telemetry_data.uart); 

    //==================================================

    //==================================================
    // SD Card 

    // // User initialization 
    // hw125_user_init(SPI2, GPIOB, GPIOX_PIN_12); 

    // // Controller init 
    // hw125_controller_init("auto_boat"); 

    //==================================================

    //==================================================
    // LEDs 

    // The timer port (not just the channel) used for the LEDs must be different than the 
    // timer used for the ESCs because they run at different speeds and the WS2812 driver 
    // turns the timer on and off for sending. 

    // // WS2812 module driver init (Neopixel LEDs) - two LEDs run on one signal 
    // ws2812_init(
    //     DEVICE_ONE, 
    //     TIM4, 
    //     TIMER_CH2, 
    //     GPIOB, 
    //     PIN_7); 
    
    //==================================================

    //==================================================
    // ESCs 

    // The timer port (not just the channel) used for the ESCs must be different than the 
    // timer used for the LEDs because they run at different speeds and the WS2812 driver 
    // turns the timer on and off for sending. 

    // // ESC driver init - right thruster (ESC1) 
    // esc_readytosky_init(
    //     DEVICE_ONE, 
    //     hardware.esc_timer, 
    //     TIMER_CH4, 
    //     GPIOB, 
    //     PIN_1, 
    //     TIM_84MHZ_1US_PSC, 
    //     esc_period, 
    //     esc1_fwd_speed_lim, 
    //     esc1_rev_speed_lim); 

    // // ESC driver init - left thruster (ESC2) 
    // esc_readytosky_init(
    //     DEVICE_TWO, 
    //     hardware.esc_timer, 
    //     TIMER_CH3, 
    //     GPIOB, 
    //     PIN_0, 
    //     TIM_84MHZ_1US_PSC, 
    //     esc_period, 
    //     esc2_fwd_speed_lim, 
    //     esc2_rev_speed_lim); 

    // // Enable the PWM timer 
    // tim_enable(hardware.esc_timer); 

    //==================================================
}


// Serial data initialization 
void Hardware::SerialDataInit(
    Hardware::SerialData &serial_data, 
    USART_TypeDef *uart, 
    DMA_Stream_TypeDef *dma_stream)
{
    serial_data.uart = uart; 
    serial_data.dma_stream = dma_stream; 
    memset((void *)serial_data.cb, CLEAR, sizeof(serial_data.cb)); 
    serial_data.cb_index.cb_size = SERIAL_MSG_BUFF_SIZE; 
    serial_data.cb_index.head = CLEAR; 
    serial_data.cb_index.tail = CLEAR; 
    serial_data.dma_index.data_size = CLEAR; 
    serial_data.dma_index.ndt_old = dma_ndt_read(serial_data.dma_stream); 
    serial_data.dma_index.ndt_new = CLEAR; 
    memset((void *)serial_data.data_in_buff, CLEAR, sizeof(serial_data.data_in_buff)); 
    serial_data.data_in_index = CLEAR; 
    memset((void *)serial_data.data_out_buff, CLEAR, sizeof(serial_data.data_out_buff)); 
    serial_data.data_out_size = CLEAR; 
}

//=======================================================================================


//=======================================================================================
// Interrupt callbacks 

// Any needed callbacks are overridden here so hardware data doesn't need to be included 
// in the interrupt file. 

//=======================================================================================


//=======================================================================================
// GPS 

// void VehicleHardware::GPS_Read(void)
// {
//     // 
// }


// void VehicleHardware::GPS_Get(void)
// {
//     // 
// }

//=======================================================================================


//=======================================================================================
// Compass 

// void VehicleHardware::CompassRead(void)
// {
//     // 
// }

//=======================================================================================


//=======================================================================================
// IMU 

// void VehicleHardware::IMU_Read(void)
// {
//     // 
// }

//=======================================================================================


//=======================================================================================
// Telemetry 

uint8_t VehicleHardware::TelemetryRead(void)
{
    // The telemetry radio communicates via UART and incoming data is processed using DMA 
    // and an interrupt. So instead of a direct call to a read function we check if the 
    // interrupt was run to set the data ready flag. 

    uint8_t data_ready_status = FLAG_CLEAR; 

    if (handler_flags.usart1_flag)
    {
        handler_flags.usart1_flag = CLEAR_BIT; 
        data_ready_status = FLAG_SET; 

        // Parse the new radio data from the circular buffer into the data buffer. 
        dma_cb_index(
            hardware.telemetry_data.dma_stream, 
            &hardware.telemetry_data.dma_index, 
            &hardware.telemetry_data.cb_index); 
        cb_parse(
            hardware.telemetry_data.cb, 
            &hardware.telemetry_data.cb_index, 
            hardware.telemetry_data.data_in_buff); 
    }

    return data_ready_status; 
}


void VehicleHardware::TelemetryGet(uint16_t &size, uint8_t *buffer)
{
    size = hardware.telemetry_data.dma_index.data_size; 
    memcpy((void *)buffer, (void *)hardware.telemetry_data.data_in_buff, size); 
}


void VehicleHardware::TelemetrySet(uint16_t &size, uint8_t *buffer)
{
    hardware.telemetry_data.data_out_size = size; 
    memcpy((void *)hardware.telemetry_data.data_out_buff, (void *)buffer, size); 
}


void VehicleHardware::TelemetryWrite(void)
{
    sik_send_data(hardware.telemetry_data.data_out_buff, hardware.telemetry_data.data_out_size); 
}

//=======================================================================================


//=======================================================================================
// RC 

// void VehicleHardware::RC_Read(void)
// {
//     // 
// }

//=======================================================================================


//=======================================================================================
// Memory 

// void VehicleHardware::MemoryRead(void)
// {
//     // 
// }


// void VehicleHardware::MemoryWrite(void)
// {
//     // 
// }

//=======================================================================================


//=======================================================================================
// Rangefinder 

// void VehicleHardware::RangefinderRead(void)
// {
//     // 
// }

//=======================================================================================
