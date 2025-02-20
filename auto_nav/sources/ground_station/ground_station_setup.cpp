/**
 * @file ground_station_setup.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Ground station setup 
 * 
 * @version 0.1
 * @date 2023-07-20
 * 
 * @copyright Copyright (c) 2023
 * 
 */

//=======================================================================================
// Includes 

#include "ground_station.h" 
#include "stm32f4xx_it.h" 

//=======================================================================================


//=======================================================================================
// Variables 

// Ground station definition 
GroundStation ground_station; 

//=======================================================================================


//=======================================================================================
// Ground Station Initialization 

// Constructor 
GroundStation::GroundStation() 
    : status_message(nullptr), 
      cmd_value(CLEAR), 
      hb_timeout_counter(CLEAR) 
{
    memset((void *)ui_buff, CLEAR, sizeof(ui_buff)); 
    memset((void *)cb, CLEAR, sizeof(cb)); 
    memset((void *)cmd_buff, CLEAR, sizeof(cmd_buff)); 
    memset((void *)cmd_id, CLEAR, sizeof(cmd_id)); 
    memset((void *)cmd_str, CLEAR, sizeof(cmd_str)); 
    memset((void *)adc_buff, CLEAR, sizeof(adc_buff)); 
    memset((void *)read_buff, CLEAR, sizeof(read_buff)); 
    memset((void *)write_buff, CLEAR, sizeof(write_buff)); 

    gs_flags.user_cmd_flag = CLEAR_BIT; 
    gs_flags.manual_control_flag = CLEAR_BIT; 
    gs_flags.radio_connection_flag = CLEAR_BIT; 

    // Data not configured here is handled in the setup function 
} 


void GroundStation::GroundStationSetup(void)
{
    // System data 
    uart = USART2; 
    dma_stream = DMA1_Stream5; 
    timer_nonblocking = TIM9; 

    cb_index.cb_size = GS_MAX_CMD_LEN; 
    cb_index.head = CLEAR; 
    cb_index.tail = CLEAR; 

    dma_index.data_size = CLEAR; 
    dma_index.ndt_old = GS_MAX_CMD_LEN; 
    dma_index.ndt_new = CLEAR; 

    //==================================================
    // General setup 

    // Initialize GPIO ports 
    gpio_port_init(); 

    // General purpose timer 
    tim_9_to_11_counter_init(
        timer_nonblocking, 
        TIM_84MHZ_1US_PSC, 
        0xFFFF,  // Max ARR value 
        TIM_UP_INT_DISABLE); 
    tim_enable(timer_nonblocking); 

    // Configure the board LED to blink when transmitting. 
    gpio_pin_init(GPIOA, PIN_5, MODER_GPO, OTYPER_PP, OSPEEDR_HIGH, PUPDR_NO); 
    gpio_write(GPIOA, GPIOX_PIN_5, GPIO_LOW); 

    //==================================================

    //==================================================
    // UART 

    // Initialize UART 
    uart_init(
        uart, 
        GPIOA, 
        PIN_3, 
        PIN_2, 
        UART_FRAC_42_9600, 
        UART_MANT_42_9600, 
        UART_DMA_DISABLE, 
        UART_DMA_ENABLE);   // DMA enabled so it can be configured later 
    
    //==================================================

    //==================================================
    // SPI 

    // SPI for the RF module 
    spi_init(
        SPI2, 
        GPIOB,   // GPIO port for SCK pin 
        PIN_10,  // SCK pin 
        GPIOC,   // GPIO port for data (MISO/MOSI) pins 
        PIN_2,   // MISO pin 
        PIN_3,   // MOSI pin 
        SPI_BR_FPCLK_16, 
        SPI_CLOCK_MODE_0); 
    
    //==================================================

    //==================================================
    // ADC 

    // Initialize the ADC port 
    adc1_clock_enable(RCC); 
    adc_port_init(
        ADC1, 
        ADC1_COMMON, 
        ADC_PCLK2_4, 
        ADC_RES_8, 
        ADC_PARAM_ENABLE,      // ADC_EOC_EACH 
        ADC_PARAM_DISABLE,     // ADC_EOC_INT_DISABLE 
        ADC_PARAM_ENABLE,      // ADC_SCAN_ENABLE 
        ADC_PARAM_ENABLE,      // ADC_CONT_ENABLE 
        ADC_PARAM_ENABLE,      // ADC_DMA_ENABLE 
        ADC_PARAM_ENABLE,      // ADC_DDS_ENABLE 
        ADC_PARAM_DISABLE);    // ADC_OVR_INT_DISABLE 

    // Initialize the ADC pins and channels 
    // Right and left thruster control (potentiometer) in manual control mode 
    adc_pin_init(ADC1, GPIOA, PIN_6, ADC_CHANNEL_6, ADC_SMP_15); 
    adc_pin_init(ADC1, GPIOA, PIN_7, ADC_CHANNEL_7, ADC_SMP_15); 

    // Set the ADC conversion sequence 
    adc_seq(ADC1, ADC_CHANNEL_6, ADC_SEQ_1); 
    adc_seq(ADC1, ADC_CHANNEL_7, ADC_SEQ_2); 

    // Set the sequence length 
    adc_seq_len_set(ADC1, ADC_SEQ_2); 

    // Turn the ADC on 
    adc_on(ADC1); 

    //==================================================

    //==================================================
    // DMA & interrupts 

    // UART 
    // Enable the IDLE line interrupt 
    uart_interrupt_init(
        uart, 
        UART_INT_DISABLE, 
        UART_INT_DISABLE, 
        UART_INT_DISABLE, 
        UART_INT_DISABLE, 
        UART_INT_ENABLE, 
        UART_INT_DISABLE, 
        UART_INT_DISABLE); 
    // Initialize the DMA stream 
    dma_stream_init(
        DMA1, 
        dma_stream, 
        DMA_CHNL_4, 
        DMA_DIR_PM, 
        DMA_CM_ENABLE,
        DMA_PRIOR_VHI, 
        DMA_DBM_DISABLE, 
        DMA_ADDR_INCREMENT,   // Increment the buffer pointer to fill the buffer 
        DMA_ADDR_FIXED,       // No peripheral increment - copy from DR only 
        DMA_DATA_SIZE_BYTE, 
        DMA_DATA_SIZE_BYTE); 
    // Configure the DMA stream for the UART 
    dma_stream_config(
        dma_stream, 
        (uint32_t)(&uart->DR), 
        (uint32_t)cb, 
        (uint32_t)NULL, 
        (uint16_t)GS_MAX_CMD_LEN); 

    // Enable the DMA stream for the UART 
    dma_stream_enable(dma_stream); 

    // Initialize interrupt handler flags (called once) 
    int_handler_init(); 

    // Enable the interrupt handlers (called for each interrupt) - for USART2_RX 
    nvic_config(USART2_IRQn, EXTI_PRIORITY_0); 


    // ADC 
    // Initialize the DMA stream for the ADC 
    dma_stream_init(
        DMA2, 
        DMA2_Stream0, 
        DMA_CHNL_0, 
        DMA_DIR_PM, 
        DMA_CM_ENABLE,
        DMA_PRIOR_VHI, 
        DMA_DBM_DISABLE, 
        DMA_ADDR_INCREMENT,   // Increment the buffer pointer to fill the buffer 
        DMA_ADDR_FIXED,       // No peripheral increment - copy from DR only 
        DMA_DATA_SIZE_HALF, 
        DMA_DATA_SIZE_HALF); 
    // Configure the DMA stream 
    dma_stream_config(
        DMA2_Stream0, 
        (uint32_t)(&ADC1->DR), 
        (uint32_t)adc_buff, 
        (uint32_t)NULL, 
        (uint16_t)GS_ADC_BUFF_SIZE); 

    // Enable the DMA stream for the ADC 
    dma_stream_enable(DMA2_Stream0); 

    // Start the ADC conversions (continuous mode) 
    adc_start(ADC1); 

    //==================================================

    //==================================================
    // RF module 

    // General setup common to all devices - must be called once during setup 
    nrf24l01_init(
        SPI2,                       // SPI port to use 
        GPIOC,                      // Slave select pin GPIO port 
        PIN_1,                      // Slave select pin number 
        GPIOC,                      // Enable pin (CE) GPIO port 
        PIN_0,                      // Enable pin (CE) number 
        TIM9,                       // General purpose timer port 
        nrf24l01_rf_channel_freq,   // Initial RF channel frequency 
        NRF24L01_DR_2MBPS,          // Initial data rate to communicate at 
        NRF24L01_RF_PWR_0DBM);      // Initial power output to use 

    // Configure the PTX and PRX settings depending on the devices role/purpose. 
    nrf24l01_ptx_config(nrf24l01_pipe_addr); 
    nrf24l01_prx_config(nrf24l01_pipe_addr, nrf24l01_pipe); 

    // Power up the device now that it is configured 
    nrf24l01_pwr_up(); 

    //==================================================

    //==================================================
    // Data 

    // Timing 
    delay_timer.clk_freq = tim_get_pclk_freq(timer_nonblocking); 
    delay_timer.time_cnt_total = CLEAR; 
    delay_timer.time_cnt = CLEAR; 
    delay_timer.time_start = SET_BIT; 

    // User interface 
    InitializeUI(); 

    // Enable all commands 
    for (uint8_t i = CLEAR; i < GS_NUM_CMDS; i++)
    {
        CommandEnable(command_table[i].cmd, command_table, SET_BIT); 
    }

    //==================================================
}

//=======================================================================================
