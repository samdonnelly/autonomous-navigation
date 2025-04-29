/**
 * @file boat_setup.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Boat setup 
 * 
 * @version 0.1
 * @date 2024-01-31
 * 
 * @copyright Copyright (c) 2024
 * 
 */

//=======================================================================================
// Includes 

#include "boat.h" 

#include "esc_config.h" 
#include "lsm303agr_config.h" 
#include "m8q_config.h" 
#include "mpu6050_config.h" 
#include "nrf24l01_config.h" 
#include "ws2812_config.h" 

//=======================================================================================


//=======================================================================================
// Macros 

// Main thread memory 
#define MAIN_STACK_MULTIPLIER 8 
#define MAIN_STACK_SIZE configMINIMAL_STACK_SIZE * MAIN_STACK_MULTIPLIER 
#define MAIN_QUEUE_LEN 10 

// Communication thread memory 
#define COMMS_STACK_MULTIPLIER 8 
#define COMMS_STACK_SIZE configMINIMAL_STACK_SIZE * COMMS_STACK_MULTIPLIER 
#define COMMS_QUEUE_LEN 10 

// Software timers thread 
#define PERIODIC_TIMER_100MS_PERIOD 20   // Ticks 
#define PERIODIC_TIMER_1S_PERIOD 200     // Ticks 

//=======================================================================================


//=======================================================================================
// Variables 

// Boat definition 
Boat boat; 

//=======================================================================================


//=======================================================================================
// Boat initialization 

// Constructor 
Boat::Boat() 
    : main_state(MainStates::INIT_STATE), 
      main_event(MainEvents::NO_EVENT), 
      leds(DEVICE_ONE, ws2812_strobe_leds, ws2812_strobe_period), 
      radio(nrf24l01_pipe) 
{
    // State information 
    memset((void *)&main_flags, CLEAR, sizeof(main_flags)); 
    main_flags.state_entry = SET_BIT; 
    main_flags.init_state = SET_BIT; 

    // System data 
    memset((void *)adc_buff, CLEAR, sizeof(adc_buff)); 
}


// Setup 
void Boat::BoatSetup(void)
{
    //==================================================
    // General 

    // Initialize GPIO ports 
    gpio_port_init(); 

    // Initialize FreeRTOS scheduler 
    osKernelInitialize(); 

    //==================================================

    //==================================================
    // Timers 

    // General purpose 1us counter 
    tim_9_to_11_counter_init(
        TIM9, 
        TIM_84MHZ_1US_PSC, 
        0xFFFF,  // Max ARR value 
        TIM_UP_INT_DISABLE); 
    tim_enable(TIM9); 

    //==================================================

    //==================================================
    // Protocols 

    // I2C: Display, GNSS, IMU and magnetometer 
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
    
    // SPI: RF module 
    spi_init(
        SPI2, 
        GPIOB,   // GPIO port for SCK pin 
        PIN_10,  // SCK pin 
        GPIOC,   // GPIO port for data (MISO/MOSI) pins 
        PIN_2,   // MISO pin 
        PIN_3,   // MOSI pin 
        SPI_BR_FPCLK_16, 
        SPI_CLOCK_MODE_0); 
    
    // SPI: SD card 
    
    // UART: serial terminal 
    uart_init(
        USART2, 
        GPIOA, 
        PIN_3, 
        PIN_2, 
        UART_FRAC_42_9600, 
        UART_MANT_42_9600, 
        UART_DMA_DISABLE, 
        UART_DMA_DISABLE); 

    // UART: Bluetooth module 
    uart_init(
        USART1, 
        GPIOA, 
        PIN_10, 
        PIN_9, 
        UART_FRAC_84_115200, 
        UART_MANT_84_115200, 
        UART_DMA_DISABLE, 
        UART_DMA_DISABLE); 

    //==================================================

    //==================================================
    // ADC 

    // Main and auxiliary battery voltages and 5V PSU voltage 

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
        ADC_PARAM_DISABLE,     // ADC_CONT_DISABLE 
        ADC_PARAM_ENABLE,      // ADC_DMA_ENABLE 
        ADC_PARAM_ENABLE,      // ADC_DDS_ENABLE 
        ADC_PARAM_DISABLE);    // ADC_OVR_INT_DISABLE 

    // Initialize the ADC pins and channels 
    adc_pin_init(ADC1, GPIOA, PIN_6, ADC_CHANNEL_6, ADC_SMP_15);   // Main battery voltage 
    adc_pin_init(ADC1, GPIOA, PIN_7, ADC_CHANNEL_7, ADC_SMP_15);   // Auxiliary battery voltage 
    adc_pin_init(ADC1, GPIOA, PIN_4, ADC_CHANNEL_4, ADC_SMP_15);   // 5V PSU voltage 

    // Set the ADC conversion sequence 
    adc_seq(ADC1, ADC_CHANNEL_6, ADC_SEQ_1);   // Main battery voltage 
    adc_seq(ADC1, ADC_CHANNEL_7, ADC_SEQ_2);   // Auxiliary battery voltage 
    adc_seq(ADC1, ADC_CHANNEL_4, ADC_SEQ_3);   // 5V PSU voltage 

    // Set the sequence length (called once and only for more than one channel) 
    adc_seq_len_set(ADC1, (adc_seq_num_t)BOAT_ADC_BUFF_SIZE); 

    // Turn the ADC on 
    adc_on(ADC1); 

    //==================================================

    //==================================================
    // DMA 

    // Initialize the DMA stream for the ADC 
    dma_stream_init(
        DMA2, 
        DMA2_Stream0, 
        DMA_CHNL_0, 
        DMA_DIR_PM, 
        DMA_CM_ENABLE,
        DMA_PRIOR_VHI, 
        DMA_DBM_DISABLE, 
        DMA_ADDR_INCREMENT, 
        DMA_ADDR_FIXED, 
        DMA_DATA_SIZE_HALF, 
        DMA_DATA_SIZE_HALF); 

    // Configure the DMA stream for the ADC 
    dma_stream_config(
        DMA2_Stream0, 
        (uint32_t)(&ADC1->DR), 
        (uint32_t)adc_buff, 
        (uint32_t)NULL, 
        (uint16_t)BOAT_ADC_BUFF_SIZE); 

    // Enable the DMA stream for the ADC 
    dma_stream_enable(DMA2_Stream0); 

    //==================================================

    //==================================================
    // Interrupts 
    //==================================================

    //==================================================
    // Displays 

    // The HD44780U will interfere with the I2C bus before being configured so it must be 
    // set up before other devices on the same bus can be set up. 
    // The screen is physically integrated into the hardware setup but not used for this 
    // application so it gets initialized then set to low power mode. 
    hd44780u_init(I2C1, TIM9, PCF8574_ADDR_HHH); 
    hd44780u_clear(); 
    hd44780u_display_off(); 
    hd44780u_backlight_off(); 

    //==================================================

    //==================================================
    // GNSS 

    // SAM-M8Q module driver init 
    m8q_init(
        I2C1, 
        &m8q_config_msgs[0][0], 
        M8Q_CONFIG_MSG_NUM, 
        M8Q_CONFIG_MSG_MAX_LEN, 
        CLEAR); 

    // Set up low power and TX ready pins 
    m8q_pwr_pin_init(GPIOC, PIN_10); 
    m8q_txr_pin_init(GPIOC, PIN_11); 

    // SAM-M8Q module controller init 
    m8q_controller_init(TIM9); 

    //==================================================

    //==================================================
    // IMU 

    // The MPU-6050 is not current used so it gets set up and set to low power mode. 
    // MPU-6050 module driver init 
    mpu6050_init(
        DEVICE_ONE, 
        I2C1, 
        MPU6050_ADDR_1,
        mpu6050_standby_mask, 
        MPU6050_DLPF_CFG_1,
        mpu6050_sample_rate_divider,
        MPU6050_AFS_SEL_4,
        MPU6050_FS_SEL_500); 
    mpu6050_low_pwr_config(DEVICE_ONE, MPU6050_SLEEP_MODE_ENABLE); 

    //==================================================

    //==================================================
    // Magnetometer 

    // LSM303AGR module driver init 
    lsm303agr_m_init(
        I2C1, 
        lsm303agr_config_dir_offsets, 
        lsm303agr_lpf_gain, 
        LSM303AGR_M_ODR_10, 
        LSM303AGR_M_MODE_CONT, 
        LSM303AGR_CFG_DISABLE, 
        LSM303AGR_CFG_DISABLE, 
        LSM303AGR_CFG_DISABLE, 
        LSM303AGR_CFG_DISABLE); 
    
    //==================================================

    //==================================================
    // RF module 

    NRF24L01_STATUS nrf24l01_init_status = NRF24L01_OK; 

    // General setup common to all device - must be called once during setup 
    nrf24l01_init_status |= nrf24l01_init(
        SPI2,                       // SPI port to use 
        GPIOC,                      // Slave select pin GPIO port 
        PIN_1,                      // Slave select pin number 
        GPIOC,                      // Enable pin (CE) GPIO port 
        PIN_0,                      // Enable pin (CE) number 
        TIM9,                       // General purpose timer port 
        nrf24l01_rf_channel_freq,   // Initial RF channel frequency 
        NRF24L01_DR_2MBPS,          // Initial data rate to communicate at 
        NRF24L01_RF_PWR_0DBM);      // Initial power output to us 

    // Configure the PTX and PRX settings depending on the devices role/purpose. 
    nrf24l01_init_status |= nrf24l01_ptx_config(nrf24l01_pipe_addr); 
    nrf24l01_init_status |= nrf24l01_prx_config(nrf24l01_pipe_addr, nrf24l01_pipe); 

    // Power up the device now that it is configured 
    nrf24l01_init_status |= nrf24l01_pwr_up(); 

    // Check init status 
    if (nrf24l01_init_status)
    {
        uart_send_str(USART2, "nRF24L01 init failed."); 
        while(1); 
    }
    else 
    {
        uart_send_str(USART2, "nRF24L01 init success."); 
    }

    //==================================================

    //==================================================
    // SD Card 
    //==================================================

    //==================================================
    // Bluetooth 

    // The HC-05 is not currently used so it gets set up and set to low power mode. 
    // HC-05 module driver init 
    hc05_init(
        USART1, 
        TIM9, 
        GPIOA,     // AT pin GPIO 
        PIN_8,     // AT pin 
        GPIOA,     // EN pin GPIO 
        PIN_12,    // EN pin 
        GPIOA,     // STATE pin GPIO 
        PIN_11);   // STATE pin 
    hc05_off(); 

    //==================================================

    //==================================================
    // LEDs 

    // The timer port (not just the channel) used for the LEDs must be different than the 
    // timer used for the ESCs because they run at different speeds and the WS2812 driver 
    // turns the timer on and off for sending. 

    // WS2812 module driver init (Neopixel LEDs) - two LEDs run on one signal 
    ws2812_init(
        DEVICE_ONE, 
        TIM4, 
        TIMER_CH2, 
        GPIOB, 
        PIN_7); 
    
    //==================================================

    //==================================================
    // ESCs 

    // The timer port (not just the channel) used for the ESCs must be different than the 
    // timer used for the LEDs because they run at different speeds and the WS2812 driver 
    // turns the timer on and off for sending. 

    // ESC driver init - right thruster (ESC1) 
    esc_init(
        DEVICE_ONE, 
        TIM3, 
        TIMER_CH4, 
        GPIOB, 
        PIN_1, 
        TIM_84MHZ_1US_PSC, 
        esc_period, 
        esc1_fwd_speed_lim, 
        esc1_rev_speed_lim); 

    // ESC driver init - left thruster (ESC2) 
    esc_init(
        DEVICE_TWO, 
        TIM3, 
        TIMER_CH3, 
        GPIOB, 
        PIN_0, 
        TIM_84MHZ_1US_PSC, 
        esc_period, 
        esc2_fwd_speed_lim, 
        esc2_rev_speed_lim); 

    // Enable the PWM timer - TIM3 set up in the ESC init 
    tim_enable(TIM3); 

    //==================================================

    //==================================================
    // GPIO 
    //==================================================

    //==================================================
    // RTOS info 

    // Thread definition 
    main_event_info = 
    {
        .attr = { .name = "BoatMainThread", 
                  .attr_bits = CLEAR, 
                  .cb_mem = NULL, 
                  .cb_size = CLEAR, 
                  .stack_mem = NULL, 
                  .stack_size = MAIN_STACK_SIZE, 
                  .priority = (osPriority_t)osPriorityLow, 
                  .tz_module = CLEAR, 
                  .reserved = CLEAR }, 
        .event = CLEAR, 
        .ThreadEventQueue = xQueueCreate(MAIN_QUEUE_LEN, sizeof(uint32_t)), 
        .dispatch = BoatMainDispatch 
    }; 
    comms_event_info = 
    {
        .attr = { .name = "BoatCommsThread", 
                  .attr_bits = CLEAR, 
                  .cb_mem = NULL, 
                  .cb_size = CLEAR, 
                  .stack_mem = NULL, 
                  .stack_size = COMMS_STACK_SIZE, 
                  .priority = (osPriority_t)osPriorityNormal, 
                  .tz_module = CLEAR, 
                  .reserved = CLEAR }, 
        .event = CLEAR, 
        .ThreadEventQueue = xQueueCreate(COMMS_QUEUE_LEN, sizeof(uint32_t)), 
        .dispatch = BoatCommsDispatch 
    }; 

    // Create the thread(s) 
    osThreadNew(eventLoop, (void *)&main_event_info, &main_event_info.attr); 
    osThreadNew(eventLoop, (void *)&comms_event_info, &comms_event_info.attr); 
    // Check that the thread creation worked 

    // Software timers (within the software timers thread) 
    periodic_timer_100ms = xTimerCreate(
        "100ms",                        // Name of timer 
        PERIODIC_TIMER_100MS_PERIOD,    // Period of timer (ticks) 
        pdTRUE,                         // Auto-reload --> pdTRUE == Repeat Timer 
        (void *)0,                      // Timer ID 
        TimerCallback100ms);            // Callback function 
    periodic_timer_1s = xTimerCreate(
        "1s",                           // Name of timer 
        PERIODIC_TIMER_1S_PERIOD,       // Period of timer (ticks) 
        pdTRUE,                         // Auto-reload --> pdTRUE == Repeat Timer 
        (void *)1,                      // Timer ID 
        TimerCallback1s);               // Callback function 

    // Create mutex 
    comms_mutex = xSemaphoreCreateMutex(); 

    // Queue the first event to start the system 
    MainEventQueue((Event)MainEvents::INIT); 

    //==================================================
}

//=======================================================================================
