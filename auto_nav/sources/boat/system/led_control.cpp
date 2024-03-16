/**
 * @file led_control.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief AB LED control 
 * 
 * @version 0.1
 * @date 2024-03-14
 * 
 * @copyright Copyright (c) 2024
 * 
 */

//=======================================================================================
// Includes 

#include "led_control.h" 
#include "ws2812_config.h" 

//=======================================================================================


//=======================================================================================
// Macros 

// Timing 
#define AB_LED_PERIOD 100000         // LED update period 
#define AB_LED_TIMEOUT 30            // period*timeout = time between LED flashes 

//=======================================================================================


//=======================================================================================
// Setup and teardown 

// Constructor 
boat_led_control::boat_led_control(TIM_TypeDef *timer) 
    : timer_nonblocking(timer), 
      led_strobe(ws2812_led_off)
{
    // Timing 
    memset((void *)&led_timer, CLEAR, sizeof(led_timer)); 
    led_timer.clk_freq = tim_get_pclk_freq(timer_nonblocking); 
    led_timer.time_start = SET_BIT; 

    // LEDs 
    memset((void *)led_data, CLEAR, sizeof(led_data)); 
} 


// Destructor 
boat_led_control::~boat_led_control() {} 

//=======================================================================================


//=======================================================================================
// LED functions 

// Set strobe colour 
void boat_led_control::strobe_colour_set(uint32_t led_colour)
{
    led_strobe = led_colour; 
}


// Strobe control 
void boat_led_control::strobe(void)
{
    static uint8_t led_counter = CLEAR; 

    // Toggle the strobe LEDs to indicate the state to the user 
    if (tim_compare(timer_nonblocking, 
                    led_timer.clk_freq, 
                    AB_LED_PERIOD, 
                    &led_timer.time_cnt_total, 
                    &led_timer.time_cnt, 
                    &led_timer.time_start))
    {
        if (!led_counter)
        {
            led_data[WS2812_LED_3] = ws2812_led_off; 
            led_data[WS2812_LED_4] = ws2812_led_off; 
            ws2812_send(DEVICE_ONE, led_data); 
            led_counter++; 
        }
        else if (led_counter >= AB_LED_TIMEOUT)
        {
            led_data[WS2812_LED_3] = led_strobe; 
            led_data[WS2812_LED_4] = led_strobe; 
            ws2812_send(DEVICE_ONE, led_data); 
            led_counter = CLEAR; 
        }
        else 
        {
            led_counter++; 
        }
    }
}


// Strobe off 
void boat_led_control::strobe_off(void)
{
    led_strobe = ws2812_led_off; 
    led_data[WS2812_LED_3] = led_strobe; 
    led_data[WS2812_LED_4] = led_strobe; 
    ws2812_send(DEVICE_ONE, led_data); 
    led_timer.time_start = SET_BIT; 
}

//=======================================================================================
