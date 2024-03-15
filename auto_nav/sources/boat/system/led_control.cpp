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
#include "ab_interface.h" 
#include "ws2812_config.h" 

//=======================================================================================


//=======================================================================================
// Macros 

// Timing 
#define AB_LED_PERIOD 100000         // LED update period 
#define AB_LED_TIMEOUT 30            // period*timeout = time between LED flashes 

//=======================================================================================


//=======================================================================================
// LED functions 

// // LED colour update 


// // LED on/off (not a strobe) 


// // LED strobe control 
// void ab_led_strobe(void)
// {
//     static uint8_t led_counter = CLEAR; 

//     // Toggle the strobe LEDs to indicate the state to the user 
//     if (tim_compare(ab_data.timer_nonblocking, 
//                     ab_data.led_timer.clk_freq, 
//                     AB_LED_PERIOD, 
//                     &ab_data.led_timer.time_cnt_total, 
//                     &ab_data.led_timer.time_cnt, 
//                     &ab_data.led_timer.time_start))
//     {
//         if (!led_counter)
//         {
//             ab_data.led_data[WS2812_LED_3] = ws2812_led_off; 
//             ab_data.led_data[WS2812_LED_4] = ws2812_led_off; 
//             ws2812_send(DEVICE_ONE, ab_data.led_data); 
//             led_counter++; 
//         }
//         else if (led_counter >= AB_LED_TIMEOUT)
//         {
//             ab_data.led_data[WS2812_LED_3] = ab_data.led_strobe; 
//             ab_data.led_data[WS2812_LED_4] = ab_data.led_strobe; 
//             ws2812_send(DEVICE_ONE, ab_data.led_data); 
//             led_counter = CLEAR; 
//         }
//         else 
//         {
//             led_counter++; 
//         }
//     }
// }


// // LED strobe off 
// void ab_led_strobe_off(void)
// {
//     ab_data.led_strobe = ws2812_led_off; 
//     ab_data.led_data[WS2812_LED_3] = ab_data.led_strobe; 
//     ab_data.led_data[WS2812_LED_4] = ab_data.led_strobe; 
//     ws2812_send(DEVICE_ONE, ab_data.led_data); 
//     ab_data.led_timer.time_start = SET_BIT; 
// }

//=======================================================================================
