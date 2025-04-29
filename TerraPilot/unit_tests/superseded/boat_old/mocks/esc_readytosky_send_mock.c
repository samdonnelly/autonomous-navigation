/**
 * @file esc_send_mock.c
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief ESC driver mock 
 * 
 * @version 0.1
 * @date 2024-06-20
 * 
 * @copyright Copyright (c) 2024
 * 
 */

//=======================================================================================
// Includes 

#include "esc_driver.h" 

//=======================================================================================


//=======================================================================================
// Driver functions 

// ESC initialization 
void esc_init(
    device_number_t device_num, 
    TIM_TypeDef *timer, 
    tim_channel_t tim_channel, 
    GPIO_TypeDef *gpio, 
    pin_selector_t pin, 
    timer_us_prescalars_t prescalar, 
    uint16_t arr, 
    uint16_t fwd_speed_lim, 
    uint16_t rev_speed_lim)
{
    // 
}


// ESC PWM command send 
void esc_send(
    device_number_t device_num, 
    int16_t throttle_cmd)
{
    // 
}

//=======================================================================================


//=======================================================================================
// Mock functions 
//=======================================================================================
