/**
 * @file ws2812_controller_mock.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief "ws2812_controller.cpp" mock functions 
 * 
 * @version 0.1
 * @date 2024-04-02
 * 
 * @copyright Copyright (c) 2024
 * 
 */

//=======================================================================================
// Includes 

#include "ws2812_controller.h" 

//=======================================================================================


//=======================================================================================
// Mock implementations 

// Constructor: Default 
WS2812_Controller::WS2812_Controller(device_number_t device_number)
{
    // 
}


// Constructor: Using LED strobe 
WS2812_Controller::WS2812_Controller(
    device_number_t device_number, 
    uint8_t strobe_led_mask, 
    uint8_t strobe_counter_period)
{
    // 
}


// Set strobe colour 
void WS2812_Controller::SetStrobeColour(uint32_t led_colour)
{
    // 
}


// Set LED to the specified colour 
void WS2812_Controller::SetLEDColour(
    ws2812_led_index_t led_num, 
    uint32_t led_colour)
{
    // 
}


// Set LEDs to the specified colour 
void WS2812_Controller::SetLEDsColour(
    uint8_t led_nums, 
    uint32_t led_colour)
{
    // 
}


// Strobe control 
void WS2812_Controller::Strobe(void)
{
    // 
}


// Turns strobe light off 
void WS2812_Controller::StrobeOff(void)
{
    // 
}


// Write the current values to the device 
void WS2812_Controller::LEDWrite(void)
{
    // 
}

//=======================================================================================
