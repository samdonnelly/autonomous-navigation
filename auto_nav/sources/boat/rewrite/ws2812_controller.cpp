/**
 * @file ws2812_controller.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief WS2812 (Neopixel) controller 
 * 
 * @version 0.1
 * @date 2024-04-01
 * 
 * @copyright Copyright (c) 2024
 * 
 */

//=======================================================================================
// Includes 

#include "ws2812_controller.h" 

//=======================================================================================


//=======================================================================================
// Setup 

// Constructor: Default 
WS2812_Controller::WS2812_Controller(device_number_t device_number)
    : device_num(device_number), 
      strobe_mask(CLEAR), 
      strobe_counter(CLEAR), 
      strobe_period(CLEAR), 
      strobe_colour(CLEAR) 
{
    memset((void *)led_colours, CLEAR, sizeof(led_colours)); 
}

// Constructor: Using LED strobe 
WS2812_Controller::WS2812_Controller(
    device_number_t device_number, 
    uint8_t strobe_led_mask, 
    uint8_t strobe_counter_period) 
    : device_num(device_number), 
      strobe_mask(strobe_led_mask), 
      strobe_counter(CLEAR), 
      strobe_period(strobe_counter_period), 
      strobe_colour(CLEAR)  
{
    memset((void *)led_colours, CLEAR, sizeof(led_colours)); 
} 

//=======================================================================================


//=======================================================================================
// On/off user functions 

// Set LED to the specified colour 
void WS2812_Controller::SetLEDColour(
    ws2812_led_index_t led_num, 
    uint32_t led_colour)
{
    LEDColourUpdate(led_colour, (SET_BIT << led_num) & ~strobe_mask); 
}

// Set LEDs to the specified colour 
void WS2812_Controller::SetLEDsColour(
    uint8_t led_nums, 
    uint32_t led_colour)
{
    LEDColourUpdate(led_colour, led_nums & ~strobe_mask); 
}

//=======================================================================================


//=======================================================================================
// Strobe user functions 

// Set strobe colour 
void WS2812_Controller::SetStrobeColour(uint32_t led_colour)
{
    strobe_colour = led_colour; 
}

// Strobe control 
void WS2812_Controller::Strobe(void)
{
    // Toggle the strobe LEDs 
    if (strobe_counter >= strobe_period)
    {
        StrobeColourUpdate(strobe_colour); 
        strobe_counter = CLEAR; 
    }
    else 
    {
        if (!strobe_counter)
        {
            StrobeOff(); 
        }
        strobe_counter++; 
    }
}

// Turns strobe light off 
void WS2812_Controller::StrobeOff(void)
{
    StrobeColourUpdate(CLEAR); 
}

//=======================================================================================


//=======================================================================================
// Helper functions 

// Update strobe LEDs to the specified colour 
void WS2812_Controller::StrobeColourUpdate(uint32_t led_colour)
{
    LEDColourUpdate(led_colour, strobe_mask); 
}

// Update specified LEDs to the specified colour 
void WS2812_Controller::LEDColourUpdate(
    uint32_t led_colour, 
    uint8_t mask)
{
    for (uint8_t i = CLEAR; i < WS2812_LED_NUM; i++)
    {
        if ((SET_BIT << i) & mask)
        {
            led_colours[i] = led_colour; 
        }
    }

    ws2812_send(device_num, led_colours); 
}

//=======================================================================================
