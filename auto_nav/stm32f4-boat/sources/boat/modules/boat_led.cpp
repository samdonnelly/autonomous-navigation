/**
 * @file boat_led.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Boat LED module 
 * 
 * @version 0.1
 * @date 2024-04-05
 * 
 * @copyright Copyright (c) 2024
 * 
 */

//=======================================================================================
// Includes 

#include "boat.h" 

#include "ws2812_config.h" 

//=======================================================================================


//=======================================================================================
// LED module functions 

// Update the colour of the boat strobe light 
void Boat::LEDStrobeUpdate(uint32_t led_colour)
{
    // Update the strobe colour 
    xSemaphoreTake(comms_mutex, portMAX_DELAY); 
    leds.SetStrobeColour(led_colour); 
    xSemaphoreGive(comms_mutex); 
}


// Turn the boat strobe light off 
void Boat::LEDStrobeOff(void)
{
    // Set the strobe colour to off and queue an update event 
    LEDStrobeUpdate(ws2812_led_off); 
    CommsEventQueue((Event)CommsEvents::LED_STROBE_OFF); 
}


// Update the colour of the starbird and port LEDs 
void Boat::LEDUpdate(
    uint32_t starbird_led_colour, 
    uint32_t port_led_colour)
{
    // Update the port/starboard LED colours 
    xSemaphoreTake(comms_mutex, portMAX_DELAY); 
    leds.SetLEDsColour((SET_3 << WS2812_LED_0), starbird_led_colour); 
    leds.SetLEDsColour((SET_3 << WS2812_LED_6), port_led_colour); 
    xSemaphoreGive(comms_mutex); 
    CommsEventQueue((Event)CommsEvents::LED_WRITE); 
}

//=======================================================================================
