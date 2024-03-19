/**
 * @file led_control.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief AB LED control interface 
 * 
 * @version 0.1
 * @date 2024-03-14
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef _AB_LED_CONTROL_H_ 
#define _AB_LED_CONTROL_H_ 

#ifdef __cplusplus
extern "C" {
#endif

//=======================================================================================
// Includes 

#include "includes_drivers.h" 

//=======================================================================================


//=======================================================================================
// Classes 

class boat_led_control 
{
private:   // Private members 

    // Timing 
    TIM_TypeDef *timer_nonblocking;          // Timer used for non-blocking delays 

    // LEDs 
    uint32_t led_data[WS2812_LED_NUM];       // Bits: Green: 16-23, Red: 8-15, Blue: 0-7 
    uint32_t led_strobe;                     // LED strobe colour 

public:   // Public members 

    // Timing 
    tim_compare_t led_timer;                 // LED output timing info  

public:   // Public member functions 
    
    // Constructor 
    boat_led_control(TIM_TypeDef *timer); 

    // Destructor 
    ~boat_led_control(); 

    /**
     * @brief Set strobe colour 
     * 
     * @param led_colour : strobe colour 
     */
    void strobe_colour_set(uint32_t led_colour); 

    /**
     * @brief Strobe control 
     * 
     * @details Periodically flashes the boat LEDs in a certain colour. LED colour is set 
     *          cased on the boats state. This is used as a visual indicator of the boats 
     *          state and to make the boat visible to surrounding entities. This should 
     *          be called repeatedly to keep the strobe going. 
     */
    void strobe(void); 

    /**
     * @brief Turns strobe light off 
     */
    void strobe_off(void); 
}; 

//=======================================================================================

#ifdef __cplusplus
}
#endif

#endif   // _AB_LED_CONTROL_H_ 
