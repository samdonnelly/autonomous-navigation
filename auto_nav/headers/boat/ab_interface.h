/**
 * @file ab_interface.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Autonomous boat interface 
 * 
 * @version 0.1
 * @date 2024-03-02
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef _AB_INTERFACE_H_ 
#define _AB_INTERFACE_H_ 

#ifdef __cplusplus
extern "C" {
#endif

//=======================================================================================
// Includes 

#include "includes_drivers.h" 
#include "includes_cpp_drivers.h" 

#include "gps_coordinates.h" 

#include "led_control.h" 
#include "auto_mode.h" 

//=======================================================================================


//=======================================================================================
// Macros 

// Data sizes 
#define AB_ADC_BUFF_SIZE 3           // Number of ADCs used 
#define AB_PL_LEN 32                 // Payload length 
#define AB_MAX_CMD_SIZE 32           // Max external command size 

// Thrusters 
#define AB_NO_THRUST 0               // Force thruster output to zero 

//=======================================================================================


//=======================================================================================
// Enums 

/**
 * @brief System states 
 */
typedef enum {
    AB_INIT_STATE,         // State 0: startup 
    AB_NOT_READY_STATE,    // State 1: not ready 
    AB_READY_STATE,        // State 2: ready 
    AB_MANUAL_STATE,       // State 3: manual control mode 
    AB_AUTO_STATE,         // State 4: autonomous mode 
    AB_LOW_PWR_STATE,      // State 5: low power 
    AB_FAULT_STATE,        // State 6: fault 
    AB_RESET_STATE         // State 7: reset 
} ab_states_t; 

//=======================================================================================


//=======================================================================================
// Structures 

// Data record for the system 
class Boat : public boat_auto_mode, 
             public boat_led_control 
{
private: 
    // 

public: 
    // System information 
    ab_states_t state;                       // State machine state 
    ADC_TypeDef *adc;                        // ADC port battery soc and pots 
    nrf24l01_data_pipe_t pipe;               // Data pipe number for the radio module 
    uint16_t fault_code;                     // System fault code 

    // Timing information 
    TIM_TypeDef *timer_nonblocking;          // Timer used for non-blocking delays 
    tim_compare_t delay_timer;               // General purpose delay timing info 
    tim_compare_t hb_timer;                  // Heartbeat timing info 
    uint8_t hb_timeout;                      // Heartbeat timeout count 

    // System data 
    uint16_t adc_buff[AB_ADC_BUFF_SIZE];     // ADC buffer - battery and PSU voltage 

    // Payload data 
    uint8_t read_buff[AB_PL_LEN];            // Data read by PRX from PTX device 
    uint8_t cmd_id[AB_MAX_CMD_SIZE];         // Stores the ID of the external command 
    uint8_t cmd_value;                       // Stores the value of the external command 
    uint8_t hb_msg[AB_PL_LEN];               // Heartbeat message 

    // Thrusters 
    int16_t right_thruster;                  // Right thruster throttle 
    int16_t left_thruster;                   // Left thruster throttle 

    // Control flags 
    uint8_t connect     : 1;                 // Radio connection status flag 
    uint8_t mc_data     : 1;                 // Manual control new data check flag 
    uint8_t state_entry : 1;                 // State entry flag 
    uint8_t init        : 1;                 // Initialization state flag 
    uint8_t ready       : 1;                 // Ready state flag 
    uint8_t idle        : 1;                 // Idle flag - for leaving manual and auto modes 
    uint8_t manual      : 1;                 // Manual control mode state flag 
    uint8_t autonomous  : 1;                 // Autonomous mode state flag 
    uint8_t low_pwr     : 1;                 // Low power state flag 
    uint8_t fault       : 1;                 // Fault state flag 
    uint8_t reset       : 1;                 // Reset state flag 

public:   // Public member function 

    // Constructor 
    Boat(TIM_TypeDef *timer, ADC_TypeDef *adc_port) 
        : boat_auto_mode(timer), 
          boat_led_control(timer), 
          state(AB_INIT_STATE), 
          adc(adc_port), 
          pipe(NRF24L01_DP_1), 
          fault_code(CLEAR), 
          timer_nonblocking(timer), 
          hb_timeout(CLEAR), 
          cmd_value(CLEAR), 
          right_thruster(CLEAR), 
          left_thruster(CLEAR), 
          connect(CLEAR_BIT), 
          mc_data(CLEAR_BIT), 
          state_entry(SET_BIT), 
          init(SET_BIT), 
          ready(CLEAR_BIT), 
          idle(CLEAR_BIT), 
          manual(CLEAR_BIT), 
          autonomous(CLEAR_BIT), 
          low_pwr(CLEAR_BIT), 
          fault(CLEAR_BIT), 
          reset(CLEAR_BIT) 
    {
        // Timing 
        uint32_t clock_frequency = tim_get_pclk_freq(timer); 
        memset((void *)&delay_timer, CLEAR, sizeof(delay_timer)); 
        delay_timer.clk_freq = clock_frequency; 
        delay_timer.time_start = SET_BIT; 
        memset((void *)&hb_timer, CLEAR, sizeof(hb_timer)); 
        hb_timer.clk_freq = clock_frequency; 
        hb_timer.time_start = SET_BIT; 

        // System 
        memset((void *)adc_buff, CLEAR, sizeof(adc_buff)); 

        // Payload 
        memset((void *)read_buff, CLEAR, sizeof(read_buff)); 
        memset((void *)cmd_id, CLEAR, sizeof(cmd_id)); 
        memset((void *)hb_msg, CLEAR, sizeof(hb_msg)); 
    } 

    // Destructor 
    ~Boat() {} 
}; 


// Data record instance 
extern Boat ab_data; 

//=======================================================================================


//=======================================================================================
// Functions 

/**
 * @brief Autonomous boat initialization 
 * 
 * @details Initializes devices and peripherals used by the boat. Meant to be called 
 *          once at the start of the program. 
 */
void ab_init(void); 


/**
 * @brief Autonomous boat application 
 * 
 * @details Main control loop of the program that's called forever. 
 */
void ab_app(void); 

//=======================================================================================

#ifdef __cplusplus
}
#endif

#endif   // _AB_INTERFACE_H_ 
