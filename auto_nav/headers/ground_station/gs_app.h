/**
 * @file project_init.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Ground station application header 
 * 
 * @version 0.1
 * @date 2023-07-20
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef _GS_APP_H_ 
#define _GS_APP_H_ 

#ifdef __cplusplus
extern "C" {
#endif

//=======================================================================================
// Includes 

#include "gs_includes_app.h"
#include "includes_drivers.h"

//=======================================================================================


//=======================================================================================
// Macros 

// System info 
#define GS_NUM_STATES 3              // Total number of system states 

// Data sizes 
#define GS_ADC_BUFF_SIZE 2           // 

// User commands 
#define GS_MAX_USER_INPUT 30         // 
#define GS_NUM_CMDS 6                // Total number of external commands available 

// Heartbeat 
#define GS_HB_PERIOD 500000          // Time between heartbeat checks (us) 

// Manual Control 
#define GS_MC_LEFT_MOTOR 0x4C        // "L" character that indicates left motor 
#define GS_MC_RIGHT_MOTOR 0x52       // "R" character that indicates right motor 
#define GS_MC_FWD_THRUST 0x50        // "P" (plus) - indicates forward thrust 
#define GS_MC_REV_THRUST 0x4D        // "M" (minus) - indicates reverse thrust 
#define GS_MC_NEUTRAL 0x4E           // "N" (neutral) - indicates neutral gear or zero thrust 
#define GS_MC_PERIOD 50000           // Time between throttle command sends (us) 
#define GS_ADC_REV_LIM 100           // ADC value reverse command limit 
#define GS_ADC_FWD_LIM 155           // ADC value forward command limit 

// Command states 
#define GS_CMD_PERIOD 250000         // Time between command sends checks (us) 
#define GS_SEND_COUNT 10             // Number of times a command gets sent 

// Thrusters 
#define GS_NO_THRUST 0               // Force thruster output to zero 

//=======================================================================================


//=======================================================================================
// Enums 

/**
 * @brief Ground station states 
 * 
 * @details 
 */
typedef enum {
    GS_HB_STATE,             // State 0: heartbeat 
    GS_MC_STATE,             // State 1: manual control 
    GS_CMD_STATE,            // State 1: command send 
    // GS_IDLE_CMD_STATE,       // State 2: idle command 
    // GS_MANUAL_CMD_STATE,     // State 3: manual control command 
    // GS_AUTO_CMD_STATE,       // State 4: autonomous command 
    // GS_INDEX_CMD_STATE       // State 5: waypoint index update command 
} gs_states_t; 

//=======================================================================================


//=======================================================================================
// Structures 

// User commands 
typedef struct gs_cmds_s 
{
    char gs_cmd[GS_MAX_USER_INPUT]; 
    void (*gs_cmd_func_ptr)(uint8_t); 
    uint8_t gs_cmd_mask; 
}
gs_cmds_t; 


// Data record for the system 
typedef struct gs_data_s 
{
    // System information 
    gs_states_t state;                            // State machine state 

    // Timing information 
    TIM_TypeDef *timer_nonblocking;                // Timer used for non-blocking delays 
    tim_compare_t delay_timer;                     // Delay timing info 

    // User commands and payload data 
    uint8_t user_buff[GS_MAX_USER_INPUT];          // Circular buffer (CB) that stores user inputs 
    uint8_t buff_index;                            // CB index used for parsing commands 
    uint8_t cmd_buff[GS_MAX_USER_INPUT];           // Stores a user command parsed from the CB 
    uint8_t cmd_id[GS_MAX_USER_INPUT];             // Stores the ID of the user command 
    uint8_t cmd_value;                             // Stores the value of the user command 
    uint8_t hb_msg[NRF24L01_MAX_PAYLOAD_LEN];      // Heartbeat message 
    uint8_t write_buff[NRF24L01_MAX_PAYLOAD_LEN];  // Data sent to PRX from PTX device 

    // System data 
    uint16_t adc_buff[GS_ADC_BUFF_SIZE];           // ADC buffer - thruster potentiometers 

    // 
    uint8_t waypoint_index;                        // 

    // 
    uint8_t cmd_send_index;                        // 

    // Control flags 
    uint8_t led_state   : 1;                       // LED state (on/off) 
    uint8_t state_entry : 1;                       // State entry flag 
    uint8_t hb          : 1;                       // Heartbeat state flag 
    uint8_t mc          : 1;                       // Manual control mode state flag 
    uint8_t cmd         : 1;                       // Command send state flag 
    // uint8_t idle        : 1;                       // Idle flag - default to heartbeat state 
    // uint8_t manual      : 1;                       // Manual control command state flag 
    // uint8_t autonomous  : 1;                       // Autonomous command state flag 
    // uint8_t index       : 1;                       // Waypoint index update command state flag 
}
gs_data_t; 

//=======================================================================================


//=======================================================================================
// Function pointers 

/**
 * @brief State machine function pointer 
 */
typedef void (*gs_state_func_ptr)(void); 

//=======================================================================================


//=======================================================================================
// Functions 

/**
 * @brief Ground station application initializzation 
 * 
 * @details 
 * 
 * @param timer_nonblocking 
 * @param adc_dma_stream 
 * @param adc 
 * @param uart_dma_stream 
 * @param uart 
 */
void gs_app_init(
    TIM_TypeDef *timer_nonblocking, 
    DMA_Stream_TypeDef *adc_dma_stream, 
    ADC_TypeDef *adc, 
    DMA_Stream_TypeDef *uart_dma_stream, 
    USART_TypeDef *uart); 


/**
 * @brief Ground station application 
 * 
 * @details 
 */
void gs_app(void); 

//=======================================================================================

#ifdef __cplusplus
}
#endif

#endif   // _GS_APP_H_ 
