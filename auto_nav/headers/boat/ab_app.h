/**
 * @file project_init.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Autonomous boat application header 
 * 
 * @version 0.1
 * @date 2023-07-20
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef _AB_APP_H_ 
#define _AB_APP_H_ 

#ifdef __cplusplus
extern "C" {
#endif

//=======================================================================================
// Includes 

#include "ab_includes_app.h"
#include "includes_drivers.h"

//=======================================================================================


//=======================================================================================
// Macros 

// System info 
#define AB_NUM_STATES 8              // Number of system states 
#define AB_NUM_CMDS 9                // Total number of external commands available 

// Timing 
#define AB_INIT_DELAY 1000000        // Init state delay (us) 
#define AB_NAV_UPDATE 100000         // Navigation calculation update period (us) 
#define AB_NAV_COUNTER 10            // update*counter = time between nav calc updates 
#define AB_HB_PERIOD 500000          // Time between heartbeat checks (us) 
#define AB_HB_TIMEOUT 30             // period*timeout = time before conection lost status 
#define AB_LED_PERIOD 100000         // LED update period 
#define AB_LED_TIMEOUT 30            // period*timeout = time between LED flashes 

// Data sizes 
#define AB_ADC_BUFF_SIZE 3           // Size according to the number of ADCs used 
#define AB_MAX_CMD_SIZE 32           // Max external command size 
#define AB_PL_LEN 32                 // Payload length 

// Navigation 
#define AB_NUM_COORDINATES 2         // Number of pre-defined GPS coordinates 
#define AB_WAYPOINT_RAD 100          // Threshold waypoint radius - expressed in meters*10 
#define AB_TN_COR 130                // True North direction correction 
#define AB_GPS_INDEX_CNT 3           // Successive index command count needed to update 
#define AB_AUTO_BASE_SPEED 50        // Base throttle of each thruster (%) 
#define AB_AUTO_MAX_ERROR 900        // Max heading error (degrees*10) - must be within +/-1800 

// Manual Control 
#define AB_MC_LEFT_MOTOR 0x4C        // "L" character that indicates left motor 
#define AB_MC_RIGHT_MOTOR 0x52       // "R" character that indicates right motor 
#define AB_MC_FWD_THRUST 0x50        // "P" (plus) - indicates forward thrust 
#define AB_MC_REV_THRUST 0x4D        // "M" (minus) - indicates reverse thrust 
#define AB_MC_NEUTRAL 0x4E           // "N" (neutral) - indicates neutral gear or zero thrust 

// Thrusters 
#define AB_NO_THRUST 0               // Force thruster output to zero 

//=======================================================================================


//=======================================================================================
// Enums 

/**
 * @brief 
 * 
 * @details 
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

// External commands 
typedef struct ab_cmds_s 
{
    char ab_cmd[AB_MAX_CMD_SIZE]; 
    void (*ab_cmd_func_ptr)(uint8_t); 
    uint8_t ab_cmd_mask; 
}
ab_cmds_t; 


// GPS coordinate data type 
typedef struct ab_waypoints_s 
{
    double lat;   // Latitude 
    double lon;   // Longitude 
}
ab_waypoints_t; 


// Data record for the system 
typedef struct ab_data_s 
{
    // System information 
    ab_states_t state;                       // State machine state 
    ADC_TypeDef *adc;                        // ADC port battery soc and pots 
    nrf24l01_data_pipe_t pipe;               // Data pipe number for the radio module 
    uint16_t fault_code;                     // System fault code 

    // Timing information 
    TIM_TypeDef *timer_nonblocking;          // Timer used for non-blocking delays 
    tim_compare_t delay_timer;               // General purpose delay timing info 
    tim_compare_t nav_timer;                 // Navigation timing info 
    tim_compare_t led_timer;                 // LED output timing info 
    tim_compare_t hb_timer;                  // Heartbeat timing info 
    uint8_t hb_timeout;                      // Heartbeat timeout count 

    // System data 
    uint16_t adc_buff[AB_ADC_BUFF_SIZE];     // ADC buffer - battery and PSU voltage 
    uint32_t led_data[WS2812_LED_NUM];       // Bits: Green: 16-23, Red: 8-15, Blue: 0-7 
    uint32_t led_strobe;                     // LED strobe colour 

    // Payload data 
    uint8_t read_buff[AB_PL_LEN];            // Data read by PRX from PTX device 
    uint8_t cmd_id[AB_MAX_CMD_SIZE];         // Stores the ID of the external command 
    uint8_t cmd_value;                       // Stores the value of the external command 
    uint8_t hb_msg[AB_PL_LEN];               // Heartbeat message 

    // Navigation data 
    uint8_t waypoint_index;                  // GPS coordinate index 
    ab_waypoints_t waypoint;                 // Target waypoint 
    ab_waypoints_t location;                 // Current geographical location 
    uint16_t waypoint_rad;                   // Distance between current and target location 
    int16_t heading_desired;                 // Desired heading 
    int16_t heading_actual;                  // Current heading 
    int16_t heading_error;                   // Heading error 

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
}
ab_data_t; 

//=======================================================================================


//=======================================================================================
// Function pointers 

/**
 * @brief State machine function pointer 
 */
typedef void (*ab_state_func_ptr_t)(void); 

//=======================================================================================


//=======================================================================================
// Functions 

/**
 * @brief 
 * 
 * @details 
 * 
 * @param adc_dma_stream : 
 * @param adc : 
 * @param timer_nonblocking : 
 * @param pipe_num : 
 */
void ab_app_init(
    TIM_TypeDef *timer_nonblocking, 
    DMA_Stream_TypeDef *adc_dma_stream, 
    ADC_TypeDef *adc, 
    nrf24l01_data_pipe_t pipe_num); 


/**
 * @brief Autonomous boat application 
 * 
 * @details 
 */
void ab_app(void); 

//=======================================================================================

#ifdef __cplusplus
}
#endif

#endif   // _AB_APP_H_ 
