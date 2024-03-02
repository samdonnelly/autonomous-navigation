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
#define AB_NUM_COORDINATES 9         // Number of pre-defined GPS coordinates 
#define AB_TN_COR 130                // True North direction correction 
#define AB_WAYPOINT_RAD 100          // Threshold waypoint radius (meters*10) 
#define AB_GPS_INDEX_CNT 3           // Successive index command count needed to update 
#define AB_AUTO_BASE_SPEED 50        // Base throttle of each thruster (%) 
#define AB_AUTO_MAX_ERROR 600        // Max heading error (degrees*10) - must be within +/-1800 
#define AB_NAV_SCALAR 10             // Scalar used to remove decimal place in nav calcs 

// Calculations 
#define AB_EARTH_RADIUS 6371         // Earth average radius (km) 
#define AB_KM_TO_M 1000              // Kilometer to meter conversion 
#define AB_PI 3.14159                // pi 
#define AB_PI_OVER_2 1.57080         // pi/2 
#define AB_DEG_TO_RAD AB_PI/180      // Degrees to radians conversion 

// Manual Control 
#define AB_MC_LEFT_MOTOR 0x4C        // "L" character that indicates left motor 
#define AB_MC_RIGHT_MOTOR 0x52       // "R" character that indicates right motor 
#define AB_MC_FWD_THRUST 0x50        // "P" (plus) - indicates forward thrust 
#define AB_MC_REV_THRUST 0x4D        // "M" (minus) - indicates reverse thrust 
#define AB_MC_NEUTRAL 0x4E           // "N" (neutral) - indicates neutral gear or zero thrust 

// Thrusters 
#define AB_NO_THRUST 0               // Force thruster output to zero 

// Conditional compilation 
#define AB_GPS_LOC_FILTER 1          // GPS location low pass filter enable 
#define AB_GPS_RAD_FILTER 0          // GPS radius calculation low pass filter enable 
#define AB_GPS_HEAD_FILTER 0         // GPS heading calculation low pass filter enable 

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
