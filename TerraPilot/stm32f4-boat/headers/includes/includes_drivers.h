/**
 * @file includes_drivers.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Includes file for the device drivers 
 * 
 * @version 0.1
 * @date 2022-08-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef _INCLUDES_DRIVERS_H_
#define _INCLUDES_DRIVERS_H_

#ifdef __cplusplus
extern "C" {
#endif

//=======================================================================================
// Includes 

// Device drivers 
#include "esc_driver.h" 
#include "hc05_driver.h" 
#include "hd44780u_driver.h" 
#include "hw125_driver.h" 
#include "lsm303agr_driver.h" 
#include "m8q_driver.h" 
#include "mpu6050_driver.h" 
#include "nrf24l01_driver.h" 
#include "sik_radio_driver.h" 
#include "ws2812_driver.h" 

// Controllers 
#include "hc05_controller.h" 
#include "hd44780u_controller.h" 
#include "hw125_controller.h" 
#include "m8q_controller.h" 
#include "mpu6050_controller.h" 

// Peripheral drivers 
#include "analog_driver.h" 
#include "dma_driver.h" 
#include "gpio_driver.h" 
#include "interrupt_driver.h" 
#include "i2c_comm.h" 
#include "ibus.h" 
#include "spi_comm.h" 
#include "timers_driver.h" 
#include "uart_comm.h" 

// Tools 
#include "linked_list_driver.h" 
#include "stm32f411xe_custom.h" 
#include "switch_debounce.h" 
#include "tools.h" 

// STM drivers 
#include "fatfs.h" 

//=======================================================================================

#ifdef __cplusplus
}
#endif

#endif  // _INCLUDES_DRIVERS_H_
