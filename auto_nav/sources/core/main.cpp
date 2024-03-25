/**
 * @file main.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Main program body 
 * 
 * @version 0.1
 * @date 2024-03-03
 * 
 * @copyright Copyright (c) 2024
 * 
 */

//=======================================================================================
// Includes 

// Configuration 
#include "system_settings.h" 

// Library 
#include "stm32f4xx_hal.h" 
#include "fatfs.h"

// Application 
#include "gs_interface.h" 
#include "ab_interface.h" 

// FreeRTOS 
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include "queue.h" 
#include "semphr.h" 
#include "timers.h" 

//=======================================================================================


//=======================================================================================
// Macros 

// Ports and pins 
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

//=======================================================================================


//=======================================================================================
// Prototypes 

/**
 * @brief System Clock Configuration
 */
void SystemClock_Config(void); 


/**
 * @brief GPIO Initialization Function 
 */
static void MX_GPIO_Init(void); 


/**
 * @brief This function is executed in case of error occurrence 
 */
void Error_Handler(void); 

//=======================================================================================


//=======================================================================================
// Main 

// The application entry point 
int main(void)
{
    // The order of the below function calls is important. Some stuff needs to be set up 
    // before other stuff can be set up. 

    // Reset of all peripherals, initialize the Flash interface, then initialize and set 
    // the time base source. 
    HAL_Init(); 

    // Configure the system clock 
    SystemClock_Config(); 

    // Run application setup code
#if GROUND_STATION 
    gs_init(); 
#elif BOAT 
    // Init 
    // Initialize FreeRTOS scheduler - add this to the init function 
    // osKernelInitialize(); 

    boat.BoatInit(); 
#endif 

    //  Initialize all configured peripherals 
    MX_GPIO_Init(); 
    MX_FATFS_Init(); 

    // Main loop 
    while (1)
    {
#if GROUND_STATION 
        gs_app(); 
#elif BOAT 
        // Start scheduler. 
        // osKernelStart(); 

        boat.BoatApp(); 
#endif 
    }
}

//=======================================================================================


//=======================================================================================
// Functions 

// System Clock Configuration 
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct; 
    RCC_ClkInitTypeDef RCC_ClkInitStruct; 

    memset((void *)&RCC_OscInitStruct, CLEAR, sizeof(RCC_OscInitTypeDef)); 
    memset((void *)&RCC_ClkInitStruct, CLEAR, sizeof(RCC_ClkInitTypeDef)); 

    // Configure the main internal regulator output voltage 
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    // Initializes the RCC Oscillators according to the specified parameters 
    // in the RCC_OscInitTypeDef structure. 
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI; 
    RCC_OscInitStruct.HSIState = RCC_HSI_ON; 
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT; 
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON; 
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI; 
    RCC_OscInitStruct.PLL.PLLM = 16; 
    RCC_OscInitStruct.PLL.PLLN = 336; 
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4; 
    RCC_OscInitStruct.PLL.PLLQ = 4; 

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    // Initializes the CPU, AHB and APB buses clocks
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | 
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2; 
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; 
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; 
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2; 
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1; 

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

// GPIO Initialization Function 
static void MX_GPIO_Init(void)
{
    // GPIO_InitTypeDef GPIO_InitStruct = {0}; 
    GPIO_InitTypeDef GPIO_InitStruct; 
    
    memset((void *)&GPIO_InitStruct, CLEAR, sizeof(GPIO_InitTypeDef)); 

    // GPIO Ports Clock Enable 
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // Configure GPIO pin Output Level 
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

    // Configure GPIO pin : B1_Pin 
    GPIO_InitStruct.Pin = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    // Configure GPIO pin : LD2_Pin 
    GPIO_InitStruct.Pin = LD2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}


// This function is executed in case of error occurrence 
void Error_Handler(void)
{
    // User can add his own implementation to report the HAL error return state 
    __disable_irq();
    while (1) {}
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
      ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}

#endif /* USE_FULL_ASSERT */

//=======================================================================================
