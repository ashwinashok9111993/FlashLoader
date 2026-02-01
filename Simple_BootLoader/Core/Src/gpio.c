/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   GPIO initialization for UART Flash Loader
  * 
  * This bootloader only requires USART1 pins (PA9, PA10) which are
  * configured in HAL_UART_MspInit() in usart.c. No additional GPIO
  * configuration needed.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief GPIO Initialization (minimal, UART pins configured in usart.c)
  * 
  * UART TX (PA9) and RX (PA10) are configured in HAL_UART_MspInit().
  * No additional GPIO pins required for flash loader.
  *
  * @retval None
  */
void MX_GPIO_Init(void)
{
  /* GPIO Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* UART pins (PA9, PA10) are configured by HAL_UART_MspInit() in usart.c */
  /* No additional GPIO configuration needed for bootloader */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
