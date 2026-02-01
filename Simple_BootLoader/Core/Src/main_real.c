/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : UART Flash Loader for STM32F103C8
  * 
  * This bootloader provides firmware update capability over UART (115200 baud).
  * Communication is handled entirely by external Python/C# script.
  * 
  * Memory Layout:
  *   - Bootloader: 0x08000000 - 0x08001FFF (8 KB)
  *   - Application: 0x08002000 - 0x0800FFFF (56 KB)
  *
  * Protocol (XModem-like with CRC):
  *   Each frame: [SOH] [BlockNum] [~BlockNum] [128 bytes data] [CRC_H] [CRC_L]
  *   Control: [NAK] = error/retry, [ACK] = success, [CAN] = abort
  *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include <string.h>
#include <stdint.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Flash loader protocol definitions */
#define SOH                 0x01    /* Start of Header */
#define EOT                 0x04    /* End of Transmission */
#define ACK                 0x06    /* Acknowledge */
#define NAK                 0x15    /* Negative Acknowledge */
#define CAN                 0x18    /* Cancel */
#define XMODEM_FRAME_SIZE   128     /* XModem frame data size */

/* Memory layout for single application */
#define BOOT_BASE_ADDR      0x08000000U  /* Bootloader base address */
#define BOOT_SIZE           0x00002000U  /* 8 KB bootloader size */
#define APP_BASE_ADDR       0x08002000U  /* Application base address */
#define APP_MAX_SIZE        0x0000E000U  /* 56 KB max application size */

/* Flash and RAM limits */
#define FLASH_SIZE          0x00010000U  /* 64 KB total */
#define RAM_START_ADDR      0x20000000U
#define RAM_END_ADDR        0x20005000U  /* 20 KB RAM */

/* Frame constants */
#define MAX_RETRIES         3
#define TIMEOUT_MS          5000  /* Increased from 1000ms to 5000ms */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* UART handle from usart.c */
extern UART_HandleTypeDef huart1;

/* Flash write buffer */
static uint8_t flash_buffer[XMODEM_FRAME_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void uart_send_byte(uint8_t byte);
static uint8_t uart_recv_byte(uint32_t timeout_ms);
static uint8_t uart_recv_byte_safe(uint32_t timeout_ms, uint8_t *timeout_flag);
static uint16_t crc16_xmodem(const uint8_t *data, uint16_t len);
static int flash_write_page(uint32_t addr, const uint8_t *data, uint16_t size);
static void flash_loader_main(void);
static void jump_to_application(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Send single byte via UART
  * @param byte: byte to transmit
  * @retval None
  */
static void uart_send_byte(uint8_t byte)
{
  HAL_UART_Transmit(&huart1, &byte, 1, HAL_MAX_DELAY);
}

/**
  * @brief Receive single byte via UART with timeout
  * @param timeout_ms: timeout in milliseconds
  * @param timeout_flag: pointer to timeout flag (set to 1 if timeout, 0 if success)
  * @retval Received byte (valid only if timeout_flag is 0)
  */
static uint8_t uart_recv_byte_safe(uint32_t timeout_ms, uint8_t *timeout_flag)
{
  uint8_t byte = 0;
  *timeout_flag = 0;
  if (HAL_UART_Receive(&huart1, &byte, 1, timeout_ms) != HAL_OK)
  {
    *timeout_flag = 1;  /* Timeout occurred */
  }
  return byte;
}

/**
  * @brief Receive single byte via UART with timeout (legacy - returns 0xFF on timeout)
  * @param timeout_ms: timeout in milliseconds
  * @retval Received byte or 0xFF on timeout
  */
static uint8_t uart_recv_byte(uint32_t timeout_ms)
{
  uint8_t byte = 0;
  if (HAL_UART_Receive(&huart1, &byte, 1, timeout_ms) != HAL_OK)
  {
    return 0xFF;  /* Timeout indicator */
  }
  return byte;
}

/**
  * @brief Calculate CRC-16 XModem (poly 0x1021)
  * @param data: pointer to data buffer
  * @param len: length of data
  * @retval CRC-16 value
  */
static uint16_t crc16_xmodem(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0;
  uint16_t i;
  
  while (len--)
  {
    crc ^= (uint16_t)(*data++) << 8;
    for (i = 0; i < 8; i++)
    {
      crc <<= 1;
      if (crc & 0x10000)
      {
        crc ^= 0x1021;
      }
      crc &= 0xFFFF;  /* Ensure 16-bit width after each operation */
    }
  }
  return crc;
}

/**
  * @brief Flash one page (1024 bytes) of data
  * 
  * This function handles the STM32F1xx flash programming:
  *   - Unlocks flash if needed
  *   - Erases flash page containing the address
  *   - Programs data in half-word (16-bit) chunks
  *   - Locks flash after completion
  *
  * @param addr: flash address (must be page-aligned for erase)
  * @param data: pointer to data to write
  * @param size: number of bytes to write (must be 2-byte aligned)
  * @retval 0 on success, -1 on error
  */
static int flash_write_page(uint32_t addr, const uint8_t *data, uint16_t size)
{
  uint32_t page_error = 0;
  FLASH_EraseInitTypeDef erase_init;
  HAL_StatusTypeDef status;
  uint16_t i;

  /* Validate address range */
  if (addr < APP_BASE_ADDR || (addr + size) > (BOOT_BASE_ADDR + FLASH_SIZE))
  {
    return -1;
  }

  /* Validate size */
  if (size & 1)  /* Must be word-aligned */
  {
    return -1;
  }

  /* Unlock flash for programming */
  HAL_FLASH_Unlock();

  /* Setup erase operation for 1KB page */
  erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
  erase_init.PageAddress = addr;
  erase_init.NbPages = 1;

  /* Erase the page */
  status = HAL_FLASHEx_Erase(&erase_init, &page_error);
  if (status != HAL_OK)
  {
    HAL_FLASH_Lock();
    return -1;
  }

  /* Program data as 16-bit words */
  for (i = 0; i < size; i += 2)
  {
    uint16_t word = (uint16_t)data[i] | ((uint16_t)data[i + 1] << 8);
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr + i, word);
    if (status != HAL_OK)
    {
      HAL_FLASH_Lock();
      return -1;
    }
  }

  /* Lock flash after programming */
  HAL_FLASH_Lock();
  return 0;
}

/**
  * @brief Main flash loader routine
  * 
  * Implements XModem protocol over UART for binary firmware download:
  *   1. Waits for external script to initiate transfer (sends 'C' for CRC mode)
  *   2. Receives frames: [SOH] [BlockNum] [~BlockNum] [128 bytes] [CRC_H] [CRC_L]
  *   3. Validates frame and sends ACK or NAK
  *   4. Programs received data to flash in 1KB chunks
  *   5. On EOT (end of transmission), waits before auto-jumping to application
  *
  * Timeouts/retries handled by external script.
  * Bootloader stays in this function until successful update or CAN received.
  *
  * @retval None
  */
static void flash_loader_main(void)
{
  uint8_t byte;
  uint8_t block_num = 1;
  uint8_t last_block = 0;
  uint16_t crc, crc_rx;
  uint32_t app_addr = APP_BASE_ADDR;
  uint32_t total_bytes = 0;
  int i, status;

  /* Wait for host to initiate CRC-based XModem transfer */
  while (1)
  {
    byte = uart_recv_byte(TIMEOUT_MS);
    if (byte == 'C')  /* XModem CRC mode indicator */
    {
      break;
    }
    if (byte == 0xFF)
    {
      /* No activity, continue waiting */
      continue;
    }
  }

  /* Main transfer loop */
  for (;;)
  {
    byte = uart_recv_byte(TIMEOUT_MS);

    if (byte == SOH)
    {
      /* Standard 128-byte frame */
      uint8_t block_num_rx = uart_recv_byte(TIMEOUT_MS);
      uint8_t block_num_inv = uart_recv_byte(TIMEOUT_MS);

      /* Validate block number and its inverse */
      if ((block_num_rx ^ block_num_inv) != 0xFF)
      {
        uart_send_byte(NAK);
        continue;
      }

      /* Read 128 bytes of data */
      for (i = 0; i < XMODEM_FRAME_SIZE; i++)
      {
        uint8_t timeout_flag;
        flash_buffer[i] = uart_recv_byte_safe(TIMEOUT_MS, &timeout_flag);
        if (timeout_flag)
        {
          /* Timeout during frame reception */
          uart_send_byte(0xEE);  /* Debug: timeout indicator */
          uart_send_byte(NAK);
          goto next_frame;
        }
      }

      /* Receive and verify CRC */
      crc_rx = (uint16_t)uart_recv_byte(TIMEOUT_MS) << 8;
      crc_rx |= uart_recv_byte(TIMEOUT_MS);

      crc = crc16_xmodem(flash_buffer, XMODEM_FRAME_SIZE);
      
      if (crc != crc_rx)
      {
        /* CRC error */
        uart_send_byte(NAK);
        goto next_frame;
      }

      /* Check for duplicate block (resend request) */
      if (block_num_rx == last_block)
      {
        uart_send_byte(ACK);
        goto next_frame;
      }

      /* Validate block number sequence */
      if (block_num_rx != block_num)
      {
        uart_send_byte(NAK);
        goto next_frame;
      }

      /* Program data to flash */
      status = flash_write_page(app_addr, flash_buffer, XMODEM_FRAME_SIZE);
      if (status != 0)
      {
        uart_send_byte(NAK);
        goto next_frame;
      }

      /* Update tracking variables */
      last_block = block_num_rx;
      block_num = (block_num + 1) & 0xFF;
      app_addr += XMODEM_FRAME_SIZE;
      total_bytes += XMODEM_FRAME_SIZE;

      /* Limit application size */
      if (total_bytes > APP_MAX_SIZE)
      {
        uart_send_byte(CAN);
        return;
      }

      uart_send_byte(ACK);
    }
    else if (byte == EOT)
    {
      /* End of transmission received */
      uart_send_byte(ACK);
      /* Brief delay before jumping to application */
      for (volatile uint32_t delay = 0; delay < 1000000; delay++);
      return;
    }
    else if (byte == CAN)
    {
      /* Abort transfer */
      return;
    }

next_frame:
    ;
  }
}

/**
  * @brief Jump to application in flash
  * 
  * Performs clean handoff to application firmware:
  *   1. Validates application entry (stack pointer must point to RAM)
  *   2. Disables interrupts and peripheral clocks
  *   3. Deinitializes HAL peripherals (especially UART)
  *   4. Relocates NVIC vector table to application base
  *   5. Sets stack pointer and jumps to reset handler
  *
  * This function does not return.
  *
  * @retval None (does not return)
  */
static void jump_to_application(void)
{
  typedef void (*app_entry_t)(void);
  uint32_t app_sp;
  uint32_t app_reset;
  app_entry_t app_reset_handler;

  /* Read application vector table */
  app_sp = *(volatile uint32_t *)(APP_BASE_ADDR + 0);
  app_reset = *(volatile uint32_t *)(APP_BASE_ADDR + 4);

  /* Sanity check: stack pointer must be in RAM range */
  if (app_sp < RAM_START_ADDR || app_sp > RAM_END_ADDR)
  {
    /* Invalid application, stay in bootloader */
    Error_Handler();
  }

  /* Disable interrupts before context switch */
  __disable_irq();

  /* Stop SysTick timer */
  SysTick->CTRL = 0;

  /* Disable all NVIC interrupts and clear pending flags */
  for (uint32_t i = 0; i < 8; i++)
  {
    NVIC->ICER[i] = 0xFFFFFFFFU;
    NVIC->ICPR[i] = 0xFFFFFFFFU;
  }

  /* Deinitialize peripherals */
  HAL_UART_DeInit(&huart1);
  HAL_DeInit();

  /* Relocate NVIC vector table to application */
  SCB->VTOR = APP_BASE_ADDR;

  /* Set application stack pointer and jump to reset handler */
  __set_MSP(app_sp);
  app_reset_handler = (app_entry_t)app_reset;
  app_reset_handler();
  
  /* Never reached */
  while (1);
}

/**
  * @brief  Application entry point
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  
  /* Initialize HAL and system clock */
  HAL_Init();
  SystemClock_Config();

  /* Initialize UART (9600 baud for flash loader) */
  MX_USART1_UART_Init();
  
  /* Run flash loader protocol */
  flash_loader_main();

  /* If we reach here, jump to application in flash */
  jump_to_application();

  /* Should never reach this point */
  while (1)
  {
  }
  /* USER CODE END 1 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Error handler - halt bootloader
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Assert handler for debug
  * @param  file: source file name
  * @param  line: assert line number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
