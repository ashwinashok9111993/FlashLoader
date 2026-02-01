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
 * Protocol (Handshake-driven, 8-byte chunks):
 *   1. Bootloader sends "BOOT\r\n" every 500ms, waits 5 seconds for START
 *   2. If no START command within 5s, auto-jump to application
 *   3. Host sends 'S' to start transfer
 *   4. Per chunk: Host sends 'D' + 8 bytes + checksum, waits for ACK
 *   5. Bootloader writes 1KB page when buffer full
 *   6. Host sends 'E' to end transfer, bootloader jumps to application
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
#define CMD_START           'S'     /* Start transfer command */
#define CMD_DATA            'D'     /* Data packet command */
#define CMD_END             'E'     /* End transfer command */
#define ACK                 0x06    /* Acknowledge */
#define NAK                 0x15    /* Negative Acknowledge */
#define CHUNK_SIZE          8       /* 8-byte chunks */
#define PAGE_SIZE           1024    /* 1KB page size */

/* Memory layout for single application */
#define BOOT_BASE_ADDR      0x08000000U  /* Bootloader base address */
#define BOOT_SIZE           0x00002000U  /* 8 KB bootloader size */
#define APP_BASE_ADDR       0x08002000U  /* Application base address */
#define APP_MAX_SIZE        0x0000E000U  /* 56 KB max application size */

/* Flash and RAM limits */
#define FLASH_SIZE          0x00010000U  /* 64 KB total */
#define RAM_START_ADDR      0x20000000U
#define RAM_END_ADDR        0x20005000U  /* 20 KB RAM */

/* Timing constants */
#define BOOTLOADER_TIMEOUT  5000  /* 5 seconds timeout before auto-jump to app */
#define HEARTBEAT_INTERVAL  500   /* Heartbeat every 500ms */
#define UART_TIMEOUT_MS     5000  /* UART receive timeout */

/* Debug output enable (set to 1 to enable debug messages) */
#define DEBUG_OUTPUT        0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* UART handle from usart.c */
extern UART_HandleTypeDef huart1;

/* Flash write buffer */
static uint8_t page_buffer[1024];  /* 1KB page buffer in RAM */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void uart_send_byte(uint8_t byte);
static uint8_t uart_recv_byte(uint32_t timeout_ms);
static void uart_flush_rx_buffer(void);
static uint8_t simple_checksum(const uint8_t *data, uint16_t len);
static int flash_write_page(uint32_t addr, const uint8_t *data, uint16_t size);
static void flash_loader_main(void);
static void jump_to_application(void);
static int check_application_valid(void);

#if DEBUG_OUTPUT
static void debug_print(const char *msg)
{
  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
}
#else
#define debug_print(msg) /* No debug output */
#endif

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
  * @brief Receive single byte via UART with timeout (returns 0xFF on timeout)
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
  * @brief Flush UART receive buffer to clear any residual data
  * @retval None
  */
static void uart_flush_rx_buffer(void)
{
  uint8_t dummy;
  /* Read and discard all pending bytes with short timeout */
  while (HAL_UART_Receive(&huart1, &dummy, 1, 10) == HAL_OK)
  {
    /* Discard byte */
  }
  __HAL_UART_FLUSH_DRREGISTER(&huart1);
}

/**
  * @brief Check if application is valid
  * @retval 1 if valid, 0 if invalid
  */
static int check_application_valid(void)
{
  uint32_t app_sp = *(volatile uint32_t *)(APP_BASE_ADDR + 0);
  uint32_t app_reset = *(volatile uint32_t *)(APP_BASE_ADDR + 4);
  
  /* Check if stack pointer is in valid RAM range */
  if (app_sp < RAM_START_ADDR || app_sp > RAM_END_ADDR)
  {
    return 0;
  }
  
  /* Check if reset vector is in application flash range (must be odd for Thumb) */
  if ((app_reset < APP_BASE_ADDR) || (app_reset > (BOOT_BASE_ADDR + FLASH_SIZE)))
  {
    return 0;
  }
  
  /* Check if reset vector has Thumb bit set */
  if ((app_reset & 0x1) == 0)
  {
    return 0;
  }
  
  return 1;
}

/**
  * @brief Calculate simple checksum (sum of all bytes)
  * @param data: pointer to data buffer
  * @param len: length of data
  * @retval 8-bit checksum value
  */
static uint8_t simple_checksum(const uint8_t *data, uint16_t len)
{
  uint8_t sum = 0;
  for (uint16_t i = 0; i < len; i++)
  {
    sum += data[i];
  }
  return sum;
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
  * @brief Main flash loader routine with 5-second auto-boot timeout
  * 
  * If no START command received within 5 seconds, jump to application.
  * Otherwise, receive firmware via UART protocol.
  *
  * @retval None
  */
static void flash_loader_main(void)
{
    uint8_t cmd;
    uint8_t chunk[CHUNK_SIZE];
    uint8_t checksum_rx, checksum_calc;
    uint32_t flash_addr = APP_BASE_ADDR;
    uint16_t page_idx = 0;
    uint32_t total_bytes = 0;
    int status;
    
    uint32_t start_time = HAL_GetTick();
    uint32_t last_heartbeat = 0;

    /* Wait for START command with 5-second timeout */
    while (1)
    {
        /* Check for timeout - auto-jump to application if valid */
        if ((HAL_GetTick() - start_time) > BOOTLOADER_TIMEOUT)
        {
            if (check_application_valid())
            {
                return;  /* Timeout - jump to application */
            }
            else
            {
                /* No valid application, reset timeout and continue waiting */
                start_time = HAL_GetTick();
            }
        }
        
        /* Send heartbeat every 500ms */
        if ((HAL_GetTick() - last_heartbeat) > HEARTBEAT_INTERVAL)
        {
            const char *msg = "BOOT\r\n";
            HAL_UART_Transmit(&huart1, (uint8_t*)msg, 6, 100);
            last_heartbeat = HAL_GetTick();
        }
        
        /* Check for START command */
        cmd = uart_recv_byte(100);
        if (cmd == CMD_START)
        {
            uart_flush_rx_buffer();
            uart_send_byte(ACK);
            HAL_Delay(50);
            uart_flush_rx_buffer();
            debug_print("START\r\n");
            break;
        }
    }

    /* Initialize page buffer with 0xFF */
    memset(page_buffer, 0xFF, PAGE_SIZE);

    /* Main transfer loop - receive 8-byte chunks */
    while (1)
    {
        cmd = uart_recv_byte(UART_TIMEOUT_MS);

        if (cmd == CMD_DATA)
        {
            debug_print("D");
            
            /* Receive 8-byte chunk */
            if (HAL_UART_Receive(&huart1, chunk, CHUNK_SIZE, UART_TIMEOUT_MS) != HAL_OK)
            {
                debug_print("T\r\n");
                uart_send_byte(NAK);
                goto next_chunk;
            }

            /* Receive checksum */
            if (HAL_UART_Receive(&huart1, &checksum_rx, 1, UART_TIMEOUT_MS) != HAL_OK)
            {
                debug_print("CT\r\n");
                uart_send_byte(NAK);
                goto next_chunk;
            }

            /* Verify checksum */
            checksum_calc = simple_checksum(chunk, CHUNK_SIZE);
            if (checksum_calc != checksum_rx)
            {
                debug_print("CE\r\n");
                uart_send_byte(NAK);
                goto next_chunk;
            }

            /* Copy 8 bytes to page buffer */
            memcpy(&page_buffer[page_idx], chunk, CHUNK_SIZE);
            page_idx += CHUNK_SIZE;
            total_bytes += CHUNK_SIZE;

            /* If page buffer is full (1KB), write to flash */
            if (page_idx >= PAGE_SIZE)
            {
                status = flash_write_page(flash_addr, page_buffer, PAGE_SIZE);
                if (status != 0)
                {
                    debug_print("FE\r\n");
                    uart_send_byte(NAK);
                    return;
                }
                flash_addr += PAGE_SIZE;
                page_idx = 0;
                memset(page_buffer, 0xFF, PAGE_SIZE);  /* Reset for next page */
            }

            /* Check size limit */
            if (total_bytes > APP_MAX_SIZE)
            {
                debug_print("OVF\r\n");
                uart_send_byte(NAK);
                return;
            }

            /* Send ACK - host will wait for this before sending next chunk */
            uart_send_byte(ACK);
            debug_print("A\r\n");
        }
        else if (cmd == CMD_END)
        {
            debug_print("END\r\n");
            
            /* Flush any remaining data in page buffer */
            if (page_idx > 0)
            {
                /* Already filled with 0xFF, just need to ensure word alignment */
                if (page_idx & 1)
                {
                    page_idx++;
                }
                status = flash_write_page(flash_addr, page_buffer, page_idx);
                if (status != 0)
                {
                    debug_print("FLE\r\n");
                    uart_send_byte(NAK);
                    return;
                }
            }
            uart_send_byte(ACK);
            debug_print("DONE\r\n");
            HAL_Delay(200);  /* Ensure UART transmission completes */
            return;  /* Jump to application */
        }

next_chunk:
        ;
    }
}

/**
  * @brief Jump to application in flash
  * 
  * Performs clean handoff to application firmware:
  *   1. Disables interrupts and peripheral clocks
  *   2. Deinitializes HAL peripherals (especially UART)
  *   3. Relocates NVIC vector table to application base
  *   4. Sets stack pointer and jumps to reset handler
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

  /* Initialize UART (115200 baud for flash loader) */
  MX_USART1_UART_Init();
  
  /* Run flash loader protocol (heartbeat sent in flash_loader_main) */
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
