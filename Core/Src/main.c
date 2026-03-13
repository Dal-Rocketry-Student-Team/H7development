/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : SPI1 bus test using MS5607 barometer
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart5, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_UART5_Init();
  MX_USART3_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  printf("\r\n\r\n====== SPI1 BUS TEST — MS5607 BAROMETER ======\r\n");

  /* ---- Fix SPI1 for reliable operation ---- */
  hspi1.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
  hspi1.Init.FifoThreshold     = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.NSS               = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;  /* 4 MHz */
  HAL_SPI_Init(&hspi1);
  printf("SPI1: 4 MHz, NSSP=off, CPOL=0/CPHA=0\r\n\r\n");

  /* ---- Deselect ALL SPI1 devices ---- */
  HAL_GPIO_WritePin(E22_NCS_GPIO_Port,  E22_NCS_Pin,  GPIO_PIN_SET);  /* radio */
  HAL_GPIO_WritePin(MS5_NCS_GPIO_Port,  MS5_NCS_Pin,  GPIO_PIN_SET);  /* baro  */

  /* ================================================================
   * MS5607 TEST
   *
   * Protocol: SPI mode 0 (CPOL=0, CPHA=0) — matches our SPI1 config
   * Reset command: 0x1E, then wait 2.8 ms
   * PROM read:     0xA0 + (addr << 1), returns 16 bits MSB first
   *   addr 0 = factory/setup
   *   addr 1 = C1 (pressure sensitivity)
   *   addr 2 = C2 (pressure offset)
   *   addr 3 = C3
   *   addr 4 = C4
   *   addr 5 = C5 (reference temperature)
   *   addr 6 = C6
   *   addr 7 = CRC + serial
   * ================================================================ */

  /* ---- Step 1: Reset ---- */
  printf("--- Step 1: Reset (0x1E) ---\r\n");
  {
      uint8_t cmd = 0x1E;
      HAL_GPIO_WritePin(MS5_NCS_GPIO_Port, MS5_NCS_Pin, GPIO_PIN_RESET);
      HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
      HAL_GPIO_WritePin(MS5_NCS_GPIO_Port, MS5_NCS_Pin, GPIO_PIN_SET);
  }
  HAL_Delay(5);  /* datasheet says 2.8 ms reload time */
  printf("Reset sent, waited 5 ms\r\n\r\n");

  /* ---- Step 2: Read all 8 PROM addresses ---- */
  printf("--- Step 2: Read PROM (calibration data) ---\r\n");
  uint16_t prom[8] = {0};
  uint8_t any_nonzero = 0;
  uint8_t all_ff = 1;

  for (int addr = 0; addr < 8; addr++) {
      uint8_t cmd = 0xA0 | (addr << 1);   /* PROM read command */
      uint8_t rx[2] = {0};

      HAL_GPIO_WritePin(MS5_NCS_GPIO_Port, MS5_NCS_Pin, GPIO_PIN_RESET);
      HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
      HAL_SPI_Receive(&hspi1, rx, 2, HAL_MAX_DELAY);
      HAL_GPIO_WritePin(MS5_NCS_GPIO_Port, MS5_NCS_Pin, GPIO_PIN_SET);

      prom[addr] = ((uint16_t)rx[0] << 8) | rx[1];

      printf("  PROM[%d] cmd=0x%02X -> 0x%04X (%5u)\r\n",
             addr, cmd, prom[addr], prom[addr]);

      if (prom[addr] != 0x0000) any_nonzero = 1;
      if (prom[addr] != 0xFFFF) all_ff = 0;
  }

  printf("\r\n--- Result ---\r\n");
  if (any_nonzero && !all_ff) {
      printf(">> PASS: Got real calibration data! SPI1 reads work.\r\n");
      printf("   C1=%u C2=%u C3=%u C4=%u C5=%u C6=%u\r\n",
             prom[1], prom[2], prom[3], prom[4], prom[5], prom[6]);
  } else if (!any_nonzero) {
      printf(">> FAIL: All zeros — MS5607 not responding.\r\n");
      printf("   Check: MS5_NCS wiring, PS pin tied LOW for SPI mode,\r\n");
      printf("   power supply, solder joints.\r\n");
  } else if (all_ff) {
      printf(">> FAIL: All 0xFFFF — MISO stuck high.\r\n");
  } else {
      printf(">> PARTIAL: Some data, but check values look odd.\r\n");
  }

  /* ---- Step 3: Quick conversion test ---- */
  printf("\r\n--- Step 3: Pressure conversion test ---\r\n");
  {
      /* Start D1 conversion, OSR=4096 */
      uint8_t cmd = 0x48;
      HAL_GPIO_WritePin(MS5_NCS_GPIO_Port, MS5_NCS_Pin, GPIO_PIN_RESET);
      HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
      HAL_GPIO_WritePin(MS5_NCS_GPIO_Port, MS5_NCS_Pin, GPIO_PIN_SET);
  }
  HAL_Delay(10);  /* max conversion time is 9.04 ms for OSR=4096 */

  /* ADC read */
  uint32_t d1_raw = 0;
  {
      uint8_t cmd = 0x00;  /* ADC read command */
      uint8_t rx[3] = {0};
      HAL_GPIO_WritePin(MS5_NCS_GPIO_Port, MS5_NCS_Pin, GPIO_PIN_RESET);
      HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
      HAL_SPI_Receive(&hspi1, rx, 3, HAL_MAX_DELAY);
      HAL_GPIO_WritePin(MS5_NCS_GPIO_Port, MS5_NCS_Pin, GPIO_PIN_SET);

      d1_raw = ((uint32_t)rx[0] << 16) | ((uint32_t)rx[1] << 8) | rx[2];
      printf("  D1 (pressure) raw = %lu (0x%06lX)\r\n",
             (unsigned long)d1_raw, (unsigned long)d1_raw);
      if (d1_raw == 0)
          printf("  >> Zero — conversion may have failed\r\n");
      else if (d1_raw == 0xFFFFFF)
          printf("  >> All 1s — MISO stuck\r\n");
      else
          printf("  >> Non-zero ADC value — sensor is converting!\r\n");
  }

  /* ---- Step 4: Temperature conversion ---- */
  {
      uint8_t cmd = 0x58;  /* D2 conversion, OSR=4096 */
      HAL_GPIO_WritePin(MS5_NCS_GPIO_Port, MS5_NCS_Pin, GPIO_PIN_RESET);
      HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
      HAL_GPIO_WritePin(MS5_NCS_GPIO_Port, MS5_NCS_Pin, GPIO_PIN_SET);
  }
  HAL_Delay(10);
  uint32_t d2_raw = 0;
  {
      uint8_t cmd = 0x00;
      uint8_t rx[3] = {0};
      HAL_GPIO_WritePin(MS5_NCS_GPIO_Port, MS5_NCS_Pin, GPIO_PIN_RESET);
      HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
      HAL_SPI_Receive(&hspi1, rx, 3, HAL_MAX_DELAY);
      HAL_GPIO_WritePin(MS5_NCS_GPIO_Port, MS5_NCS_Pin, GPIO_PIN_SET);

      d2_raw = ((uint32_t)rx[0] << 16) | ((uint32_t)rx[1] << 8) | rx[2];
      printf("  D2 (temp)     raw = %lu (0x%06lX)\r\n",
             (unsigned long)d2_raw, (unsigned long)d2_raw);
  }

  /* ---- Compute actual temp + pressure if we got real data ---- */
  if (any_nonzero && !all_ff && d1_raw != 0 && d2_raw != 0) {
      printf("\r\n--- Computed values ---\r\n");
      int32_t dT   = (int32_t)d2_raw - ((int32_t)prom[5] << 8);
      int32_t TEMP = 2000 + ((int64_t)dT * prom[6]) / (1L << 23);
      int64_t OFF  = ((int64_t)prom[2] << 17) + ((int64_t)prom[4] * dT) / (1L << 6);
      int64_t SENS = ((int64_t)prom[1] << 16) + ((int64_t)prom[3] * dT) / (1L << 7);
      int32_t P    = (int32_t)(((int64_t)d1_raw * SENS / (1L << 21)) - OFF) / (1L << 15);

      printf("  Temperature = %ld.%02ld C\r\n", (long)(TEMP/100), (long)(TEMP%100));
      printf("  Pressure    = %ld.%02ld mbar\r\n", (long)(P/100), (long)(P%100));
  }

  printf("\r\n====== DONE ======\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      HAL_Delay(1000);
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) { Error_Handler(); }
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};
  HAL_MPU_Disable();
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */