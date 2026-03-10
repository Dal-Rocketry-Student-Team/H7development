/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
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
#include <string.h>

#include "lsm6dsv16x_reg.h" // LSM6DSV16X driver header file
#include "MadgwickAHRS.h" // Madgwick AHRS algorithm header file

#include "sx1262.h"        // SX1262 LoRa driver header file
#include "sx1262_hal.h"     // SX1262 hardware abstraction header file
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEG2RAD 0.017453292519943295f // Pi / 180
#define RAD2DEG 57.29577951308232f    // 180 / Pi

/* ====== EDIT IF NEEDED ====== */
#define SPI_HANDLE   hspi2                 // SPI instance wired to IMU
#define CS_PORT      LSM_NCS_GPIO_Port     // IMU chip-select port
#define CS_PIN       LSM_NCS_Pin           // IMU chip-select pin
/* ============================ */
#define REG_WHO_AM_I 0x0F
#define SPI_READ_BIT 0x80
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t fusion_tick = 0;

// --- Madgwick globals exposed by the library ---
extern volatile float q0, q1, q2, q3;      // quaternion (from Madgwick)
extern volatile float sampleFreq;          // Madgwick internal sample rate
static float gyro_bias_dps[3] = {0};       // boot-time gyro bias estimate

// There are 3 axes of data for both the accelerometer and gyroscope, each a 16 bit value
int16_t accel_raw[3] = {0}, gyro_raw[3] = {0};
float accel_g[3] = {0}, gyro_dps[3] = {0};

// Making an instance of the ctx_t struct to use in accessing the lsm6dsv16x
stmdev_ctx_t lsm6dsv16x_ctx;

// data-ready flags to see if new data is available
lsm6dsv16x_data_ready_t drdy;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
================================
PLATFORM COMMUNICATION FUNCTIONS
================================
*/

int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
    HAL_GPIO_WritePin(LSM_NCS_GPIO_Port, LSM_NCS_Pin, GPIO_PIN_RESET);

    uint8_t tx_buf[1] = { reg & 0x7F }; // Write operation
    HAL_SPI_Transmit(handle, tx_buf, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(handle, (uint8_t*)bufp, len, HAL_MAX_DELAY);

    HAL_GPIO_WritePin(LSM_NCS_GPIO_Port, LSM_NCS_Pin, GPIO_PIN_SET);
    return 0;
}

int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
    HAL_GPIO_WritePin(LSM_NCS_GPIO_Port, LSM_NCS_Pin, GPIO_PIN_RESET);

    uint8_t tx_buf[1] = { reg | 0x80 }; // Read operation
    HAL_SPI_Transmit(handle, tx_buf, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(handle, bufp, len, HAL_MAX_DELAY);

    HAL_GPIO_WritePin(LSM_NCS_GPIO_Port, LSM_NCS_Pin, GPIO_PIN_SET);
    return 0;
}

// To redirect the printf to output to the UART instead so I can see it in putty
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart5, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

static void IMU_Init_LSM6DSV16X(stmdev_ctx_t *ctx) {
    uint8_t who = 0;

    // 1) WHO_AM_I sanity
    lsm6dsv16x_device_id_get(ctx, &who);

    // 2) Reset (use one)
    lsm6dsv16x_sw_reset(ctx);
    HAL_Delay(10);

    // 3) Force UI to SPI-only behavior (good hygiene)
    lsm6dsv16x_ui_i2c_i3c_mode_set(ctx, LSM6DSV16X_I2C_I3C_DISABLE);
    lsm6dsv16x_spi_mode_set(ctx, LSM6DSV16X_SPI_4_WIRE);

    // 4) Safe defaults for multi-byte reads + coherence
    lsm6dsv16x_auto_increment_set(ctx, 1);
    lsm6dsv16x_block_data_update_set(ctx, 1);

    // 5) Modes
    lsm6dsv16x_xl_mode_set(ctx, LSM6DSV16X_XL_HIGH_PERFORMANCE_MD);
    lsm6dsv16x_gy_mode_set(ctx, LSM6DSV16X_GY_HIGH_PERFORMANCE_MD);

    // 6) Scales (rocket-safe defaults)
    lsm6dsv16x_xl_full_scale_set(ctx, LSM6DSV16X_16g);
    lsm6dsv16x_gy_full_scale_set(ctx, LSM6DSV16X_2000dps);

    // 7) Filters (optional)
    lsm6dsv16x_filt_xl_lp2_set(ctx, 1);
    lsm6dsv16x_filt_gy_lp1_set(ctx, 1);

    // 8) ODR ON (turns sensors on)
    lsm6dsv16x_xl_data_rate_set(ctx, LSM6DSV16X_ODR_AT_240Hz);
    lsm6dsv16x_gy_data_rate_set(ctx, LSM6DSV16X_ODR_AT_240Hz);

    // Give it a moment to start producing samples
    HAL_Delay(20);
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

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_UART5_Init();
  MX_USART3_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_Base_Start_IT(&htim3);      // start periodic update IRQ

  // Setup lsm6dsv16x_ctx correctly for this device setup
  lsm6dsv16x_ctx.handle = &hspi2;
  lsm6dsv16x_ctx.mdelay = HAL_Delay;
  lsm6dsv16x_ctx.write_reg = platform_write;
  lsm6dsv16x_ctx.read_reg = platform_read;

  IMU_Init_LSM6DSV16X(&lsm6dsv16x_ctx);

  /* USER CODE BEGIN 2 */

  /* --- SX1262 LoRa radio initialisation --- */
  SX1262_Init();

  // Configure LoRa: 915 MHz, SF9, BW125K, CR4/5 - good balance of 
  // range and data rate for our use case. Adjust as needed.
  sx1262_lora_mod_t mod = {
    .sf = SX1262_LORA_SF9,
    .bw = SX1262_LORA_BW_125K,
    .cr = SX1262_LORA_CR_4_5,
    .ldro = false, // not needed because auto-set by library based on SF and BW
  };

  sx1262_lora_pkt_t pkt = {
    .preamble_len = 12,      // 12 symbols recommended by Semtech
    .fixed_length = false,   // explicit header (variable length)
    .payload_len  = 64,      // max expected payload
    .crc_on       = true,    // always use CRC for flight data
    .invert_iq    = false,   // normal IQ
  };

  SX1262_ConfigureLora(915000000, &mod, &pkt);

  printf("\r\n=== MISO PIN PROBE ===\r\n");

  // De-init whichever SPI is on PA5/6/7 so we can read PA6 as GPIO
  HAL_SPI_DeInit(&hspi1);  // or hspi6, whichever you have active

  GPIO_InitTypeDef g = {0};
  g.Pin = GPIO_PIN_6;  // PA6 = MISO
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &g);

  // Reset E22 first
  HAL_GPIO_WritePin(E22_RESET_GPIO_Port, E22_RESET_Pin, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(E22_RESET_GPIO_Port, E22_RESET_Pin, GPIO_PIN_SET);
  HAL_Delay(50);
  while(HAL_GPIO_ReadPin(E22_BUSY_GPIO_Port, E22_BUSY_Pin)) {}

  printf("MISO (CS high): %d\r\n",
        HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6));

  HAL_GPIO_WritePin(E22_NCS_GPIO_Port, E22_NCS_Pin, GPIO_PIN_RESET);
  HAL_Delay(1);
  printf("MISO (CS low):  %d\r\n",
        HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6));

  HAL_GPIO_WritePin(E22_NCS_GPIO_Port, E22_NCS_Pin, GPIO_PIN_SET);

  printf("=== MISO PROBE COMPLETE ===\r\n");
  while(1) {}
    
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
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
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
