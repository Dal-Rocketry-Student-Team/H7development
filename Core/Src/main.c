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
#include "memorymap.h"
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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEG2RAD 0.017453292519943295f // Pi / 180
#define RAD2DEG 57.29577951308232f    // 180 / Pi
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

/*
===============
SERVO FUNCTIONS
===============
*/
void Servo_SetAngle(TIM_HandleTypeDef *htim, uint32_t Channel, float angle)
{
    // angle range: -90 to +90 degrees
    float pulse_length_ms = ((angle + 90.0f) / 180.0f) * 1.0f + 1.0f; // maps [-90,+90] to [1ms,2ms]

    // Convert pulse length in ms to timer counts (0.2us resolution)
    uint32_t pulse_counts = (uint32_t)(pulse_length_ms * 5000.0f);

    __HAL_TIM_SET_COMPARE(htim, Channel, pulse_counts);
}

void Servo_Sweep_Demo(TIM_HandleTypeDef *htim, uint32_t Channel)
{
    const int delay_ms = 10;  // Adjust this for speed of sweep
    float angle;

    // 0° to -90°
    for (angle = 0; angle >= -90; angle -= 1.0f) {
        Servo_SetAngle(htim, Channel, angle);
        HAL_Delay(delay_ms);
    }

    HAL_Delay(500);

    // -90° to +90°
    for (angle = -90; angle <= 90; angle += 1.0f) {
        Servo_SetAngle(htim, Channel, angle);
        HAL_Delay(delay_ms);
    }

    HAL_Delay(500);

    // +90° back to 0°
    for (angle = 90; angle >= 0; angle -= 1.0f) {
        Servo_SetAngle(htim, Channel, angle);
        HAL_Delay(delay_ms);
    }
}

// To redirect the printf to output to the UART instead so I can see it in putty
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart5, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3) {
      fusion_tick = 1;              // 500 Hz “call Madgwick now”
  }
}

static void IMU_CalibrateGyro(stmdev_ctx_t *ctx, float bias_out_dps[3]) {
    // Assumes the board is held still for ~0.5 s
    const int N = 200;
    int32_t sx = 0, sy = 0, sz = 0;
    int16_t g[3];
    for (int i = 0; i < N; i++) {
        lsm6dsv16x_angular_rate_raw_get(ctx, g);
        sx += g[0]; sy += g[1]; sz += g[2];
        HAL_Delay(2); // ~500 ms total
    }
    float gx = (float)(sx / N);
    float gy = (float)(sy / N);
    float gz = (float)(sz / N);
    bias_out_dps[0] = lsm6dsv16x_from_fs500_to_mdps((int16_t)gx) / 1000.0f;
    bias_out_dps[1] = lsm6dsv16x_from_fs500_to_mdps((int16_t)gy) / 1000.0f;
    bias_out_dps[2] = lsm6dsv16x_from_fs500_to_mdps((int16_t)gz) / 1000.0f;
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
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_Base_Start_IT(&htim3);      // start periodic update IRQ

  // Setup lsm6dsv16x_ctx correctly for thios device setup
  lsm6dsv16x_ctx.handle = &hspi2;
  lsm6dsv16x_ctx.mdelay = HAL_Delay;
  lsm6dsv16x_ctx.write_reg = platform_write;
  lsm6dsv16x_ctx.read_reg = platform_read;

  // Perform self tests on the accelerometer and gyroscope
  lsm6dsv16x_xl_self_test_set(&lsm6dsv16x_ctx, LSM6DSV16X_XL_ST_POSITIVE);
  lsm6dsv16x_gy_self_test_set(&lsm6dsv16x_ctx, LSM6DSV16X_GY_ST_POSITIVE);
  HAL_Delay(500); // Wait for the self test to complete
  lsm6dsv16x_xl_self_test_set(&lsm6dsv16x_ctx, LSM6DSV16X_XL_ST_DISABLE);
  lsm6dsv16x_gy_self_test_set(&lsm6dsv16x_ctx, LSM6DSV16X_GY_ST_DISABLE);

  /*----------Device Reset-----------*/
  lsm6dsv16x_sh_reset_set(&lsm6dsv16x_ctx, PROPERTY_ENABLE);
  uint8_t rst;
  do {
    lsm6dsv16x_sh_reset_get(&lsm6dsv16x_ctx, &rst);
  } while (rst);

  // Match Madgwick rate to IMU ODR (you set 104 Hz)
  sampleFreq = 104.0f;

  // Quick gyro bias while stationary
  IMU_CalibrateGyro(&lsm6dsv16x_ctx, gyro_bias_dps);


  /*---------------Run Time Settings--------------*/
  lsm6dsv16x_block_data_update_set(&lsm6dsv16x_ctx, PROPERTY_ENABLE);

  // Enable high performance mode (API uses "mode" naming)
  lsm6dsv16x_xl_mode_set(&lsm6dsv16x_ctx, LSM6DSV16X_XL_HIGH_PERFORMANCE_MD);
  lsm6dsv16x_gy_mode_set(&lsm6dsv16x_ctx, LSM6DSV16X_GY_HIGH_PERFORMANCE_MD);

  lsm6dsv16x_xl_data_rate_set(&lsm6dsv16x_ctx, LSM6DSV16X_ODR_AT_120Hz);
  lsm6dsv16x_gy_data_rate_set(&lsm6dsv16x_ctx, LSM6DSV16X_ODR_AT_120Hz);
  lsm6dsv16x_xl_full_scale_set(&lsm6dsv16x_ctx, LSM6DSV16X_8g);
  lsm6dsv16x_gy_full_scale_set(&lsm6dsv16x_ctx, LSM6DSV16X_500dps);

  // Enable Low Pass Filter 1/2 on the accelerometer and gyroscope
  // (driver function names use the 'filt_' prefix)
  lsm6dsv16x_filt_xl_lp2_set(&lsm6dsv16x_ctx, PROPERTY_ENABLE);
  lsm6dsv16x_filt_gy_lp1_set(&lsm6dsv16x_ctx, PROPERTY_ENABLE);
  /*
   * The previous code used a function named
   *   lsm6dsv16x_gy_hp_path_internal_set(..., LSM6DSV16X_HP_FILTER_16mHz)
   * which does not exist in the provided lsm6dsv16x driver API. If a
   * gyro high-pass internal path is required, pick the appropriate
   * API from lsm6dsv16x_reg.h (for example filt_xl_hp_set is available
   * for the accelerometer). For now this call is omitted.
   */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    // Servo_Sweep_Demo(&htim2, TIM_CHANNEL_2);

    lsm6dsv16x_flag_data_ready_get(&lsm6dsv16x_ctx, &drdy);

    if (drdy.drdy_xl && drdy.drdy_gy) {
      // If both accelerometer and gyroscope data are ready, retrieve the data
      lsm6dsv16x_acceleration_raw_get(&lsm6dsv16x_ctx, (int16_t*)accel_raw);
      lsm6dsv16x_angular_rate_raw_get(&lsm6dsv16x_ctx, (int16_t*)gyro_raw);

      accel_g[0] = (lsm6dsv16x_from_fs8_to_mg(accel_raw[0])) / 1000.0f;
      accel_g[1] = (lsm6dsv16x_from_fs8_to_mg(accel_raw[1])) / 1000.0f;
      accel_g[2] = (lsm6dsv16x_from_fs8_to_mg(accel_raw[2])) / 1000.0f;

      gyro_dps[0] = (lsm6dsv16x_from_fs500_to_mdps(gyro_raw[0])) / 1000.0f;
      gyro_dps[1] = (lsm6dsv16x_from_fs500_to_mdps(gyro_raw[1])) / 1000.0f;
      gyro_dps[2] = (lsm6dsv16x_from_fs500_to_mdps(gyro_raw[2])) / 1000.0f;

      // Normalize accel in-place (required by Madgwick)
      float inv = 1.0f / sqrtf(accel_g[0]*accel_g[0] + accel_g[1]*accel_g[1] + accel_g[2]*accel_g[2]);
      accel_g[0] *= inv;  accel_g[1] *= inv;  accel_g[2] *= inv;

      // Run fusion (IMU variant: gyro in rad/s, accel in g, normalized)
      MadgwickAHRSupdateIMU(gyro_dps[0]*DEG2RAD, gyro_dps[1]*DEG2RAD, gyro_dps[2]*DEG2RAD,
                            accel_g[0],         accel_g[1],         accel_g[2]);

      // Stream quaternion + sensors (CSV line that your Python can parse)
      printf("IMU,%lu,%.6f,%.6f,%.6f,%.6f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f\r\n",
              (unsigned long)HAL_GetTick(), q0, q1, q2, q3,
              gyro_dps[0], gyro_dps[1], gyro_dps[2],
              accel_g[0],  accel_g[1],  accel_g[2]);

    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
