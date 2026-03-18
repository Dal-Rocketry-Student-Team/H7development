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
#include <math.h>

#include "lsm6dsv16x_reg.h" // LSM6DSV16X driver header file
#include "MadgwickAHRS.h" // Madgwick AHRS algorithm header file
#include "sx1262.h"        // SX1262 LoRa driver header file
#include "sx1262_hal.h"     // SX1262 hardware abstraction header file
#include "MS5607SPI.h"     // MS5607 barometer driver header file
#include "telemetry.h"     // Telemetry packet definitions

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

// === LSM6DSV variables ===
int16_t accel_raw[3] = {0}, gyro_raw[3] = {0};// There are 3 axes of data for both the accelerometer and gyroscope, each a 16 bit value
float accel_g[3] = {0}, gyro_dps[3] = {0};
stmdev_ctx_t lsm6dsv16x_ctx;// Making an instance of the ctx_t struct to use in accessing the lsm6dsv16x
lsm6dsv16x_data_ready_t drdy;// data-ready flags to see if new data is available

// === SX1262 RX variables (always in scope, used by RX mode) ===
static uint32_t rx_pkt_count = 0;
uint8_t rx_buf[255] = {0};      // SX1262 max payload is 255 bytes
uint8_t rx_len = 0;
int rx_rc = 0;
uint8_t i = 0;
sx1262_pkt_status_t pkt_status = {0};

// === Ground station barometer (updated every loop, used as altitude reference) ===
static float gs_temp_c     = 15.0f;    // initialised to ISA standard, overwritten immediately
static float gs_pressure_pa = 101325.0f;

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

/**
 * @brief  Calculate altitude above ground level using the hypsometric formula.
 *
 * Formula:  h = (T_ground_K / 0.0065) * (1 - (P_rocket / P_ground)^0.190295)
 *
 * Why this formula works:
 *   Pressure drops roughly exponentially with altitude. The hypsometric formula
 *   inverts that relationship to get height from a pressure ratio. The exponent
 *   0.190295 comes from R*L/g — the gas constant for dry air, the temperature
 *   lapse rate, and gravitational acceleration combined into one constant.
 *
 * Why we use ground station temp and pressure instead of sea-level constants:
 *   Using the ground station's LIVE readings as the reference gives altitude
 *   above ground level (AGL) directly. If we used sea-level constants we would
 *   get altitude above sea level, which requires knowing the launch site elevation
 *   and is less useful in flight.
 *
 * @param  rocket_Pa      Pressure reading from the rocket barometer (Pa)
 * @param  ground_Pa      Pressure reading from the ground station barometer (Pa)
 * @param  ground_temp_C  Temperature from the ground station barometer (°C)
 * @return Altitude AGL in metres. Positive = above ground. Zero at launch site.
 */
static float calculate_altitude_m(float rocket_Pa, float ground_Pa, float ground_temp_C)
{
    float T_K = ground_temp_C + 273.15f;           // Convert Celsius to Kelvin
    float ratio = rocket_Pa / ground_Pa;           // Pressure ratio, rocket / ground
    return (T_K / 0.0065f) * (1.0f - powf(ratio, 0.190295f));
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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_Base_Start_IT(&htim3);      // start periodic update IRQ

  // Setup lsm6dsv16x_ctx correctly for this device setup
  lsm6dsv16x_ctx.handle = &hspi2;
  lsm6dsv16x_ctx.mdelay = HAL_Delay;
  lsm6dsv16x_ctx.write_reg = platform_write;
  lsm6dsv16x_ctx.read_reg = platform_read;

  IMU_Init_LSM6DSV16X(&lsm6dsv16x_ctx);

  // 1. Initialize, at startup
  MS5607StateTypeDef ms5607_status = MS5607_Init(&hspi1, MS5_NCS_GPIO_Port, MS5_NCS_Pin);
  if (ms5607_status == MS5607_STATE_FAILED) {
      printf("Barometer init failed!\r\n");
  }

  // 2. Set baro and temp OSR (oversampling ratio) to max for best resolution (and slowest conversion time)
  MS5607SetPressureOSR(OSR_4096);
  MS5607SetTemperatureOSR(OSR_4096);

  printf("\r\n=== GROUNDSTATION RX ===\r\n\n");

  /* --------------------------------------------------------
   * Step 0: Full hardware + chip init (reset, TCXO, cal, DC-DC)
   * This follows section 14.2 pre-requisites.
   * -------------------------------------------------------- */

  SX1262_Init();

  {
      printf("SX1262 err code after RADIO INIT: 0x%04X\r\n", SX1262_GetDeviceErrors());
      uint8_t status = SX1262_GetStatus();
      sx1262_status_t decoded;
      SX1262_DecodeStatus(status, &decoded);
      printf("  → Status: 0x%02X | Cmd Status: 0x%X, Chip Mode: 0x%X\r\n",
            status, decoded.cmd_status, decoded.chip_mode);
  }

  SX1262_HW_DelayMs(5);

  /* --------------------------------------------------------
   * Step 1-5 (Section 14.2): Configure LoRa radio
   *   1. SetStandby(STDBY_RC)          — done inside ConfigureLora
   *   2. SetPacketType(LoRa)           — done inside ConfigureLora
   *   3. SetRfFrequency(915 MHz)       — done inside ConfigureLora
   *      + CalibrateImage(902-928 MHz) — done inside ConfigureLora
   *   4. SetPaConfig(+22 dBm SX1262)   — done inside ConfigureLora
   *   5. SetTxParams(+22 dBm, 200us)   — done inside ConfigureLora
   *   + SetModulationParams, SetPacketParams, SyncWord, IRQs
   * -------------------------------------------------------- */
  sx1262_lora_mod_t mod = {
      .sf   = SX1262_LORA_SF9,
      .bw   = SX1262_LORA_BW_125K,
      .cr   = SX1262_LORA_CR_4_5,
      .ldro = false,
  };

  sx1262_lora_pkt_t pkt = {
      .preamble_len = 12,
      .fixed_length = false,   /* explicit header */
      .payload_len  = sizeof(rocket_telemetry_t),
      .crc_on       = true,
      .invert_iq    = false,
  };

  SX1262_ConfigureLora(915000000UL, &mod, &pkt);
  printf("Radio configured: 915 MHz, SF9, BW125K, CR4/5, +22 dBm\r\n");
  
  printf("SX1262 err code after CONFIGURE LORA: 0x%04X\r\n", SX1262_GetDeviceErrors());
  
  {
    uint8_t status = SX1262_GetStatus();
    sx1262_status_t decoded;
    SX1262_DecodeStatus(status, &decoded);
    printf("  → Status: 0x%02X | Cmd Status: 0x%X, Chip Mode: 0x%X\r\n", status, decoded.cmd_status, decoded.chip_mode);
  }

  /* Quick sanity: read back sync word */
  {
      uint8_t sw[2] = {0};
      SX1262_ReadRegister(SX1262_REG_LORA_SYNC_WORD_MSB, sw, 2);
      printf("Sync word readback: 0x%02X%02X (expect 0x1424)\r\n", sw[0], sw[1]);
  }

  /* Check device errors before transmitting */
  {
      uint16_t errs = SX1262_GetDeviceErrors();
      printf("Device errors pre-TX: 0x%04X %s\r\n", errs,
             errs == 0 ? "(clean)" : "(WARNING)");
      if (errs) SX1262_ClearDeviceErrors();
  }
  
  /* ======== CONTINUOUS RX MODE (GROUND STATION) ========
  * Continuously listens for incoming LoRa packets at 915 MHz.
  * Prints packet contents, signal strength (RSSI/SNR), and CRC status.
  * Timeout set to 0 = continuous reception mode.
  */
  printf("Starting continuous RX mode at 915 MHz...\r\n");
  printf("Waiting for LoRa packets (SF9, BW125K, private sync word)...\r\n");
    
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {

    /* === Barometer Code === */
    MS5607Update();     // Reads sensor, applies calibration, stored internally
    gs_temp_c      = (float)MS5607GetTemperatureC();
    gs_pressure_pa = (float)MS5607GetPressurePa();

    rx_rc = SX1262_ReceiveLora(rx_buf, sizeof(rx_buf), &rx_len, 1000);

    if (rx_rc == 0) {
        SX1262_GetPacketStatus(&pkt_status);

        if (rx_len == sizeof(rocket_telemetry_t)) {
            /* Cast the raw byte buffer directly to the packet struct.
             * Safe because rocket_telemetry_t is __attribute__((packed)) —
             * no padding bytes, so the memory layout is identical on both ends. */
            rocket_telemetry_t *p = (rocket_telemetry_t *)rx_buf;

            // Decode baro fields from the RX message
            float rkt_temp_c = p->temperature_cdeg / 100.0f; // convert from centi-degrees to degrees
            float rkt_press_pa = p->pressure_pa;

            // Calculate altitude AGL using the ground station as reference
            float altitude_m = calculate_altitude_m(rkt_press_pa, gs_pressure_pa, gs_temp_c);

            // Formatted print output
            printf("\r\n============================================\r\n");
            printf(" PKT #%-5u  T+%lu ms\r\n", p->packet_id, p->timestamp_ms);
            printf(" Signal_RSSI = %d dBm | RSSI = %d dBm | SNR = %d dB\r\n", pkt_status.signal_rssi, pkt_status.rssi_pkt, pkt_status.snr_pkt);
            printf("------------------------------------------------------------------------------\r\n");
            printf(" ALT      %8.1f m AGL\r\n", altitude_m);
            printf(" BARO RKT   %6.2f C    %7.0f Pa\r\n", rkt_temp_c, rkt_press_pa);
            printf(" BARO GND   %6.2f C    %7.0f Pa\r\n", gs_temp_c, gs_pressure_pa);
            printf(" ACCEL  X=%7.3f g   Y=%7.3f g   Z=%7.3f g\r\n",
                   p->ax / 2048.0f, p->ay / 2048.0f, p->az / 2048.0f);
            printf(" GYRO   X=%7.1f dps Y=%7.1f dps Z=%7.1f dps\r\n",
                   p->gx / 16.4f, p->gy / 16.4f, p->gz / 16.4f);
            printf(" GPS    lat=%.6f   lon=%.6f\r\n", p->gps_lat, p->gps_lon);
            printf("============================================\r\n");

            rx_pkt_count++;     // increment packet count for next packet's printout

        } else {
            /* Length mismatch — wrong transmitter, stale packet, or framing error.
             * Print raw hex so you can diagnose what actually arrived. */
            printf("  WARNING: got %u bytes, expected %u\r\n",
                   rx_len, (unsigned)sizeof(rocket_telemetry_t));
            printf("  Raw: ");
            for (i = 0; i < rx_len; i++) printf("%02X ", rx_buf[i]);
            printf("\r\n");
        }

    } else if (rx_rc == -1) {
        /* Timeout — no packet in the last second, print a heartbeat dot */
        printf(".");
        fflush(stdout); // ensure the dot appears immediately

    } else {
        /* rc == -2: CRC error */
        printf("\r\n[RX CRC ERROR] err=0x%04X\r\n", SX1262_GetDeviceErrors());
        SX1262_ClearDeviceErrors();
        SX1262_ClearIrqStatus(SX1262_IRQ_ALL);
    }

    /* Loop continues immediately to wait for the next packet */
  
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
