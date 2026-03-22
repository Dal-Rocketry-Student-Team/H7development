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

#include "lsm6dsv16x_reg.h"   // LSM6DSV16X driver header file
#include "MadgwickAHRS.h"     // Madgwick AHRS algorithm header file
#include "MS5607SPI.h"        // MS5607 pressure sensor driver header file
#include "sx1262.h"           // SX1262 LoRa driver header file
#include "sx1262_hal.h"       // SX1262 hardware abstraction header file
#include "telemetry.h"        // telemetry packet definitions
#include "GPS.h"              // GPS parsing and circular buffer header file

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

// === Madgwick globals exposed by the library ===
extern volatile float q0, q1, q2, q3;      // quaternion (from Madgwick)
extern volatile float sampleFreq;          // Madgwick internal sample rate
static float gyro_bias_dps[3] = {0};       // boot-time gyro bias estimate

// === IMU variables ===
int16_t accel_raw[3] = {0}, gyro_raw[3] = {0};    // There are 3 axes of data for both the accelerometer and gyroscope, each a 16 bit value
float accel_g[3] = {0}, gyro_dps[3] = {0};
stmdev_ctx_t lsm6dsv16x_ctx;      // Making an instance of the ctx_t struct to use in accessing the lsm6dsv16x
lsm6dsv16x_data_ready_t drdy;     // data-ready flags to see if new data is available

// === Telemetry variables ===
static rocket_telemetry_t telem = {0};   // creating an instance of the telemetry payload
static uint16_t telem_pkt_id = 0;       // Rolling counter

// === GPS variables ===
static GPS_RMC_t last_gps_fix = {0};   // storage for the most recent GPS fix, to be included in telemetry
static GPS_RMC_t new_fix = {0};         // temporary storage for a newly popped GPS fix, to check if it's new before updating last_gps_fix
// === Debug display variables ===
// Declared at file scope (static storage, .bss) so they are not re-allocated
// on the stack every loop iteration.  All are recomputed from telem /
// last_gps_fix each iteration and used only in the UART printf block.
//
// IMU — integer engineering units (no floating-point printf required):
//   Accel: raw * 1000 / 2048  → milliG        (print as X.XXX g)
//   Gyro:  raw * 10   / 164   → tenths of dps  (print as X.X dps)
static int32_t  ax_mg,   ay_mg,   az_mg;          // accel in milli-g
static int32_t  gx_tdps, gy_tdps, gz_tdps;        // gyro in tenths-dps
// Baro
static int16_t  temp_cdeg;   // temperature in centi-degrees (copy of telem field)
static int16_t  temp_int;    // integer part of temperature in degrees
static uint16_t temp_frac;   // fractional part (always positive, 2 digits)
// GPS — sign/magnitude split for %lu.%07lu printing without %f
static int32_t  lat_raw, lon_raw;    // absolute value copies for digit extraction
static char     lat_hem, lon_hem;    // hemisphere characters ('N'/'S', 'E'/'W')
static uint32_t gps_t, gps_d;       // utc_time and utc_date copies for unpacking
static uint32_t spd_int, spd_frac;  // speed in m/s split at decimal point
static uint32_t hdg_int, hdg_frac;  // heading in degrees split at decimal point


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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  GPS_init();  // Initialize GPS (sets up UART receive interrupt)

  // TEMPORARY DIAGNOSTIC — remove after confirming UART reception
  // Polls USART1 for 5 seconds and dumps everything raw to Putty.
  // If you see garbage: baud rate mismatch.
  // If you see clean $GNRMC sentences with V: working UART, no satellite fix.
  // If you see nothing: wiring or CubeMX peripheral enable issue.
  {
      uint8_t b;
      uint32_t deadline = HAL_GetTick() + 5000;
      printf("\r\n--- RAW USART1 (5s) ---\r\n");
      while (HAL_GetTick() < deadline) {
          if (HAL_UART_Receive(&huart1, &b, 1, 10) == HAL_OK)
              HAL_UART_Transmit(&huart5, &b, 1, HAL_MAX_DELAY);
      }
      printf("\r\n--- END RAW ---\r\n");
      GPS_init(); // re-arm the interrupt after the polling diagnostic
  }

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_Base_Start_IT(&htim3);      // start periodic update IRQ

  // Setup lsm6dsv16x_ctx correctly for this device setup
  lsm6dsv16x_ctx.handle = &hspi2;
  lsm6dsv16x_ctx.mdelay = HAL_Delay;
  lsm6dsv16x_ctx.write_reg = platform_write;
  lsm6dsv16x_ctx.read_reg = platform_read;

  IMU_Init_LSM6DSV16X(&lsm6dsv16x_ctx);

  printf("\r\n=== SX1262 Continuous TX Bring-Up ===\r\n");

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
    printf("  → Status: 0x%02X | Cmd Status: 0x%X, Chip Mode: 0x%X\r\n", status, decoded.cmd_status, decoded.chip_mode);
  }

  // Give the radio some settling time
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
      .fixed_length = false,
      .payload_len  = sizeof(telem),
      .crc_on       = true,
      .invert_iq    = false,
  };

  SX1262_ConfigureLora(915000000UL, &mod, &pkt);
  printf("Radio configured: 915 MHz, SF9, BW125K, CR4/5, +22 dBm\r\n");
  
  {
    printf("SX1262 err code after CONFIGURE LORA: 0x%04X\r\n", SX1262_GetDeviceErrors());
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

  /* ==================== SELECT RADIO MODE ==================== */

  #define TX_MODE  1  /*0 = CW tone (easiest to see in SDR Sharp)
                        1 = continuous LoRa packets
                        2 = infinite LoRa preamble
                        3 = CAD mode (RX-ONLY)
                        4 = Radio disabled */

  /* ==================== SELECT RADIO MODE ==================== */

  #if TX_MODE == 0
    /* ======== CW TONE MODE ========
    * Emits an unmodulated carrier at 915 MHz.
    * In SDR Sharp you'll see a single spike at 915.000 MHz.
    * Great for verifying the RF path works at all.
    */
    printf("Starting CW tone at 915 MHz...\r\n");
    SX1262_SetStandby(SX1262_STDBY_RC);
    SX1262_SetPacketType(SX1262_PACKET_TYPE_LORA);
    SX1262_SetRfFrequency(915000000UL);
    SX1262_SetPaConfig(0x04, 0x07, 0x00);
    SX1262_SetTxParams(22, SX1262_RAMP_200_US);
    SX1262_SetTxContinuousWave();
    printf("CW active — check SDR Sharp at 915 MHz\r\n");
    /* CW stays on indefinitely — loop does nothing */

  #elif TX_MODE == 1
    /* ======== CONTINUOUS LORA PACKET MODE ========
    * Sends packets in a loop with a short delay between them.
    * In SDR Sharp you'll see periodic chirp bursts around 915 MHz.
    */
    int tx_rc = 0;

    printf("Starting continuous LoRa TX...\r\n");

  #elif TX_MODE == 2
    /* ======== INFINITE PREAMBLE MODE ========
    * Emits a continuous LoRa preamble (repeating upchirps).
    * In SDR Sharp you'll see a steady stream of chirps.
    */
    printf("Starting infinite preamble at 915 MHz...\r\n");
    SX1262_SetTxInfinitePreamble();
    printf("Preamble active — check SDR Sharp at 915 MHz\r\n");

  #elif TX_MODE == 3
    /* ======== CAD MODE (RX-ONLY) ========
    * Continuously performs Channel Activity Detection (CAD) at 915 MHz.
    * In SDR Sharp you'll see periodic short bursts as the radio briefly
      turns on its receiver to listen for activity, then goes back to sleep.
    */
    printf("Starting continuous CAD mode at 915 MHz...\r\n");
    SX1262_SetCad();
    printf("CAD active — check SDR Sharp at 915 MHz\r\n");
    /* CAD stays on indefinitely — loop does nothing */

  #elif TX_MODE == 4
    /* ======== RADIO OFF MODE ======== */
    printf("Turning radio off...\r\n");
    SX1262_SetStandby(SX1262_STDBY_RC);

  #endif

  printf("\r\n=== MS5607 Barometer Init ===\r\n");

  /* Setup MS5607 */
  MS5607StateTypeDef ms5607_status = MS5607_Init(&hspi1, MS5_NCS_GPIO_Port, MS5_NCS_Pin);
  if (ms5607_status == MS5607_STATE_FAILED) {
    printf("Barometer init failed!\r\n");
  } else {
    printf("Barometer initialized successfully\r\n");
  }

  /* Configure oversampling ratio */
  MS5607SetPressureOSR(OSR_4096);
  MS5607SetTemperatureOSR(OSR_4096);
    
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    
    #if TX_MODE == 1

      /* === IMU Code === */
      lsm6dsv16x_acceleration_raw_get(&lsm6dsv16x_ctx, accel_raw);
      lsm6dsv16x_angular_rate_raw_get(&lsm6dsv16x_ctx, gyro_raw);

      /* === Barometer Code === 
      GPS updates at ~1 Hz.  We poll every loop iteration and update
      last_gps_fix only when a new fix arrives; otherwise the previous
      fix is silently reused.  This keeps the loop fully non-blocking.*/
      MS5607Update();

      /* === GPS Code === */
      new_fix = (GPS_RMC_t){0};      // Clear temporary storage
      if (GPS_pop(&new_fix)) last_gps_fix = new_fix;  // Update last_gps_fix only if a new fix was popped

      /* === RADIO MODE BEHAVIOR === */
      // Telemetry struct packing
      telem.packet_id           = telem_pkt_id;           // assign current packet ID
      telem.timestamp_ms        = HAL_GetTick();          // simple timestamp in milliseconds since boot
      telem.ax                  = accel_raw[0];           // raw accel counts (not converted to g's for simplicity)
      telem.ay                  = accel_raw[1];           // raw accel counts
      telem.az                  = accel_raw[2];           // raw accel counts
      telem.gx                  = gyro_raw[0];            // raw gyro counts (not converted to dps for simplicity)
      telem.gy                  = gyro_raw[1];            // raw gyro counts  
      telem.gz                  = gyro_raw[2];                                  // raw gyro counts
      telem.temperature_cdeg    = (int16_t)(MS5607GetTemperatureC()*100);       // baro temp in centi-degrees C (e.g. 2315 = 23.15 °C)
      telem.pressure_pa         = MS5607GetPressurePa();                        // raw pressure in Pa
      telem.gps_lat             = last_gps_fix.lat_e7 / 1e7f;                   // gps converted to float degrees (e.g. 37.7749, which is positive for N latitude)
      telem.gps_lon             = last_gps_fix.lon_e7 / 1e7f;                   // gps converted to float degrees (e.g. -122.4194, which is negative for W longitude)
      telem.gps_utc_time        = last_gps_fix.utc_time;                        // gps utc time in hhmmss format, e.g. 231523 means 23:15:23 UTC
      telem.gps_utc_date        = last_gps_fix.utc_date;                        // gps utc date in ddmmyy format, e.g. 150623 means 15 June 2023
      telem.gps_speed_cms       = last_gps_fix.speed_cms;                       // gps speed in cm/s, e.g. 5144 means 51.44 cm/s
      telem.gps_course_cd       = last_gps_fix.course_cd;                       // gps course over ground in centidegrees, e.g. 12345 means 123.45°

      /* === Debug UART output === */
      /* Unpack GPS fields from integers for display — avoids %f and float printf */
      ax_mg   = (int32_t)telem.ax * 1000 / 2048;
      ay_mg   = (int32_t)telem.ay * 1000 / 2048;
      az_mg   = (int32_t)telem.az * 1000 / 2048;
      gx_tdps = (int32_t)telem.gx * 10 / 164;
      gy_tdps = (int32_t)telem.gy * 10 / 164;
      gz_tdps = (int32_t)telem.gz * 10 / 164;
 
      temp_cdeg = telem.temperature_cdeg;
      temp_int  = temp_cdeg / 100;
      temp_frac = (uint16_t)((temp_cdeg < 0 ? -temp_cdeg : temp_cdeg) % 100);
 
      lat_raw = last_gps_fix.lat_e7;
      lon_raw = last_gps_fix.lon_e7;
      lat_hem = (lat_raw >= 0) ? 'N' : 'S';
      lon_hem = (lon_raw >= 0) ? 'E' : 'W';
      if (lat_raw < 0) lat_raw = -lat_raw;
      if (lon_raw < 0) lon_raw = -lon_raw;
 
      gps_t    = last_gps_fix.utc_time;
      gps_d    = last_gps_fix.utc_date;
      spd_int  = last_gps_fix.speed_cms / 100;
      spd_frac = last_gps_fix.speed_cms % 100;
      hdg_int  = last_gps_fix.course_cd / 100;
      hdg_frac = last_gps_fix.course_cd % 100;

      tx_rc = SX1262_TransmitLora((uint8_t*)&telem, sizeof(telem), 1000);

      /* === Debug UART output ===
       * Layout mirrors the ground station receiver so both terminals look
       * consistent when monitoring a test flight side by side.
       * Border width: 44 chars to match ground station's === line. */
      printf("\r\n==========================================\r\n");
      printf(" PKT #%-5u  T+%lu ms   %u bytes\r\n",
             telem_pkt_id, telem.timestamp_ms, (unsigned)sizeof(telem));
      printf("------------------------------------------\r\n");
      printf(" ACCEL  X=%c%lu.%03lu g  Y=%c%lu.%03lu g  Z=%c%lu.%03lu g\r\n",
             ax_mg < 0 ? '-' : ' ', (unsigned long)(ax_mg < 0 ? -ax_mg : ax_mg) / 1000, (unsigned long)(ax_mg < 0 ? -ax_mg : ax_mg) % 1000,
             ay_mg < 0 ? '-' : ' ', (unsigned long)(ay_mg < 0 ? -ay_mg : ay_mg) / 1000, (unsigned long)(ay_mg < 0 ? -ay_mg : ay_mg) % 1000,
             az_mg < 0 ? '-' : ' ', (unsigned long)(az_mg < 0 ? -az_mg : az_mg) / 1000, (unsigned long)(az_mg < 0 ? -az_mg : az_mg) % 1000);
      printf(" GYRO   X=%c%lu.%01lu dps Y=%c%lu.%01lu dps Z=%c%lu.%01lu dps\r\n",
             gx_tdps < 0 ? '-' : ' ', (unsigned long)(gx_tdps < 0 ? -gx_tdps : gx_tdps) / 10, (unsigned long)(gx_tdps < 0 ? -gx_tdps : gx_tdps) % 10,
             gy_tdps < 0 ? '-' : ' ', (unsigned long)(gy_tdps < 0 ? -gy_tdps : gy_tdps) / 10, (unsigned long)(gy_tdps < 0 ? -gy_tdps : gy_tdps) % 10,
             gz_tdps < 0 ? '-' : ' ', (unsigned long)(gz_tdps < 0 ? -gz_tdps : gz_tdps) / 10, (unsigned long)(gz_tdps < 0 ? -gz_tdps : gz_tdps) % 10);
      printf(" BARO   %d.%02u C    %lu Pa\r\n",
             temp_int, temp_frac, (unsigned long)telem.pressure_pa);
      if (last_gps_fix.valid) {
          printf(" GPS    %lu.%07lu %c   %lu.%07lu %c\r\n",
                 (unsigned long)(lat_raw / 10000000), (unsigned long)(lat_raw % 10000000), lat_hem,
                 (unsigned long)(lon_raw / 10000000), (unsigned long)(lon_raw % 10000000), lon_hem);
          printf("        %02lu:%02lu:%02lu UTC   %02lu/%02lu/%02lu\r\n",
                 gps_t/10000, (gps_t%10000)/100, gps_t%100,
                 gps_d/10000, (gps_d%10000)/100, gps_d%100);
          printf("        %lu.%02lu m/s   %lu.%02lu deg\r\n",
                 spd_int, spd_frac, hdg_int, hdg_frac);
      } else {
          printf(" GPS    No fix\r\n");
      }

      if (tx_rc == 0) {
          printf("  TX    OK\r\n");
      } else {
          printf("  TX    FAIL (rc=%d, err=0x%04X)\r\n", tx_rc, SX1262_GetDeviceErrors());
          /* Try to recover */
          SX1262_ClearDeviceErrors();
          SX1262_ClearIrqStatus(SX1262_IRQ_ALL);
          SX1262_SetStandby(SX1262_STDBY_RC);
          HAL_Delay(100);
      }

      printf("==========================================\r\n");

      telem_pkt_id++;   // increment packet ID for next transmission

      HAL_Delay(500);  /* 500ms between packets — easy to see on SDR */
    
    #endif
  
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
