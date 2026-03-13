/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Bare-minimum SX1262 SPI debug
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

/* Bare SPI helper: send tx, receive rx, len bytes, with manual NSS */
static HAL_StatusTypeDef SX_SPI(uint8_t *tx, uint8_t *rx, uint16_t len)
{
    /* Wait for BUSY to go low */
    uint32_t t0 = HAL_GetTick();
    while (HAL_GPIO_ReadPin(E22_BUSY_GPIO_Port, E22_BUSY_Pin) == GPIO_PIN_SET) {
        if (HAL_GetTick() - t0 > 100) {
            printf("  !! BUSY stuck high\r\n");
            return HAL_TIMEOUT;
        }
    }
    HAL_GPIO_WritePin(E22_NCS_GPIO_Port, E22_NCS_Pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef rc = HAL_SPI_TransmitReceive(&hspi1, tx, rx, len, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(E22_NCS_GPIO_Port, E22_NCS_Pin, GPIO_PIN_SET);
    return rc;
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
  /* USER CODE BEGIN 2 */

  printf("\r\n\r\n====== SX1262 BARE MINIMUM DEBUG ======\r\n");

  /* ---- Fix SPI1: disable NSSP, software NSS, slow clock ---- */
  hspi1.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
  hspi1.Init.FifoThreshold     = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.NSS               = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128; /* 128/128 = 1 MHz */
  if (HAL_SPI_Init(&hspi1) != HAL_OK) {
      printf("SPI1 init FAILED\r\n");
      while(1) {}
  }
  printf("SPI1: NSSP=off, 1 MHz, NSS=soft\r\n");

  /* ---- Idle all control pins ---- */
  HAL_GPIO_WritePin(E22_NCS_GPIO_Port,   E22_NCS_Pin,   GPIO_PIN_SET);
  HAL_GPIO_WritePin(E22_TXEN_GPIO_Port,  E22_TXEN_Pin,  GPIO_PIN_RESET);
  HAL_GPIO_WritePin(E22_RXEN_GPIO_Port,  E22_RXEN_Pin,  GPIO_PIN_RESET);

  /* ---- Hardware reset ---- */
  printf("Resetting SX1262...\r\n");
  HAL_GPIO_WritePin(E22_RESET_GPIO_Port, E22_RESET_Pin, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(E22_RESET_GPIO_Port, E22_RESET_Pin, GPIO_PIN_SET);
  HAL_Delay(100);

  /* Check BUSY cleared */
  if (HAL_GPIO_ReadPin(E22_BUSY_GPIO_Port, E22_BUSY_Pin) == GPIO_PIN_SET) {
      printf("!! BUSY still HIGH after reset — chip may not be powered\r\n");
  } else {
      printf("BUSY=0 after reset (good)\r\n");
  }

  /* ---- GetStatus right after reset ---- */
  printf("\r\n--- GetStatus after reset ---\r\n");
  {
      uint8_t tx[2] = {0xC0, 0x00};
      uint8_t rx[2] = {0xAA, 0xAA};
      HAL_StatusTypeDef rc = SX_SPI(tx, rx, 2);
      printf("HAL=%d  rx: 0x%02X 0x%02X\r\n", rc, rx[0], rx[1]);
      printf("  Status byte = 0x%02X\r\n", rx[1]);
      uint8_t mode = (rx[1] >> 4) & 0x7;
      uint8_t cmd  = (rx[1] >> 1) & 0x7;
      printf("  Chip mode = %d ", mode);
      switch(mode) {
          case 2: printf("(STDBY_RC)"); break;
          case 3: printf("(STDBY_XOSC)"); break;
          case 4: printf("(FS)"); break;
          case 5: printf("(RX)"); break;
          case 6: printf("(TX)"); break;
          default: printf("(UNKNOWN)"); break;
      }
      printf("\r\n  Cmd status = %d ", cmd);
      switch(cmd) {
          case 2: printf("(data available)"); break;
          case 3: printf("(cmd timeout)"); break;
          case 4: printf("(cmd error)"); break;
          case 5: printf("(exec failure)"); break;
          case 6: printf("(TX done)"); break;
          default: printf("(reserved)"); break;
      }
      printf("\r\n");

      if (rx[0] == 0xAA && rx[1] == 0xAA)
          printf("  >> FAIL: rx buffer untouched by HAL\r\n");
      else if (rx[0] == 0x00 && rx[1] == 0x00)
          printf("  >> SUSPECT: all zeros — chip may not be responding\r\n");
      else if (mode == 2)
          printf("  >> OK: chip is in STDBY_RC as expected after reset\r\n");
      else
          printf("  >> UNEXPECTED: chip should be STDBY_RC after reset\r\n");
  }

  /* ---- SetStandby(STDBY_RC) then GetStatus ---- */
  printf("\r\n--- SetStandby(STDBY_RC) then GetStatus ---\r\n");
  {
      uint8_t tx[2] = {0x80, 0x00};
      uint8_t rx[2] = {0};
      SX_SPI(tx, rx, 2);
      printf("  During cmd: 0x%02X 0x%02X\r\n", rx[0], rx[1]);
  }
  HAL_Delay(5);
  {
      uint8_t tx[2] = {0xC0, 0x00};
      uint8_t rx[2] = {0};
      SX_SPI(tx, rx, 2);
      printf("  GetStatus:  0x%02X -> mode=%d cmd=%d\r\n",
             rx[1], (rx[1]>>4)&0x7, (rx[1]>>1)&0x7);
  }

  /* ---- GetDeviceErrors ---- */
  printf("\r\n--- GetDeviceErrors ---\r\n");
  {
      uint8_t tx[4] = {0x17, 0x00, 0x00, 0x00};
      uint8_t rx[4] = {0};
      SX_SPI(tx, rx, 4);
      uint16_t errs = ((uint16_t)rx[2] << 8) | rx[3];
      printf("  Raw: %02X %02X %02X %02X -> errors=0x%04X\r\n",
             rx[0], rx[1], rx[2], rx[3], errs);
  }

  /* ---- Register round-trip: write 0xAB to 0x0740, read back ---- */
  printf("\r\n--- Register round-trip (0xAB -> 0x0740) ---\r\n");
  {
      /* Set LoRa packet type first so 0x0740 is accessible */
      uint8_t tx[2] = {0x8A, 0x01};
      uint8_t rx[2] = {0};
      SX_SPI(tx, rx, 2);
  }
  HAL_Delay(2);
  {
      uint8_t tx[4] = {0x0D, 0x07, 0x40, 0xAB};
      uint8_t rx[4] = {0};
      SX_SPI(tx, rx, 4);
  }
  HAL_Delay(2);
  {
      uint8_t tx[5] = {0x1D, 0x07, 0x40, 0x00, 0x00};
      uint8_t rx[5] = {0};
      SX_SPI(tx, rx, 5);
      printf("  Read: %02X %02X %02X %02X [%02X]\r\n",
             rx[0], rx[1], rx[2], rx[3], rx[4]);
      printf("  Data = 0x%02X (expect 0xAB)\r\n", rx[4]);
      if (rx[4] == 0xAB)
          printf("  >> PASS\r\n");
      else
          printf("  >> FAIL\r\n");
  }

  printf("\r\n====== POLLING GetStatus every 1s ======\r\n\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
      uint8_t tx[2] = {0xC0, 0x00};
      uint8_t rx[2] = {0};
      SX_SPI(tx, rx, 2);
      printf("Status=0x%02X  mode=%d cmd=%d  BUSY=%d\r\n",
             rx[1], (rx[1]>>4)&0x7, (rx[1]>>1)&0x7,
             HAL_GPIO_ReadPin(E22_BUSY_GPIO_Port, E22_BUSY_Pin));
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
