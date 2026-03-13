/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI2_CLK_Pin GPIO_PIN_5
#define SPI2_CLK_GPIO_Port GPIOA
#define SPI2_MISO_Pin GPIO_PIN_6
#define SPI2_MISO_GPIO_Port GPIOA
#define SPI2_MOSI_Pin GPIO_PIN_7
#define SPI2_MOSI_GPIO_Port GPIOA
#define LSM_INT2_Pin GPIO_PIN_4
#define LSM_INT2_GPIO_Port GPIOC
#define LSM_NCS_Pin GPIO_PIN_5
#define LSM_NCS_GPIO_Port GPIOC
#define LSM_INT1_Pin GPIO_PIN_0
#define LSM_INT1_GPIO_Port GPIOB
#define LIS_NCS_Pin GPIO_PIN_7
#define LIS_NCS_GPIO_Port GPIOE
#define ADX_NCS_Pin GPIO_PIN_15
#define ADX_NCS_GPIO_Port GPIOE
#define MS5_NCS_Pin GPIO_PIN_10
#define MS5_NCS_GPIO_Port GPIOB
#define ICM_NCS_Pin GPIO_PIN_12
#define ICM_NCS_GPIO_Port GPIOB
#define SPI_CLK_Pin GPIO_PIN_13
#define SPI_CLK_GPIO_Port GPIOB
#define SPI_MISO_Pin GPIO_PIN_14
#define SPI_MISO_GPIO_Port GPIOB
#define SPI_MOSI_Pin GPIO_PIN_15
#define SPI_MOSI_GPIO_Port GPIOB
#define GPS_NCS_Pin GPIO_PIN_8
#define GPS_NCS_GPIO_Port GPIOD
#define GPS_RST_Pin GPIO_PIN_9
#define GPS_RST_GPIO_Port GPIOD
#define E22_RESET_Pin GPIO_PIN_11
#define E22_RESET_GPIO_Port GPIOD
#define E22_DIO1_Pin GPIO_PIN_12
#define E22_DIO1_GPIO_Port GPIOD
#define E22_TXEN_Pin GPIO_PIN_13
#define E22_TXEN_GPIO_Port GPIOD
#define E22_RXEN_Pin GPIO_PIN_14
#define E22_RXEN_GPIO_Port GPIOD
#define E22_BUSY_Pin GPIO_PIN_15
#define E22_BUSY_GPIO_Port GPIOD
#define E22_NCS_Pin GPIO_PIN_6
#define E22_NCS_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
