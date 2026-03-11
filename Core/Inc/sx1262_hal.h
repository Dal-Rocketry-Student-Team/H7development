/**
 * @file    sx1262_hal.h
 * @brief   Hardware abstraction for the E22-900MM22S (SX1262) module
 *          on the H7development PCB (STM32H723VGT6).
 *
 * Pin mapping (verified against schematic):
 *   SPI1  (PA5=SCK, PA6=MISO, PA7=MOSI) — dedicated radio bus
 *   E22_NCS   → PC6   (GPIO output, active low chip select)
 *   E22_BUSY  → PD15  (GPIO input,  high = chip busy)
 *   E22_RXEN  → PD14  (GPIO output, RF-switch RX enable, active high)
 *   E22_TXEN  → PD13  (GPIO output, RF-switch TX enable, active high)
 *   E22_DIO1  → PD12  (GPIO input / EXTI, IRQ line)
 *   E22_RESET → PD11  (GPIO output, active low reset)
 *
 * NOTE: PA5 (SCK) requires internal pull-down in CubeMX for correct CPOL=0 idle state.
 */

#ifndef SX1262_HAL_H
#define SX1262_HAL_H

#include "main.h"
#include "spi.h"

/* SPI handle — SPI1 on PA5/PA6/PA7 with pull-down on SCK */
#define SX1262_SPI_HANDLE       hspi1

/* Chip-select */
#define SX1262_NSS_PORT         E22_NCS_GPIO_Port
#define SX1262_NSS_PIN          E22_NCS_Pin

/* Hardware reset */
#define SX1262_NRST_PORT        E22_RESET_GPIO_Port
#define SX1262_NRST_PIN         E22_RESET_Pin

/* BUSY */
#define SX1262_BUSY_PORT        E22_BUSY_GPIO_Port
#define SX1262_BUSY_PIN         E22_BUSY_Pin

/* DIO1 IRQ */
#define SX1262_DIO1_PORT        E22_DIO1_GPIO_Port
#define SX1262_DIO1_PIN         E22_DIO1_Pin

/* RF switch */
#define SX1262_TXEN_PORT        E22_TXEN_GPIO_Port
#define SX1262_TXEN_PIN         E22_TXEN_Pin
#define SX1262_RXEN_PORT        E22_RXEN_GPIO_Port
#define SX1262_RXEN_PIN         E22_RXEN_Pin

/* BUSY timeout */
#define SX1262_BUSY_TIMEOUT_MS  100

/* HAL functions */
void    SX1262_HW_Init(void);
void    SX1262_HW_Reset(void);
int     SX1262_HW_WaitBusy(void);
void    SX1262_HW_NssLow(void);
void    SX1262_HW_NssHigh(void);
void    SX1262_HW_DelayMs(uint32_t ms);
uint8_t SX1262_HW_ReadDio1(void);
void    SX1262_HW_SetRxEn(uint8_t state);
void    SX1262_HW_SetTxEn(uint8_t state);

#endif /* SX1262_HAL_H */
