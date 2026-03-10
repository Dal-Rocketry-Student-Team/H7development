/**
 * @file    sx1262_hal.h
 * @brief   Hardware abstraction for the E22-900MM22S (SX1262) module
 *          on the H7development PCB (STM32H723VGT6).
 *
 * Pin mapping (verified against schematic):
 *   SPI6  (PA5=SCK, PA6=MISO, PA7=MOSI)  — NOT A shared bus with LSM6DSV16X
 *   E22_NCS   → PC6   (GPIO output, active low chip select)
 *   E22_BUSY  → PD15  (GPIO input,  high = chip busy)
 *   E22_RXEN  → PD14  (GPIO output, RF-switch RX enable, active high)
 *   E22_TXEN  → PD13  (GPIO output, RF-switch TX enable, active high)
 *   E22_DIO1  → PD12  (GPIO input / EXTI, IRQ line)
 *   E22_RESET → PD11  (GPIO output, active low reset)
 *
 * CubeMX (.ioc) pin configuration:
 *     PC6  → GPIO_Output, label "E22_NCS",    init High, Push-Pull, High speed
 *     PD15 → GPIO_Input,  label "E22_BUSY",  No pull
 *     PD14 → GPIO_Output, label "E22_RXEN",  init Low,  Push-Pull
 *     PD13 → GPIO_Output, label "E22_TXEN",  init Low,  Push-Pull
 *     PD12 → GPIO_Input,  label "E22_DIO1",  No pull (or EXTI for interrupt)
 *     PD11 → GPIO_Output, label "E22_RESET", init High, Push-Pull
 */

#ifndef SX1262_HW_H
#define SX1262_HW_H

#include "main.h"   /* Brings in HAL + all GPIO label macros from CubeMX */
#include "spi.h"

/* ===================================================================
 * EDIT THIS SECTION to match your CubeMX GPIO labels / pin assignments.
 * The labels below must match what you set in your .ioc file.
 * =================================================================== */

/* SPI handle — SPI6 is the radio bus on the PCB */
#define SX1262_SPI_HANDLE       hspi6

/* Chip-select (directly from MCU, active low) */
#define SX1262_NSS_PORT         E22_NCS_GPIO_Port
#define SX1262_NSS_PIN          E22_NCS_Pin

/* Hardware reset (active low, PD11) */
#define SX1262_NRST_PORT        E22_RESET_GPIO_Port
#define SX1262_NRST_PIN         E22_RESET_Pin

/* BUSY output from the module (high = chip busy) */
#define SX1262_BUSY_PORT        E22_BUSY_GPIO_Port
#define SX1262_BUSY_PIN         E22_BUSY_Pin

/* DIO1 — general-purpose IRQ line (RxDone, TxDone, Timeout, etc.) */
#define SX1262_DIO1_PORT        E22_DIO1_GPIO_Port
#define SX1262_DIO1_PIN         E22_DIO1_Pin

/* RF-switch control pins on the E22 module
 * The E22-900MM22S has a TX/RX switch that needs TXEN and RXEN driven
 * by the MCU (or DIO2 can drive TXEN automatically via SetDio2AsRfSwitchCtrl).
 *
 * Your schematic shows TXEN and RXEN routed to MCU GPIOs.
 * We'll manage TXEN through DIO2 (automatic) and RXEN through a GPIO. */
#define SX1262_TXEN_PORT        E22_TXEN_GPIO_Port
#define SX1262_TXEN_PIN         E22_TXEN_Pin

#define SX1262_RXEN_PORT        E22_RXEN_GPIO_Port
#define SX1262_RXEN_PIN         E22_RXEN_Pin

/* ===================================================================
 * Timeout for BUSY-wait polling (milliseconds).
 * If BUSY doesn't go low within this time, something is very wrong.
 * =================================================================== */
#define SX1262_BUSY_TIMEOUT_MS  100

/* ===================================================================
 * Low-level HAL functions (implemented in sx1262_hw.c)
 * =================================================================== */

/**
 * @brief  Initialise the hardware lines (reset the module, etc.)
 *         Call once at startup BEFORE any SX1262 SPI commands.
 */
void SX1262_HW_Init(void);

/**
 * @brief  Hard-reset the SX1262 via the NRST pin.
 *         Holds NRST low for ≥100 µs then releases and waits for BUSY low.
 */
void SX1262_HW_Reset(void);

/**
 * @brief  Block until the BUSY pin goes low or timeout expires.
 * @retval 0 on success, -1 on timeout.
 */
int SX1262_HW_WaitBusy(void);

/**
 * @brief  Assert NSS low  (start SPI transaction).
 */
void SX1262_HW_NssLow(void);

/**
 * @brief  Deassert NSS high (end SPI transaction).
 */
void SX1262_HW_NssHigh(void);

/**
 * @brief  SPI write-then-read.
 * @param  txBuf  Data to clock out on MOSI.
 * @param  rxBuf  Buffer for data clocked in on MISO (may be NULL for write-only).
 * @param  len    Number of bytes.
 */
void SX1262_HW_SpiTransfer(const uint8_t *txBuf, uint8_t *rxBuf, uint16_t len);

/**
 * @brief  Millisecond delay wrapper.
 */
void SX1262_HW_DelayMs(uint32_t ms);

/**
 * @brief  Read the DIO1 pin state (1 = IRQ asserted).
 */
uint8_t SX1262_HW_ReadDio1(void);

/**
 * @brief  Control the RF-switch RXEN line.
 * @param  state  1 = enable RX path,  0 = disable.
 *
 * TXEN is controlled automatically by the SX1262 DIO2 pin
 * (via SetDio2AsRfSwitchCtrl).  RXEN must be toggled by firmware.
 */
void SX1262_HW_SetRxEn(uint8_t state);

/**
 * @brief  Control the RF-switch TXEN line manually (if needed).
 */
void SX1262_HW_SetTxEn(uint8_t state);

#endif /* SX1262_HW_H */
