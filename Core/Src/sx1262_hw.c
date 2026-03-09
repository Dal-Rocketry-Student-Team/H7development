/**
 * @file    sx1262_hw.c
 * @brief   Hardware abstraction implementation for the E22-900MM22S.
 */

#include "sx1262_hw.h"

/* ------------------------------------------------------------------ */
void SX1262_HW_Init(void)
{
    /* Make sure NSS is high (idle) */
    SX1262_HW_NssHigh();

    /* RF-switch off */
    SX1262_HW_SetTxEn(0);
    SX1262_HW_SetRxEn(0);

    /* Hard-reset the module */
    SX1262_HW_Reset();
}

/* ------------------------------------------------------------------ */
void SX1262_HW_Reset(void)
{
    HAL_GPIO_WritePin(SX1262_NRST_PORT, SX1262_NRST_PIN, GPIO_PIN_RESET);
    HAL_Delay(1);   /* >100 µs — 1 ms is plenty */
    HAL_GPIO_WritePin(SX1262_NRST_PORT, SX1262_NRST_PIN, GPIO_PIN_SET);
    HAL_Delay(10);  /* allow the chip to boot (BUSY goes high then low) */
    SX1262_HW_WaitBusy();
}

/* ------------------------------------------------------------------ */
int SX1262_HW_WaitBusy(void)
{
    uint32_t start = HAL_GetTick();
    while (HAL_GPIO_ReadPin(SX1262_BUSY_PORT, SX1262_BUSY_PIN) == GPIO_PIN_SET) {
        if ((HAL_GetTick() - start) > SX1262_BUSY_TIMEOUT_MS) {
            return -1;  /* timed out — SX1262 stuck */
        }
    }
    return 0;
}

/* ------------------------------------------------------------------ */
void SX1262_HW_NssLow(void)
{
    HAL_GPIO_WritePin(SX1262_NSS_PORT, SX1262_NSS_PIN, GPIO_PIN_RESET);
}

/* ------------------------------------------------------------------ */
void SX1262_HW_NssHigh(void)
{
    HAL_GPIO_WritePin(SX1262_NSS_PORT, SX1262_NSS_PIN, GPIO_PIN_SET);
}

/* ------------------------------------------------------------------ */
void SX1262_HW_SpiTransfer(const uint8_t *txBuf, uint8_t *rxBuf, uint16_t len)
{
    if (rxBuf != NULL) {
        HAL_SPI_TransmitReceive(&SX1262_SPI_HANDLE, (uint8_t *)txBuf, rxBuf, len, HAL_MAX_DELAY);
    } else {
        HAL_SPI_Transmit(&SX1262_SPI_HANDLE, (uint8_t *)txBuf, len, HAL_MAX_DELAY);
    }
}

/* ------------------------------------------------------------------ */
void SX1262_HW_DelayMs(uint32_t ms)
{
    HAL_Delay(ms);
}

/* ------------------------------------------------------------------ */
uint8_t SX1262_HW_ReadDio1(void)
{
    return (HAL_GPIO_ReadPin(SX1262_DIO1_PORT, SX1262_DIO1_PIN) == GPIO_PIN_SET) ? 1 : 0;
}

/* ------------------------------------------------------------------ */
void SX1262_HW_SetRxEn(uint8_t state)
{
    HAL_GPIO_WritePin(SX1262_RXEN_PORT, SX1262_RXEN_PIN,
                      state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* ------------------------------------------------------------------ */
void SX1262_HW_SetTxEn(uint8_t state)
{
    HAL_GPIO_WritePin(SX1262_TXEN_PORT, SX1262_TXEN_PIN,
                      state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
