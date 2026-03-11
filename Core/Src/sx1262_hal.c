/**
 * @file    sx1262_hal.c
 * @brief   Hardware abstraction implementation for the E22-900MM22S.
 */

#include "sx1262_hal.h"

void SX1262_HW_Init(void)
{
    SX1262_HW_NssHigh();
    SX1262_HW_SetTxEn(0);
    SX1262_HW_SetRxEn(0);
    SX1262_HW_Reset();
}

void SX1262_HW_Reset(void)
{
    HAL_GPIO_WritePin(SX1262_NRST_PORT, SX1262_NRST_PIN, GPIO_PIN_RESET);
    HAL_Delay(10);   /* 10 ms low pulse (matches working manual sequence) */
    HAL_GPIO_WritePin(SX1262_NRST_PORT, SX1262_NRST_PIN, GPIO_PIN_SET);
    HAL_Delay(100);  /* 100 ms for POR + TCXO settling */
    SX1262_HW_WaitBusy();
}

int SX1262_HW_WaitBusy(void)
{
    uint32_t start = HAL_GetTick();
    while (HAL_GPIO_ReadPin(SX1262_BUSY_PORT, SX1262_BUSY_PIN) == GPIO_PIN_SET) {
        if ((HAL_GetTick() - start) > SX1262_BUSY_TIMEOUT_MS) {
            return -1;
        }
    }
    return 0;
}

void SX1262_HW_NssLow(void)
{
    HAL_GPIO_WritePin(SX1262_NSS_PORT, SX1262_NSS_PIN, GPIO_PIN_RESET);
}

void SX1262_HW_NssHigh(void)
{
    HAL_GPIO_WritePin(SX1262_NSS_PORT, SX1262_NSS_PIN, GPIO_PIN_SET);
}

void SX1262_HW_DelayMs(uint32_t ms)
{
    HAL_Delay(ms);
}

uint8_t SX1262_HW_ReadDio1(void)
{
    return (HAL_GPIO_ReadPin(SX1262_DIO1_PORT, SX1262_DIO1_PIN) == GPIO_PIN_SET) ? 1 : 0;
}

void SX1262_HW_SetRxEn(uint8_t state)
{
    HAL_GPIO_WritePin(SX1262_RXEN_PORT, SX1262_RXEN_PIN,
                      state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void SX1262_HW_SetTxEn(uint8_t state)
{
    HAL_GPIO_WritePin(SX1262_TXEN_PORT, SX1262_TXEN_PIN,
                      state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
