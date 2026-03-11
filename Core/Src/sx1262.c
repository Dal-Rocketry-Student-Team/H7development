/**
 * @file    sx1262.c
 * @brief   SX1262 driver for the E22-900MM22S module.
 *
 * All SPI functions use HAL_SPI_TransmitReceive in a single call per
 * NSS assertion — the pattern proven to work on this STM32H7 + SPI1 setup.
 */

#include "sx1262.h"
#include "sx1262_hal.h"
#include <string.h>

static inline uint32_t _us_to_ticks(uint32_t us)
{
    return (uint64_t)us * 64 / 1000;
}

/* ===================================================================
 * Low-level SPI — single TransmitReceive per NSS assertion
 * =================================================================== */

void SX1262_WriteCommand(uint8_t opcode, const uint8_t *params, uint8_t len)
{
    uint8_t txBuf[258], rxBuf[258];
    uint16_t total = 1 + len;
    txBuf[0] = opcode;
    if (len > 0 && params != NULL) memcpy(&txBuf[1], params, len);

    SX1262_HW_WaitBusy();
    SX1262_HW_NssLow();
    HAL_SPI_TransmitReceive(&SX1262_SPI_HANDLE, txBuf, rxBuf, total, HAL_MAX_DELAY);
    SX1262_HW_NssHigh();
    SX1262_HW_WaitBusy();
}

void SX1262_ReadCommand(uint8_t opcode, uint8_t *result, uint8_t len)
{
    uint8_t txBuf[258], rxBuf[258];
    uint16_t total = 2 + len;
    memset(txBuf, 0x00, total);
    txBuf[0] = opcode;

    SX1262_HW_WaitBusy();
    SX1262_HW_NssLow();
    HAL_SPI_TransmitReceive(&SX1262_SPI_HANDLE, txBuf, rxBuf, total, HAL_MAX_DELAY);
    SX1262_HW_NssHigh();

    if (len > 0 && result != NULL) memcpy(result, &rxBuf[2], len);
}

void SX1262_WriteRegister(uint16_t addr, const uint8_t *data, uint8_t len)
{
    uint8_t txBuf[259], rxBuf[259];
    uint16_t total = 3 + len;
    txBuf[0] = SX1262_CMD_WRITE_REGISTER;
    txBuf[1] = (uint8_t)(addr >> 8);
    txBuf[2] = (uint8_t)(addr & 0xFF);
    memcpy(&txBuf[3], data, len);

    SX1262_HW_WaitBusy();
    SX1262_HW_NssLow();
    HAL_SPI_TransmitReceive(&SX1262_SPI_HANDLE, txBuf, rxBuf, total, HAL_MAX_DELAY);
    SX1262_HW_NssHigh();
    SX1262_HW_WaitBusy();
}

void SX1262_ReadRegister(uint16_t addr, uint8_t *data, uint8_t len)
{
    uint8_t txBuf[260], rxBuf[260];
    uint16_t total = 4 + len;
    memset(txBuf, 0x00, total);
    txBuf[0] = SX1262_CMD_READ_REGISTER;
    txBuf[1] = (uint8_t)(addr >> 8);
    txBuf[2] = (uint8_t)(addr & 0xFF);

    SX1262_HW_WaitBusy();
    SX1262_HW_NssLow();
    HAL_SPI_TransmitReceive(&SX1262_SPI_HANDLE, txBuf, rxBuf, total, HAL_MAX_DELAY);
    SX1262_HW_NssHigh();

    memcpy(data, &rxBuf[4], len);
}

void SX1262_WriteBuffer(uint8_t offset, const uint8_t *data, uint8_t len)
{
    uint8_t txBuf[258], rxBuf[258];
    uint16_t total = 2 + len;
    txBuf[0] = SX1262_CMD_WRITE_BUFFER;
    txBuf[1] = offset;
    memcpy(&txBuf[2], data, len);

    SX1262_HW_WaitBusy();
    SX1262_HW_NssLow();
    HAL_SPI_TransmitReceive(&SX1262_SPI_HANDLE, txBuf, rxBuf, total, HAL_MAX_DELAY);
    SX1262_HW_NssHigh();
    SX1262_HW_WaitBusy();
}

void SX1262_ReadBuffer(uint8_t offset, uint8_t *data, uint8_t len)
{
    uint8_t txBuf[259], rxBuf[259];
    uint16_t total = 3 + len;
    memset(txBuf, 0x00, total);
    txBuf[0] = SX1262_CMD_READ_BUFFER;
    txBuf[1] = offset;

    SX1262_HW_WaitBusy();
    SX1262_HW_NssLow();
    HAL_SPI_TransmitReceive(&SX1262_SPI_HANDLE, txBuf, rxBuf, total, HAL_MAX_DELAY);
    SX1262_HW_NssHigh();

    memcpy(data, &rxBuf[3], len);
}

/* ===================================================================
 * Operational mode commands
 * =================================================================== */

void SX1262_SetStandby(sx1262_standby_t mode) {
    uint8_t p = (uint8_t)mode;
    SX1262_WriteCommand(SX1262_CMD_SET_STANDBY, &p, 1);
}

void SX1262_SetSleep(uint8_t config) {
    SX1262_WriteCommand(SX1262_CMD_SET_SLEEP, &config, 1);
}

void SX1262_SetFs(void) {
    SX1262_WriteCommand(SX1262_CMD_SET_FS, NULL, 0);
}

void SX1262_SetTx(uint32_t timeout_us) {
    uint32_t ticks = _us_to_ticks(timeout_us);
    uint8_t params[3] = {
        (uint8_t)((ticks >> 16) & 0xFF),
        (uint8_t)((ticks >>  8) & 0xFF),
        (uint8_t)( ticks        & 0xFF),
    };
    SX1262_HW_SetRxEn(0);
    SX1262_HW_SetTxEn(1);
    SX1262_WriteCommand(SX1262_CMD_SET_TX, params, 3);
}

void SX1262_SetRx(uint32_t timeout_us) {
    uint32_t ticks = (timeout_us == 0) ? 0xFFFFFF : _us_to_ticks(timeout_us);
    uint8_t params[3] = {
        (uint8_t)((ticks >> 16) & 0xFF),
        (uint8_t)((ticks >>  8) & 0xFF),
        (uint8_t)( ticks        & 0xFF),
    };
    SX1262_HW_SetTxEn(0);
    SX1262_HW_SetRxEn(1);
    SX1262_WriteCommand(SX1262_CMD_SET_RX, params, 3);
}

void SX1262_SetRxDutyCycle(uint32_t rx_us, uint32_t sleep_us) {
    uint32_t rt = _us_to_ticks(rx_us), st = _us_to_ticks(sleep_us);
    uint8_t p[6] = {(rt>>16)&0xFF,(rt>>8)&0xFF,rt&0xFF,(st>>16)&0xFF,(st>>8)&0xFF,st&0xFF};
    SX1262_WriteCommand(SX1262_CMD_SET_RX_DUTY_CYCLE, p, 6);
}

void SX1262_SetCad(void) {
    SX1262_WriteCommand(SX1262_CMD_SET_CAD, NULL, 0);
}

void SX1262_SetTxContinuousWave(void) {
    SX1262_HW_SetRxEn(0);
    SX1262_HW_SetTxEn(1);
    SX1262_WriteCommand(SX1262_CMD_SET_TX_CONTINUOUS_WAVE, NULL, 0);
}

void SX1262_SetTxInfinitePreamble(void) {
    SX1262_HW_SetRxEn(0);
    SX1262_HW_SetTxEn(1);
    SX1262_WriteCommand(SX1262_CMD_SET_TX_INFINITE_PREAMBLE, NULL, 0);
}

/* ===================================================================
 * Configuration commands
 * =================================================================== */

void SX1262_SetPacketType(sx1262_packet_type_t type) {
    uint8_t p = (uint8_t)type;
    SX1262_WriteCommand(SX1262_CMD_SET_PACKET_TYPE, &p, 1);
}

void SX1262_SetRfFrequency(uint32_t freq_hz) {
    uint32_t rf = (uint32_t)((uint64_t)freq_hz * (1UL << 25) / 32000000UL);
    uint8_t p[4] = {(rf>>24)&0xFF,(rf>>16)&0xFF,(rf>>8)&0xFF,rf&0xFF};
    SX1262_WriteCommand(SX1262_CMD_SET_RF_FREQUENCY, p, 4);
}

void SX1262_SetPaConfig(uint8_t duty, uint8_t hp, uint8_t sel) {
    uint8_t p[4] = {duty, hp, sel, 0x01};
    SX1262_WriteCommand(SX1262_CMD_SET_PA_CONFIG, p, 4);
}

void SX1262_SetTxParams(int8_t power, sx1262_ramp_time_t ramp) {
    uint8_t p[2] = {(uint8_t)power, (uint8_t)ramp};
    SX1262_WriteCommand(SX1262_CMD_SET_TX_PARAMS, p, 2);
}

void SX1262_SetRegulatorMode(sx1262_regulator_t mode) {
    uint8_t p = (uint8_t)mode;
    SX1262_WriteCommand(SX1262_CMD_SET_REGULATOR_MODE, &p, 1);
}

void SX1262_SetBufferBaseAddress(uint8_t tx, uint8_t rx) {
    uint8_t p[2] = {tx, rx};
    SX1262_WriteCommand(SX1262_CMD_SET_BUFFER_BASE_ADDRESS, p, 2);
}

void SX1262_SetDio2AsRfSwitchCtrl(bool en) {
    uint8_t p = en ? 0x01 : 0x00;
    SX1262_WriteCommand(SX1262_CMD_SET_DIO2_AS_RF_SWITCH, &p, 1);
}

void SX1262_SetDio3AsTcxoCtrl(sx1262_tcxo_voltage_t v, uint32_t delay_us) {
    uint32_t t = _us_to_ticks(delay_us);
    uint8_t p[4] = {(uint8_t)v, (t>>16)&0xFF, (t>>8)&0xFF, t&0xFF};
    SX1262_WriteCommand(SX1262_CMD_SET_DIO3_AS_TCXO_CTRL, p, 4);
}

void SX1262_SetRxTxFallbackMode(sx1262_fallback_t mode) {
    uint8_t p = (uint8_t)mode;
    SX1262_WriteCommand(SX1262_CMD_SET_RX_TX_FALLBACK_MODE, &p, 1);
}

/* ===================================================================
 * LoRa config — full 8/9 byte params
 * =================================================================== */

void SX1262_SetLoRaModulationParams(const sx1262_lora_mod_t *mod) {
    uint8_t p[8] = {(uint8_t)mod->sf,(uint8_t)mod->bw,(uint8_t)mod->cr,
                     mod->ldro?0x01:0x00, 0,0,0,0};
    SX1262_WriteCommand(SX1262_CMD_SET_MODULATION_PARAMS, p, 8);
}

void SX1262_SetLoRaPacketParams(const sx1262_lora_pkt_t *pkt) {
    uint8_t p[9] = {(pkt->preamble_len>>8)&0xFF, pkt->preamble_len&0xFF,
                     pkt->fixed_length?0x01:0x00, pkt->payload_len,
                     pkt->crc_on?0x01:0x00, pkt->invert_iq?0x01:0x00, 0,0,0};
    SX1262_WriteCommand(SX1262_CMD_SET_PACKET_PARAMS, p, 9);
}

void SX1262_SetLoRaSyncWord(uint16_t sw) {
    uint8_t d[2] = {(sw>>8)&0xFF, sw&0xFF};
    SX1262_WriteRegister(SX1262_REG_LORA_SYNC_WORD_MSB, d, 2);
}

void SX1262_SetLoRaSymbNumTimeout(uint8_t n) {
    SX1262_WriteCommand(SX1262_CMD_SET_LORA_SYMB_NUM_TIMEOUT, &n, 1);
}

/* ===================================================================
 * IRQ
 * =================================================================== */

void SX1262_SetDioIrqParams(uint16_t irq, uint16_t d1, uint16_t d2, uint16_t d3) {
    uint8_t p[8] = {(irq>>8)&0xFF,irq&0xFF,(d1>>8)&0xFF,d1&0xFF,
                     (d2>>8)&0xFF,d2&0xFF,(d3>>8)&0xFF,d3&0xFF};
    SX1262_WriteCommand(SX1262_CMD_SET_DIO_IRQ_PARAMS, p, 8);
}

uint16_t SX1262_GetIrqStatus(void) {
    uint8_t b[2]={0};
    SX1262_ReadCommand(SX1262_CMD_GET_IRQ_STATUS, b, 2);
    return ((uint16_t)b[0]<<8)|b[1];
}

void SX1262_ClearIrqStatus(uint16_t mask) {
    uint8_t p[2] = {(mask>>8)&0xFF, mask&0xFF};
    SX1262_WriteCommand(SX1262_CMD_CLEAR_IRQ_STATUS, p, 2);
}

/* ===================================================================
 * Calibration
 * =================================================================== */

void SX1262_Calibrate(uint8_t c) {
    SX1262_WriteCommand(SX1262_CMD_CALIBRATE, &c, 1);
    SX1262_HW_WaitBusy();
}

void SX1262_CalibrateImage(uint8_t f1, uint8_t f2) {
    uint8_t p[2] = {f1, f2};
    SX1262_WriteCommand(SX1262_CMD_CALIBRATE_IMAGE, p, 2);
}

/* ===================================================================
 * Status
 * =================================================================== */

uint8_t SX1262_GetStatus(void) {
    uint8_t s=0;
    SX1262_ReadCommand(SX1262_CMD_GET_STATUS, &s, 1);
    return s;
}

void SX1262_GetRxBufferStatus(uint8_t *plen, uint8_t *ptr) {
    uint8_t b[2]={0};
    SX1262_ReadCommand(SX1262_CMD_GET_RX_BUFFER_STATUS, b, 2);
    *plen=b[0]; *ptr=b[1];
}

void SX1262_GetPacketStatus(sx1262_pkt_status_t *s) {
    uint8_t b[3]={0};
    SX1262_ReadCommand(SX1262_CMD_GET_PACKET_STATUS, b, 3);
    s->rssi_pkt=-(int16_t)b[0]/2; s->snr_pkt=(int8_t)b[1]/4; s->signal_rssi=-(int16_t)b[2]/2;
}

int16_t SX1262_GetRssiInst(void) {
    uint8_t v=0;
    SX1262_ReadCommand(SX1262_CMD_GET_RSSI_INST, &v, 1);
    return -(int16_t)v/2;
}

uint16_t SX1262_GetDeviceErrors(void) {
    uint8_t b[2]={0};
    SX1262_ReadCommand(SX1262_CMD_GET_DEVICE_ERRORS, b, 2);
    return ((uint16_t)b[0]<<8)|b[1];
}

void SX1262_ClearDeviceErrors(void) {
    uint8_t p[2]={0,0};
    SX1262_WriteCommand(SX1262_CMD_CLEAR_DEVICE_ERRORS, p, 2);
}

/* ===================================================================
 * Workarounds
 * =================================================================== */

static void _workaround_tx_clamp(void) {
    uint8_t v; SX1262_ReadRegister(SX1262_REG_TX_CLAMP_CONFIG,&v,1);
    v|=0x1E; SX1262_WriteRegister(SX1262_REG_TX_CLAMP_CONFIG,&v,1);
}

static void _workaround_invert_iq(bool inv) {
    uint8_t v; SX1262_ReadRegister(0x0736,&v,1);
    if(inv) v&=0xFB; else v|=0x04;
    SX1262_WriteRegister(0x0736,&v,1);
}

/* ===================================================================
 * Init
 * =================================================================== */

int SX1262_Init(void)
{
    SX1262_HW_Init();
    SX1262_SetStandby(SX1262_STDBY_RC);

    SX1262_SetDio3AsTcxoCtrl(SX1262_TCXO_1V8, 10000);
    SX1262_SetStandby(SX1262_STDBY_XOSC);
    SX1262_HW_DelayMs(15);
    SX1262_ClearDeviceErrors();

    SX1262_SetStandby(SX1262_STDBY_RC);
    SX1262_Calibrate(0x7F);
    SX1262_HW_DelayMs(5);

    uint16_t e = SX1262_GetDeviceErrors();
    if (e & 0x0064) {
        SX1262_ClearDeviceErrors();
        SX1262_SetDio3AsTcxoCtrl(SX1262_TCXO_1V8, 10000);
        SX1262_SetStandby(SX1262_STDBY_XOSC);
        SX1262_HW_DelayMs(20);
        SX1262_SetStandby(SX1262_STDBY_RC);
        SX1262_Calibrate(0x7F);
        SX1262_HW_DelayMs(5);
        SX1262_ClearDeviceErrors();
    }

    SX1262_SetRegulatorMode(SX1262_REGULATOR_DC_DC);
    SX1262_SetDio2AsRfSwitchCtrl(false);
    SX1262_SetBufferBaseAddress(0x00, 0x80);
    SX1262_SetRxTxFallbackMode(SX1262_FALLBACK_STDBY_RC);

    /* Apply TX clamp workaround (datasheet errata — prevents sub-optimal PA) */
    _workaround_tx_clamp();

    return 0;
}

/* ===================================================================
 * High-level
 * =================================================================== */

void SX1262_ConfigureLora(uint32_t freq_hz,
                          const sx1262_lora_mod_t *mod,
                          const sx1262_lora_pkt_t *pkt)
{
    SX1262_SetStandby(SX1262_STDBY_RC);
    SX1262_SetPacketType(SX1262_PACKET_TYPE_LORA);
    SX1262_SetRfFrequency(freq_hz);
    SX1262_CalibrateImage(0xE1, 0xE9);
    SX1262_SetPaConfig(0x04, 0x07, 0x00);
    _workaround_tx_clamp();
    SX1262_SetTxParams(22, SX1262_RAMP_200_US);
    SX1262_SetLoRaModulationParams(mod);
    SX1262_SetLoRaPacketParams(pkt);
    _workaround_invert_iq(pkt->invert_iq);
    SX1262_SetLoRaSyncWord(SX1262_LORA_SYNC_WORD_PRIVATE);

    uint16_t irq = SX1262_IRQ_TX_DONE|SX1262_IRQ_RX_DONE|SX1262_IRQ_TIMEOUT|SX1262_IRQ_CRC_ERR;
    SX1262_SetDioIrqParams(irq, irq, 0, 0);
}

int SX1262_TransmitLora(const uint8_t *data, uint8_t len, uint32_t timeout_ms)
{
    SX1262_WriteBuffer(0x00, data, len);
    SX1262_ClearIrqStatus(SX1262_IRQ_ALL);

    uint32_t timeout_us = timeout_ms > 0 ? timeout_ms * 1000UL : 0;
    SX1262_SetTx(timeout_us);

    uint32_t start = HAL_GetTick();
    while (1) {
        uint16_t irq = SX1262_GetIrqStatus();
        if (irq & SX1262_IRQ_TX_DONE) {
            SX1262_ClearIrqStatus(SX1262_IRQ_ALL);
            SX1262_HW_SetTxEn(0); SX1262_HW_SetRxEn(0);
            return 0;
        }
        if (irq & SX1262_IRQ_TIMEOUT) {
            SX1262_ClearIrqStatus(SX1262_IRQ_ALL);
            SX1262_HW_SetTxEn(0); SX1262_HW_SetRxEn(0);
            return -1;
        }
        if (timeout_ms > 0 && (HAL_GetTick()-start) > (timeout_ms+500)) {
            SX1262_SetStandby(SX1262_STDBY_RC);
            SX1262_ClearIrqStatus(SX1262_IRQ_ALL);
            SX1262_HW_SetTxEn(0); SX1262_HW_SetRxEn(0);
            return -2;
        }
    }
}

int SX1262_ReceiveLora(uint8_t *buf, uint8_t buf_size, uint8_t *rx_len,
                       uint32_t timeout_ms)
{
    SX1262_ClearIrqStatus(SX1262_IRQ_ALL);
    uint32_t timeout_us = timeout_ms > 0 ? timeout_ms * 1000UL : 0;
    SX1262_SetRx(timeout_us);

    uint32_t start = HAL_GetTick();
    while (1) {
        uint16_t irq = SX1262_GetIrqStatus();
        if (irq & SX1262_IRQ_RX_DONE) {
            SX1262_ClearIrqStatus(SX1262_IRQ_ALL);
            SX1262_HW_SetTxEn(0); SX1262_HW_SetRxEn(0);
            if (irq & SX1262_IRQ_CRC_ERR) { *rx_len=0; return -2; }
            uint8_t pl,rs;
            SX1262_GetRxBufferStatus(&pl,&rs);
            if(pl>buf_size) pl=buf_size;
            SX1262_ReadBuffer(rs,buf,pl);
            *rx_len=pl;
            return 0;
        }
        if (irq & SX1262_IRQ_TIMEOUT) {
            SX1262_ClearIrqStatus(SX1262_IRQ_ALL);
            SX1262_HW_SetTxEn(0); SX1262_HW_SetRxEn(0);
            *rx_len=0; return -1;
        }
        if (timeout_ms > 0 && (HAL_GetTick()-start) > (timeout_ms+500)) {
            SX1262_SetStandby(SX1262_STDBY_RC);
            SX1262_ClearIrqStatus(SX1262_IRQ_ALL);
            SX1262_HW_SetTxEn(0); SX1262_HW_SetRxEn(0);
            *rx_len=0; return -2;
        }
    }
}
