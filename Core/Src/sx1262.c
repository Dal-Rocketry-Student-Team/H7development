/**
 * @file    sx1262.c
 * @brief   SX1262 driver implementation for the E22-900MM22S module.
 *
 * Follows the TX / RX initialisation sequences from the SX1261/2 Datasheet
 * Section 14.2 (TX) and Section 14.3 (RX), plus the E22 module-specific
 * requirements (DIO3 TCXO, external RF switch via TXEN/RXEN).
 *
 * Workarounds from Datasheet Section 15 are applied where noted.
 */

#include "sx1262.h"
#include "sx1262_hal.h"
#include <string.h>

/* ===================================================================
 * Internal helpers
 * =================================================================== */

/**
 * @brief Convert a timeout in microseconds to the SX1262 24-bit format.
 *        Resolution = 15.625 µs per tick (64 kHz RC clock).
 */
static inline uint32_t _us_to_ticks(uint32_t us)
{
    /* ticks = us / 15.625 = us * 64 / 1000 */
    return (uint64_t)us * 64 / 1000;
}

/* ===================================================================
 * Low-level SPI command helpers
 * =================================================================== */

void SX1262_WriteCommand(uint8_t opcode, const uint8_t *params, uint8_t len)
{
    SX1262_HW_WaitBusy();
    SX1262_HW_NssLow();

    /* Send opcode */
    uint8_t op = opcode;
    HAL_SPI_Transmit(&SX1262_SPI_HANDLE, &op, 1, HAL_MAX_DELAY);

    /* Send parameters */
    if (len > 0 && params != NULL) {
        HAL_SPI_Transmit(&SX1262_SPI_HANDLE, (uint8_t *)params, len, HAL_MAX_DELAY);
    }

    SX1262_HW_NssHigh();

    /* BUSY goes high after NSS rises for "write" commands; wait for it to drop */
    SX1262_HW_WaitBusy();
}

void SX1262_ReadCommand(uint8_t opcode, uint8_t *result, uint8_t len)
{
    SX1262_HW_WaitBusy();
    SX1262_HW_NssLow();

    /* Send opcode + 1 NOP for status — same pattern as your working IMU driver:
     * HAL_SPI_Transmit for the command, HAL_SPI_Receive for the response. */
    uint8_t header[2] = { opcode, 0x00 };
    HAL_SPI_Transmit(&SX1262_SPI_HANDLE, header, 2, HAL_MAX_DELAY);

    /* Now clock in the result bytes */
    if (len > 0 && result != NULL) {
        HAL_SPI_Receive(&SX1262_SPI_HANDLE, result, len, HAL_MAX_DELAY);
    }

    SX1262_HW_NssHigh();
}

void SX1262_WriteRegister(uint16_t addr, const uint8_t *data, uint8_t len)
{
    SX1262_HW_WaitBusy();
    SX1262_HW_NssLow();

    uint8_t header[3] = {
        SX1262_CMD_WRITE_REGISTER,
        (uint8_t)(addr >> 8),
        (uint8_t)(addr & 0xFF)
    };
    HAL_SPI_Transmit(&SX1262_SPI_HANDLE, header, 3, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&SX1262_SPI_HANDLE, (uint8_t *)data, len, HAL_MAX_DELAY);

    SX1262_HW_NssHigh();
    SX1262_HW_WaitBusy();
}

void SX1262_ReadRegister(uint16_t addr, uint8_t *data, uint8_t len)
{
    SX1262_HW_WaitBusy();
    SX1262_HW_NssLow();

    /* opcode + addr[15:8] + addr[7:0] + NOP(status) */
    uint8_t header[4] = {
        SX1262_CMD_READ_REGISTER,
        (uint8_t)(addr >> 8),
        (uint8_t)(addr & 0xFF),
        0x00  /* NOP */
    };
    HAL_SPI_Transmit(&SX1262_SPI_HANDLE, header, 4, HAL_MAX_DELAY);

    /* Clock in data bytes */
    HAL_SPI_Receive(&SX1262_SPI_HANDLE, data, len, HAL_MAX_DELAY);

    SX1262_HW_NssHigh();
}

void SX1262_WriteBuffer(uint8_t offset, const uint8_t *data, uint8_t len)
{
    SX1262_HW_WaitBusy();
    SX1262_HW_NssLow();

    uint8_t header[2] = { SX1262_CMD_WRITE_BUFFER, offset };
    HAL_SPI_Transmit(&SX1262_SPI_HANDLE, header, 2, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&SX1262_SPI_HANDLE, (uint8_t *)data, len, HAL_MAX_DELAY);

    SX1262_HW_NssHigh();
    SX1262_HW_WaitBusy();
}

void SX1262_ReadBuffer(uint8_t offset, uint8_t *data, uint8_t len)
{
    SX1262_HW_WaitBusy();
    SX1262_HW_NssLow();

    /* opcode + offset + NOP(status) */
    uint8_t header[3] = { SX1262_CMD_READ_BUFFER, offset, 0x00 };
    HAL_SPI_Transmit(&SX1262_SPI_HANDLE, header, 3, HAL_MAX_DELAY);

    /* Clock in data bytes */
    HAL_SPI_Receive(&SX1262_SPI_HANDLE, data, len, HAL_MAX_DELAY);

    SX1262_HW_NssHigh();
}

/* ===================================================================
 * Operational mode commands
 * =================================================================== */

void SX1262_SetStandby(sx1262_standby_t mode)
{
    uint8_t p = (uint8_t)mode;
    SX1262_WriteCommand(SX1262_CMD_SET_STANDBY, &p, 1);
}

void SX1262_SetSleep(uint8_t config)
{
    /* config bit 0: warm start (retain config), bit 2: RTC timeout wake */
    SX1262_WriteCommand(SX1262_CMD_SET_SLEEP, &config, 1);
}

void SX1262_SetFs(void)
{
    SX1262_WriteCommand(SX1262_CMD_SET_FS, NULL, 0);
}

void SX1262_SetTx(uint32_t timeout_us)
{
    uint32_t ticks = _us_to_ticks(timeout_us);
    uint8_t params[3] = {
        (uint8_t)((ticks >> 16) & 0xFF),
        (uint8_t)((ticks >>  8) & 0xFF),
        (uint8_t)( ticks        & 0xFF),
    };

    /* --- E22 RF-switch: TXEN=1 (TX path), RXEN=0 (not RX) --- */
    /* DIO2 is not connected on this PCB, so we drive both manually. */
    SX1262_HW_SetRxEn(0);
    SX1262_HW_SetTxEn(1);

    SX1262_WriteCommand(SX1262_CMD_SET_TX, params, 3);
}

void SX1262_SetRx(uint32_t timeout_us)
{
    uint32_t ticks;
    if (timeout_us == 0) {
        /* Continuous RX: special value 0xFFFFFF */
        ticks = 0xFFFFFF;
    } else {
        ticks = _us_to_ticks(timeout_us);
    }
    uint8_t params[3] = {
        (uint8_t)((ticks >> 16) & 0xFF),
        (uint8_t)((ticks >>  8) & 0xFF),
        (uint8_t)( ticks        & 0xFF),
    };

    /* --- E22 RF-switch: RXEN=1 (RX path), TXEN=0 (not TX) --- */
    /* DIO2 is not connected on this PCB, so we drive both manually. */
    SX1262_HW_SetTxEn(0);
    SX1262_HW_SetRxEn(1);

    SX1262_WriteCommand(SX1262_CMD_SET_RX, params, 3);
}

void SX1262_SetRxDutyCycle(uint32_t rx_period_us, uint32_t sleep_period_us)
{
    uint32_t rx_ticks = _us_to_ticks(rx_period_us);
    uint32_t sl_ticks = _us_to_ticks(sleep_period_us);
    uint8_t params[6] = {
        (uint8_t)((rx_ticks >> 16) & 0xFF),
        (uint8_t)((rx_ticks >>  8) & 0xFF),
        (uint8_t)( rx_ticks        & 0xFF),
        (uint8_t)((sl_ticks >> 16) & 0xFF),
        (uint8_t)((sl_ticks >>  8) & 0xFF),
        (uint8_t)( sl_ticks        & 0xFF),
    };
    SX1262_WriteCommand(SX1262_CMD_SET_RX_DUTY_CYCLE, params, 6);
}

void SX1262_SetCad(void)
{
    SX1262_WriteCommand(SX1262_CMD_SET_CAD, NULL, 0);
}

/* ===================================================================
 * Configuration commands
 * =================================================================== */

void SX1262_SetPacketType(sx1262_packet_type_t type)
{
    uint8_t p = (uint8_t)type;
    SX1262_WriteCommand(SX1262_CMD_SET_PACKET_TYPE, &p, 1);
}

void SX1262_SetRfFrequency(uint32_t freq_hz)
{
    /* RF_Freq register = freq_hz * 2^25 / F_XTAL (32 MHz)
     *                  = freq_hz * 33554432 / 32000000
     *                  = freq_hz * 2^25 / 2^25 * (2^25/32e6)
     * Simplified:       = (uint32_t)( (uint64_t)freq_hz * (1 << 25) / 32000000 ) */
    uint32_t rf_freq = (uint32_t)((uint64_t)freq_hz * (1UL << 25) / 32000000UL);
    uint8_t params[4] = {
        (uint8_t)((rf_freq >> 24) & 0xFF),
        (uint8_t)((rf_freq >> 16) & 0xFF),
        (uint8_t)((rf_freq >>  8) & 0xFF),
        (uint8_t)( rf_freq        & 0xFF),
    };
    SX1262_WriteCommand(SX1262_CMD_SET_RF_FREQUENCY, params, 4);
}

void SX1262_SetPaConfig(uint8_t pa_duty_cycle, uint8_t hp_max, uint8_t device_sel)
{
    uint8_t params[4] = { pa_duty_cycle, hp_max, device_sel, 0x01 /* paLut, always 0x01 */ };
    SX1262_WriteCommand(SX1262_CMD_SET_PA_CONFIG, params, 4);
}

void SX1262_SetTxParams(int8_t power_dbm, sx1262_ramp_time_t ramp)
{
    uint8_t params[2] = { (uint8_t)power_dbm, (uint8_t)ramp };
    SX1262_WriteCommand(SX1262_CMD_SET_TX_PARAMS, params, 2);
}

void SX1262_SetRegulatorMode(sx1262_regulator_t mode)
{
    uint8_t p = (uint8_t)mode;
    SX1262_WriteCommand(SX1262_CMD_SET_REGULATOR_MODE, &p, 1);
}

void SX1262_SetBufferBaseAddress(uint8_t tx_base, uint8_t rx_base)
{
    uint8_t params[2] = { tx_base, rx_base };
    SX1262_WriteCommand(SX1262_CMD_SET_BUFFER_BASE_ADDRESS, params, 2);
}

void SX1262_SetDio2AsRfSwitchCtrl(bool enable)
{
    uint8_t p = enable ? 0x01 : 0x00;
    SX1262_WriteCommand(SX1262_CMD_SET_DIO2_AS_RF_SWITCH, &p, 1);
}

void SX1262_SetDio3AsTcxoCtrl(sx1262_tcxo_voltage_t voltage, uint32_t delay_us)
{
    uint32_t ticks = _us_to_ticks(delay_us);
    uint8_t params[4] = {
        (uint8_t)voltage,
        (uint8_t)((ticks >> 16) & 0xFF),
        (uint8_t)((ticks >>  8) & 0xFF),
        (uint8_t)( ticks        & 0xFF),
    };
    SX1262_WriteCommand(SX1262_CMD_SET_DIO3_AS_TCXO_CTRL, params, 4);
}

void SX1262_SetRxTxFallbackMode(sx1262_fallback_t mode)
{
    uint8_t p = (uint8_t)mode;
    SX1262_WriteCommand(SX1262_CMD_SET_RX_TX_FALLBACK_MODE, &p, 1);
}

/* ===================================================================
 * LoRa configuration
 * =================================================================== */

void SX1262_SetLoRaModulationParams(const sx1262_lora_mod_t *mod)
{
    /* Datasheet Table 13-42: SetModulationParams always takes 8 parameter bytes.
     * For LoRa, only the first 4 are meaningful; the rest must be zero. */
    uint8_t params[8] = {
        (uint8_t)mod->sf,
        (uint8_t)mod->bw,
        (uint8_t)mod->cr,
        mod->ldro ? 0x01 : 0x00,
        0x00, 0x00, 0x00, 0x00,  /* ModParam5-8: unused for LoRa */
    };
    SX1262_WriteCommand(SX1262_CMD_SET_MODULATION_PARAMS, params, 8);
}

void SX1262_SetLoRaPacketParams(const sx1262_lora_pkt_t *pkt)
{
    /* Datasheet Table 13-51: SetPacketParams always takes 9 parameter bytes.
     * For LoRa, only the first 6 are meaningful; the rest must be zero. */
    uint8_t params[9] = {
        (uint8_t)((pkt->preamble_len >> 8) & 0xFF),
        (uint8_t)( pkt->preamble_len       & 0xFF),
        pkt->fixed_length ? 0x01 : 0x00,
        pkt->payload_len,
        pkt->crc_on       ? 0x01 : 0x00,
        pkt->invert_iq    ? 0x01 : 0x00,
        0x00, 0x00, 0x00,  /* PacketParam7-9: unused for LoRa */
    };
    SX1262_WriteCommand(SX1262_CMD_SET_PACKET_PARAMS, params, 9);
}

void SX1262_SetLoRaSyncWord(uint16_t sync_word)
{
    uint8_t sw[2] = {
        (uint8_t)((sync_word >> 8) & 0xFF),
        (uint8_t)( sync_word       & 0xFF),
    };
    SX1262_WriteRegister(SX1262_REG_LORA_SYNC_WORD_MSB, sw, 2);
}

void SX1262_SetLoRaSymbNumTimeout(uint8_t symb_num)
{
    SX1262_WriteCommand(SX1262_CMD_SET_LORA_SYMB_NUM_TIMEOUT, &symb_num, 1);
}

/* ===================================================================
 * IRQ
 * =================================================================== */

void SX1262_SetDioIrqParams(uint16_t irq_mask, uint16_t dio1_mask,
                            uint16_t dio2_mask, uint16_t dio3_mask)
{
    uint8_t params[8] = {
        (uint8_t)((irq_mask  >> 8) & 0xFF), (uint8_t)(irq_mask  & 0xFF),
        (uint8_t)((dio1_mask >> 8) & 0xFF), (uint8_t)(dio1_mask & 0xFF),
        (uint8_t)((dio2_mask >> 8) & 0xFF), (uint8_t)(dio2_mask & 0xFF),
        (uint8_t)((dio3_mask >> 8) & 0xFF), (uint8_t)(dio3_mask & 0xFF),
    };
    SX1262_WriteCommand(SX1262_CMD_SET_DIO_IRQ_PARAMS, params, 8);
}

uint16_t SX1262_GetIrqStatus(void)
{
    uint8_t buf[2] = {0};
    SX1262_ReadCommand(SX1262_CMD_GET_IRQ_STATUS, buf, 2);
    return ((uint16_t)buf[0] << 8) | buf[1];
}

void SX1262_ClearIrqStatus(uint16_t mask)
{
    uint8_t params[2] = {
        (uint8_t)((mask >> 8) & 0xFF),
        (uint8_t)( mask       & 0xFF),
    };
    SX1262_WriteCommand(SX1262_CMD_CLEAR_IRQ_STATUS, params, 2);
}

/* ===================================================================
 * Calibration
 * =================================================================== */

void SX1262_Calibrate(uint8_t calib_param)
{
    SX1262_WriteCommand(SX1262_CMD_CALIBRATE, &calib_param, 1);
    SX1262_HW_WaitBusy();  /* calibration runs while BUSY is high */
}

void SX1262_CalibrateImage(uint8_t freq1, uint8_t freq2)
{
    uint8_t params[2] = { freq1, freq2 };
    SX1262_WriteCommand(SX1262_CMD_CALIBRATE_IMAGE, params, 2);
}

/* ===================================================================
 * Status / diagnostics
 * =================================================================== */

uint8_t SX1262_GetStatus(void)
{
    uint8_t status = 0;
    SX1262_ReadCommand(SX1262_CMD_GET_STATUS, &status, 1);
    return status;
}

void SX1262_GetRxBufferStatus(uint8_t *payload_len, uint8_t *rx_start_ptr)
{
    uint8_t buf[2] = {0};
    SX1262_ReadCommand(SX1262_CMD_GET_RX_BUFFER_STATUS, buf, 2);
    *payload_len   = buf[0];
    *rx_start_ptr  = buf[1];
}

void SX1262_GetPacketStatus(sx1262_pkt_status_t *status)
{
    uint8_t buf[3] = {0};
    SX1262_ReadCommand(SX1262_CMD_GET_PACKET_STATUS, buf, 3);
    status->rssi_pkt    = -(int16_t)buf[0] / 2;
    status->snr_pkt     = (int8_t)buf[1] / 4;
    status->signal_rssi = -(int16_t)buf[2] / 2;
}

int16_t SX1262_GetRssiInst(void)
{
    uint8_t val = 0;
    SX1262_ReadCommand(SX1262_CMD_GET_RSSI_INST, &val, 1);
    return -(int16_t)val / 2;
}

uint16_t SX1262_GetDeviceErrors(void)
{
    uint8_t buf[2] = {0};
    SX1262_ReadCommand(SX1262_CMD_GET_DEVICE_ERRORS, buf, 2);
    return ((uint16_t)buf[0] << 8) | buf[1];
}

void SX1262_ClearDeviceErrors(void)
{
    uint8_t params[2] = { 0x00, 0x00 };
    SX1262_WriteCommand(SX1262_CMD_CLEAR_DEVICE_ERRORS, params, 2);
}

/* ===================================================================
 * Workarounds from Datasheet Section 15
 * =================================================================== */

/**
 * @brief  Section 15.1 — Modulation Quality with 500 kHz LoRa BW.
 *         Must be called after SetModulationParams with BW500.
 */
static void _workaround_mod_quality_500k(void)
{
    uint8_t val;
    SX1262_ReadRegister(0x0889, &val, 1);
    val &= 0xFB;  /* clear bit 2 */
    SX1262_WriteRegister(0x0889, &val, 1);
}

/**
 * @brief  Section 15.2 — Implicit Header Mode Timeout Behavior.
 *         Must be called before SetRx() when using implicit header.
 */
static void _workaround_implicit_header(void)
{
    uint8_t val;
    SX1262_ReadRegister(0x0920, &val, 1);
    val |= 0x1E;
    SX1262_WriteRegister(0x0920, &val, 1);
}

/**
 * @brief  Section 15.4 — Better Resistance of the SX1262 Tx to Antenna Mismatch.
 *         Call after every SetPaConfig().
 */
static void _workaround_tx_clamp(void)
{
    uint8_t val;
    SX1262_ReadRegister(SX1262_REG_TX_CLAMP_CONFIG, &val, 1);
    val |= 0x1E;  /* set bits [4:1] */
    SX1262_WriteRegister(SX1262_REG_TX_CLAMP_CONFIG, &val, 1);
}

/**
 * @brief  Section 15.3 — Optimizing the Inverted IQ Operation.
 *         Must be called when IQ is inverted (e.g. LoRaWAN downlink).
 */
static void _workaround_invert_iq(bool inverted)
{
    uint8_t val;
    SX1262_ReadRegister(0x0736, &val, 1);
    if (inverted) {
        val &= 0xFB;  /* clear bit 2 */
    } else {
        val |= 0x04;  /* set bit 2 */
    }
    SX1262_WriteRegister(0x0736, &val, 1);
}

/* ===================================================================
 * Full initialisation
 * =================================================================== */

int SX1262_Init(void)
{
    /* 1) Hardware reset */
    SX1262_HW_Init();

    /* After reset, chip is in STDBY_RC. BUSY goes low when ready. */

    /* 2) Set STDBY_RC explicitly (belt-and-suspenders) */
    SX1262_SetStandby(SX1262_STDBY_RC);

    /* 3) Configure DIO3 to power the 32 MHz TCXO on the E22 module.
     *    DIO3 pin 18 is NOT broken out on your PCB, but that's fine:
     *    the E22 module internally connects DIO3 to its TCXO on-module.
     *    This SPI command controls the internal SX1262 DIO3 output regardless
     *    of whether the pin is routed externally.
     *    From the Ebyte docs: "Use a DIO3 to power a 32MHz TCXO crystal internally."
     *    Delay: 10 ms = 10000 µs — generous time for TCXO to stabilise. */
    SX1262_SetDio3AsTcxoCtrl(SX1262_TCXO_1V8, 10000);

    /* 4) Transition to STDBY_XOSC to actually power up the TCXO via DIO3.
     *    In STDBY_RC the TCXO is NOT running — the chip only enables DIO3
     *    when it needs the 32 MHz clock (STDBY_XOSC, FS, TX, RX).
     *    We must wait here for the TCXO to start and stabilise. */
    SX1262_SetStandby(SX1262_STDBY_XOSC);
    SX1262_HW_DelayMs(15);  /* Extra margin on top of the TCXO internal delay */

    /* 5) Clear the XOSC_START_ERR that fires at POR with TCXO.
     *    At power-on the chip attempted auto-calibration before it knew
     *    a TCXO was present, so errors are expected.  Now that the TCXO
     *    is confirmed running, we clear errors and re-calibrate. */
    SX1262_ClearDeviceErrors();

    /* 6) Go back to STDBY_RC for calibration (required by datasheet —
     *    "The calibrate function starts ... in STDBY_RC mode"). */
    SX1262_SetStandby(SX1262_STDBY_RC);

    /* 7) Now calibrate everything with the TCXO properly configured.
     *    The chip will automatically enable the TCXO during calibration
     *    because SetDio3AsTcxoCtrl was already called.
     *    Bits: RC64k | RC13M | PLL | ADC_pulse | ADC_bulk_N | ADC_bulk_P | Image = 0x7F */
    SX1262_Calibrate(0x7F);
    SX1262_HW_DelayMs(5);   /* Calibration takes ~3.5 ms max */

    /* 8) Verify calibration succeeded — retry once if TCXO/PLL still failing */
    uint16_t init_errors = SX1262_GetDeviceErrors();
    if (init_errors & 0x0064) {
        /* XOSC_START_ERR (0x0020), PLL_CALIB_ERR (0x0004), or PLL_LOCK_ERR (0x0040)
         * still set.  Clear, re-init TCXO, and try again. */
        SX1262_ClearDeviceErrors();
        SX1262_SetDio3AsTcxoCtrl(SX1262_TCXO_1V8, 10000);
        SX1262_SetStandby(SX1262_STDBY_XOSC);
        SX1262_HW_DelayMs(20);
        SX1262_SetStandby(SX1262_STDBY_RC);
        SX1262_Calibrate(0x7F);
        SX1262_HW_DelayMs(5);
        SX1262_ClearDeviceErrors();
    }

    /* 9) Set regulator mode: DC-DC is more efficient (requires external inductor,
     *    which is present on the E22 module). */
    SX1262_SetRegulatorMode(SX1262_REGULATOR_DC_DC);

    /* 10) DIO2 (pin 19) is NOT connected on this PCB, so do NOT enable
     *     DIO2 as automatic RF switch control.  Instead, TXEN and RXEN are
     *     driven manually by the MCU (PD13 and PD14) in SetTx() and SetRx(). */
    SX1262_SetDio2AsRfSwitchCtrl(false);

    /* 11) Set buffer base addresses: TX at 0x00, RX at 0x80
     *     This gives 128 bytes each. Adjust if you need larger payloads. */
    SX1262_SetBufferBaseAddress(0x00, 0x80);

    /* 12) After TX/RX, fall back to STDBY_RC (default, saves power) */
    SX1262_SetRxTxFallbackMode(SX1262_FALLBACK_STDBY_RC);

    return 0;
}

/* ===================================================================
 * High-level convenience functions
 * =================================================================== */

void SX1262_ConfigureLora(uint32_t freq_hz,
                          const sx1262_lora_mod_t *mod,
                          const sx1262_lora_pkt_t *pkt)
{
    /* Ensure we're in STDBY_RC before configuring */
    SX1262_SetStandby(SX1262_STDBY_RC);

    /* Step 1: Packet type must be set first (Datasheet Section 14.5) */
    SX1262_SetPacketType(SX1262_PACKET_TYPE_LORA);

    /* Step 2: Set RF frequency */
    SX1262_SetRfFrequency(freq_hz);

    /* Step 3: Image calibration for the 902-928 MHz ISM band */
    SX1262_CalibrateImage(0xE1, 0xE9);

    /* Step 4: PA config for SX1262 at +22 dBm
     *   paDutyCycle = 0x04, hpMax = 0x07, deviceSel = 0x00 (SX1262) */
    SX1262_SetPaConfig(0x04, 0x07, 0x00);

    /* Apply TX clamp workaround (Section 15.4) */
    _workaround_tx_clamp();

    /* Step 5: Set TX power and ramp time */
    SX1262_SetTxParams(22, SX1262_RAMP_200_US);

    /* Step 6: Modulation parameters (must come before PacketParams) */
    SX1262_SetLoRaModulationParams(mod);

    /* 500 kHz BW workaround if applicable */
    if (mod->bw == SX1262_LORA_BW_500K) {
        _workaround_mod_quality_500k();
    }

    /* Step 7: Packet parameters */
    SX1262_SetLoRaPacketParams(pkt);

    /* Inverted IQ workaround */
    _workaround_invert_iq(pkt->invert_iq);

    /* Step 8: Sync word — private network (peer-to-peer) for rocketry */
    SX1262_SetLoRaSyncWord(SX1262_LORA_SYNC_WORD_PRIVATE);

    /* Step 9: Map TxDone, RxDone, Timeout, CrcErr to DIO1 */
    uint16_t irq_mask = SX1262_IRQ_TX_DONE | SX1262_IRQ_RX_DONE |
                        SX1262_IRQ_TIMEOUT | SX1262_IRQ_CRC_ERR;
    SX1262_SetDioIrqParams(irq_mask, irq_mask, 0x0000, 0x0000);
}

/* ------------------------------------------------------------------ */
int SX1262_TransmitLora(const uint8_t *data, uint8_t len, uint32_t timeout_ms)
{
    /* Write payload into TX buffer starting at offset 0x00 */
    SX1262_WriteBuffer(0x00, data, len);

    /* Clear any pending IRQs */
    SX1262_ClearIrqStatus(SX1262_IRQ_ALL);

    /* Enter TX mode; timeout in µs.
     * The SX1262 timeout is based on the 64 kHz RC clock.
     * 0 = no timeout (TX will run until done or error). */
    uint32_t timeout_us = (timeout_ms > 0) ? (timeout_ms * 1000UL) : 0;
    SX1262_SetTx(timeout_us);

    /* Poll for TxDone or Timeout */
    uint32_t start = HAL_GetTick();
    while (1) {
        uint16_t irq = SX1262_GetIrqStatus();

        if (irq & SX1262_IRQ_TX_DONE) {
            SX1262_ClearIrqStatus(SX1262_IRQ_ALL);
            SX1262_HW_SetTxEn(0);  /* RF switch off */
            SX1262_HW_SetRxEn(0);
            return 0;  /* success */
        }

        if (irq & SX1262_IRQ_TIMEOUT) {
            SX1262_ClearIrqStatus(SX1262_IRQ_ALL);
            SX1262_HW_SetTxEn(0);
            SX1262_HW_SetRxEn(0);
            return -1;  /* timeout */
        }

        /* Software watchdog in case IRQs never fire */
        if (timeout_ms > 0 && (HAL_GetTick() - start) > (timeout_ms + 500)) {
            SX1262_SetStandby(SX1262_STDBY_RC);
            SX1262_ClearIrqStatus(SX1262_IRQ_ALL);
            SX1262_HW_SetTxEn(0);
            SX1262_HW_SetRxEn(0);
            return -2;  /* error */
        }

        /* Yield — or just poll.  In an RTOS you'd wait on DIO1 EXTI instead. */
    }
}

/* ------------------------------------------------------------------ */
int SX1262_ReceiveLora(uint8_t *buf, uint8_t buf_size, uint8_t *rx_len,
                       uint32_t timeout_ms)
{
    /* Clear pending IRQs */
    SX1262_ClearIrqStatus(SX1262_IRQ_ALL);

    /* Enter RX mode */
    uint32_t timeout_us = (timeout_ms > 0) ? (timeout_ms * 1000UL) : 0;
    SX1262_SetRx(timeout_us);

    /* Poll for RxDone or Timeout */
    uint32_t start = HAL_GetTick();
    while (1) {
        uint16_t irq = SX1262_GetIrqStatus();

        if (irq & SX1262_IRQ_RX_DONE) {
            SX1262_ClearIrqStatus(SX1262_IRQ_ALL);
            SX1262_HW_SetTxEn(0);
            SX1262_HW_SetRxEn(0);

            /* Check CRC */
            if (irq & SX1262_IRQ_CRC_ERR) {
                *rx_len = 0;
                return -2;  /* CRC error */
            }

            /* Read payload */
            uint8_t pld_len, rx_start;
            SX1262_GetRxBufferStatus(&pld_len, &rx_start);

            if (pld_len > buf_size) pld_len = buf_size;
            SX1262_ReadBuffer(rx_start, buf, pld_len);
            *rx_len = pld_len;

            return 0;  /* success */
        }

        if (irq & SX1262_IRQ_TIMEOUT) {
            SX1262_ClearIrqStatus(SX1262_IRQ_ALL);
            SX1262_HW_SetTxEn(0);
            SX1262_HW_SetRxEn(0);
            *rx_len = 0;
            return -1;  /* timeout */
        }

        /* Software watchdog */
        if (timeout_ms > 0 && (HAL_GetTick() - start) > (timeout_ms + 500)) {
            SX1262_SetStandby(SX1262_STDBY_RC);
            SX1262_ClearIrqStatus(SX1262_IRQ_ALL);
            SX1262_HW_SetTxEn(0);
            SX1262_HW_SetRxEn(0);
            *rx_len = 0;
            return -2;
        }
    }
}
