/**
 * @file    sx1262.h
 * @brief   SX1262 LoRa/FSK driver for the E22-900MM22S module.
 *
 * Based on the Semtech SX1261/2 Datasheet DS.SX1261-2.W.APP Rev 2.2 (Dec 2024)
 * and the Ebyte E22-900MM22S User Manual v1.0.
 *
 * Key hardware notes for your E22-900MM22S:
 *   - SX1262 die (high-power PA, +22 dBm capable)
 *   - 32 MHz TCXO powered by DIO3 internally on the module
 *   - External RF switch: TXEN (pin 9) and RXEN (pin 10)
 *   - SPI CPOL=0, CPHA=0, up to 16 MHz SCK
 *   - 256-byte FIFO for TX and RX
 */

#ifndef SX1262_H
#define SX1262_H

#include <stdint.h>
#include <stdbool.h>

/* ===================================================================
 * SPI Command Opcodes  (Datasheet Section 13)
 * =================================================================== */

/* Operational modes */
#define SX1262_CMD_SET_SLEEP                0x84
#define SX1262_CMD_SET_STANDBY              0x80
#define SX1262_CMD_SET_FS                   0xC1
#define SX1262_CMD_SET_TX                   0x83
#define SX1262_CMD_SET_RX                   0x82
#define SX1262_CMD_STOP_TIMER_ON_PREAMBLE   0x9F
#define SX1262_CMD_SET_RX_DUTY_CYCLE        0x94
#define SX1262_CMD_SET_CAD                  0xC5
#define SX1262_CMD_SET_TX_CONTINUOUS_WAVE    0xD1
#define SX1262_CMD_SET_TX_INFINITE_PREAMBLE 0xD2

/* Register / buffer access */
#define SX1262_CMD_WRITE_REGISTER           0x0D
#define SX1262_CMD_READ_REGISTER            0x1D
#define SX1262_CMD_WRITE_BUFFER             0x0E
#define SX1262_CMD_READ_BUFFER              0x1E

/* DIO / IRQ */
#define SX1262_CMD_SET_DIO_IRQ_PARAMS       0x08
#define SX1262_CMD_GET_IRQ_STATUS           0x12
#define SX1262_CMD_CLEAR_IRQ_STATUS         0x02
#define SX1262_CMD_SET_DIO2_AS_RF_SWITCH    0x9D
#define SX1262_CMD_SET_DIO3_AS_TCXO_CTRL    0x97

/* RF / packet configuration */
#define SX1262_CMD_SET_RF_FREQUENCY         0x86
#define SX1262_CMD_SET_PACKET_TYPE          0x8A
#define SX1262_CMD_GET_PACKET_TYPE          0x11
#define SX1262_CMD_SET_TX_PARAMS            0x8E
#define SX1262_CMD_SET_MODULATION_PARAMS    0x8B
#define SX1262_CMD_SET_PACKET_PARAMS        0x8C
#define SX1262_CMD_SET_CAD_PARAMS           0x88
#define SX1262_CMD_SET_BUFFER_BASE_ADDRESS  0x8F
#define SX1262_CMD_SET_LORA_SYMB_NUM_TIMEOUT 0xA0

/* Power */
#define SX1262_CMD_SET_PA_CONFIG            0x95
#define SX1262_CMD_SET_REGULATOR_MODE       0x96
#define SX1262_CMD_SET_RX_TX_FALLBACK_MODE  0x93

/* Calibration */
#define SX1262_CMD_CALIBRATE                0x89
#define SX1262_CMD_CALIBRATE_IMAGE          0x98

/* Status */
#define SX1262_CMD_GET_STATUS               0xC0
#define SX1262_CMD_GET_RX_BUFFER_STATUS     0x13
#define SX1262_CMD_GET_PACKET_STATUS        0x14
#define SX1262_CMD_GET_RSSI_INST            0x15
#define SX1262_CMD_GET_STATS                0x10
#define SX1262_CMD_RESET_STATS              0x00
#define SX1262_CMD_GET_DEVICE_ERRORS        0x17
#define SX1262_CMD_CLEAR_DEVICE_ERRORS      0x07

/* ===================================================================
 * Important register addresses  (Datasheet Section 12)
 * =================================================================== */
#define SX1262_REG_LORA_SYNC_WORD_MSB   0x0740
#define SX1262_REG_LORA_SYNC_WORD_LSB   0x0741
#define SX1262_REG_RX_GAIN              0x08AC
#define SX1262_REG_OCP                  0x08E7   /* Over-current protection */
#define SX1262_REG_TX_CLAMP_CONFIG      0x08D8   /* PA clamping workaround */

/* Sync word presets */
#define SX1262_LORA_SYNC_WORD_PUBLIC    0x3444   /* LoRaWAN public network */
#define SX1262_LORA_SYNC_WORD_PRIVATE   0x1424   /* Private / peer-to-peer */

/*  Ground station FIFO split — inverse of the rocket:
 *  RX region: 0x00–0xD7  (216 bytes) — large downlink telemetry, rocket → ground
 *  TX region: 0xD8–0xFF  ( 40 bytes) — small uplink commands,   ground → rocket
 */
#define SX1262_RX_BASE   0x00u
#define SX1262_TX_BASE   0xD8u

/* ===================================================================
 * Enumerations
 * =================================================================== */

/** Standby clock source */
typedef enum {
    SX1262_STDBY_RC   = 0x00,   /**< 13 MHz RC oscillator */
    SX1262_STDBY_XOSC = 0x01,   /**< 32 MHz XTAL / TCXO  */
} sx1262_standby_t;

/** Packet type (modem selection) */
typedef enum {
    SX1262_PACKET_TYPE_GFSK  = 0x00,
    SX1262_PACKET_TYPE_LORA  = 0x01,
    SX1262_PACKET_TYPE_LR_FHSS = 0x03,
} sx1262_packet_type_t;

/** Regulator mode */
typedef enum {
    SX1262_REGULATOR_LDO     = 0x00,   /**< LDO only (default, higher current) */
    SX1262_REGULATOR_DC_DC   = 0x01,   /**< DC-DC + LDO (more efficient)        */
} sx1262_regulator_t;

/** Fallback mode after TX/RX */
typedef enum {
    SX1262_FALLBACK_STDBY_RC   = 0x20,
    SX1262_FALLBACK_STDBY_XOSC = 0x30,
    SX1262_FALLBACK_FS         = 0x40,
} sx1262_fallback_t;

/** TCXO voltage (for DIO3) */
typedef enum {
    SX1262_TCXO_1V6 = 0x00,
    SX1262_TCXO_1V7 = 0x01,
    SX1262_TCXO_1V8 = 0x02,
    SX1262_TCXO_2V2 = 0x03,
    SX1262_TCXO_2V4 = 0x04,
    SX1262_TCXO_2V7 = 0x05,
    SX1262_TCXO_3V0 = 0x06,
    SX1262_TCXO_3V3 = 0x07,
} sx1262_tcxo_voltage_t;

/** PA ramp time */
typedef enum {
    SX1262_RAMP_10_US   = 0x00,
    SX1262_RAMP_20_US   = 0x01,
    SX1262_RAMP_40_US   = 0x02,
    SX1262_RAMP_80_US   = 0x03,
    SX1262_RAMP_200_US  = 0x04,
    SX1262_RAMP_800_US  = 0x05,
    SX1262_RAMP_1700_US = 0x06,
    SX1262_RAMP_3400_US = 0x07,
} sx1262_ramp_time_t;

/* --- LoRa modulation parameters --- */

typedef enum {
    SX1262_LORA_SF5  = 0x05,
    SX1262_LORA_SF6  = 0x06,
    SX1262_LORA_SF7  = 0x07,
    SX1262_LORA_SF8  = 0x08,
    SX1262_LORA_SF9  = 0x09,
    SX1262_LORA_SF10 = 0x0A,
    SX1262_LORA_SF11 = 0x0B,
    SX1262_LORA_SF12 = 0x0C,
} sx1262_lora_sf_t;

typedef enum {
    SX1262_LORA_BW_7K8   = 0x00,
    SX1262_LORA_BW_10K4  = 0x08,
    SX1262_LORA_BW_15K6  = 0x01,
    SX1262_LORA_BW_20K8  = 0x09,
    SX1262_LORA_BW_31K25 = 0x02,
    SX1262_LORA_BW_41K7  = 0x0A,
    SX1262_LORA_BW_62K5  = 0x03,
    SX1262_LORA_BW_125K  = 0x04,
    SX1262_LORA_BW_250K  = 0x05,
    SX1262_LORA_BW_500K  = 0x06,
} sx1262_lora_bw_t;

typedef enum {
    SX1262_LORA_CR_4_5    = 0x01,
    SX1262_LORA_CR_4_6    = 0x02,
    SX1262_LORA_CR_4_7    = 0x03,
    SX1262_LORA_CR_4_8    = 0x04,
    SX1262_LORA_CR_4_5_LI = 0x05,  /**< Long interleaver variants */
    SX1262_LORA_CR_4_6_LI = 0x06,
    SX1262_LORA_CR_4_8_LI = 0x07,
} sx1262_lora_cr_t;

/* --- IRQ flags (bitmask, Table 8-4 / 13-29) --- */

#define SX1262_IRQ_TX_DONE              (1 << 0)
#define SX1262_IRQ_RX_DONE              (1 << 1)
#define SX1262_IRQ_PREAMBLE_DETECTED    (1 << 2)
#define SX1262_IRQ_SYNC_WORD_VALID      (1 << 3)
#define SX1262_IRQ_HEADER_VALID         (1 << 4)
#define SX1262_IRQ_HEADER_ERR           (1 << 5)
#define SX1262_IRQ_CRC_ERR              (1 << 6)
#define SX1262_IRQ_CAD_DONE             (1 << 7)
#define SX1262_IRQ_CAD_DETECTED         (1 << 8)
#define SX1262_IRQ_TIMEOUT              (1 << 9)
#define SX1262_IRQ_ALL                  0x03FF

/* ===================================================================
 * Configuration structures
 * =================================================================== */

/** LoRa modulation config */
typedef struct {
    sx1262_lora_sf_t  sf;           /**< Spreading factor SF5..SF12           */
    sx1262_lora_bw_t  bw;           /**< Bandwidth                            */
    sx1262_lora_cr_t  cr;           /**< Coding rate                           */
    bool              ldro;         /**< Low data-rate optimisation (auto-set) */
} sx1262_lora_mod_t;

/** LoRa packet config */
typedef struct {
    uint16_t preamble_len;          /**< Preamble length in symbols (≥12 recommended) */
    bool     fixed_length;          /**< true = implicit header, false = explicit      */
    uint8_t  payload_len;           /**< Payload length in bytes (1-255)               */
    bool     crc_on;                /**< Enable CRC                                    */
    bool     invert_iq;             /**< Invert IQ (for downlink in LoRaWAN)           */
} sx1262_lora_pkt_t;

/** Received packet info */
typedef struct {
    int16_t  rssi_pkt;    /**< RSSI of last packet (dBm)    */
    int8_t   snr_pkt;     /**< SNR  of last packet (dB)     */
    int16_t  signal_rssi; /**< Signal RSSI after despreading */
} sx1262_pkt_status_t;

/** Command status (bits 3:1 of status byte) */
typedef enum {
    SX1262_CMD_STATUS_RESERVED = 0x00,
    SX1262_CMD_STATUS_RFU      = 0x01,        /**< Reserved for future use */
    SX1262_CMD_STATUS_DATA_AVAILABLE = 0x02, /**< Data available for host */
    SX1262_CMD_STATUS_TIMEOUT  = 0x03,        /**< Command timeout */
    SX1262_CMD_STATUS_PROCESSING_ERR = 0x04, /**< Command processing error */
    SX1262_CMD_STATUS_EXEC_FAILURE = 0x05,   /**< Failure to execute command */
    SX1262_CMD_STATUS_TX_DONE  = 0x06,        /**< Command TX done */
} sx1262_cmd_status_t;

/** Chip mode (bits 6:4 of status byte) */
typedef enum {
    SX1262_CHIP_MODE_UNUSED    = 0x00,
    SX1262_CHIP_MODE_RFU       = 0x01,        /**< Reserved for future use */
    SX1262_CHIP_MODE_STDBY_RC  = 0x02,        /**< Standby with RC oscillator */
    SX1262_CHIP_MODE_STDBY_XOSC = 0x03,       /**< Standby with XOSC */
    SX1262_CHIP_MODE_FS        = 0x04,        /**< Frequency synthesizer on */
    SX1262_CHIP_MODE_RX        = 0x05,        /**< RX mode */
    SX1262_CHIP_MODE_TX        = 0x06,        /**< TX mode */
} sx1262_chip_mode_t;

/** Decoded status byte */
typedef struct {
    sx1262_cmd_status_t  cmd_status;   /**< Command status (bits 3:1) */
    sx1262_chip_mode_t   chip_mode;    /**< Chip mode (bits 6:4) */
} sx1262_status_t;

/* ===================================================================
 * Driver API
 * =================================================================== */

/**
 * @brief  Full initialisation: HW reset, TCXO, calibration, DC-DC,
 *         RF-switch config.  Call once after power-up.
 * @retval 0 on success, negative on error.
 */
int SX1262_Init(void);

/* --- Test modes --- */

/** Emit an unmodulated CW tone at the configured frequency/power. */
void SX1262_SetTxContinuousWave(void);

/** Emit an infinite LoRa preamble (useful for link testing). */
void SX1262_SetTxInfinitePreamble(void);

/* --- Low-level SPI command helpers --- */

void SX1262_WriteCommand(uint8_t opcode, const uint8_t *params, uint8_t len);
void SX1262_ReadCommand(uint8_t opcode, uint8_t *result, uint8_t len);
void SX1262_WriteRegister(uint16_t addr, const uint8_t *data, uint8_t len);
void SX1262_ReadRegister(uint16_t addr, uint8_t *data, uint8_t len);
void SX1262_WriteBuffer(uint8_t offset, const uint8_t *data, uint8_t len);
void SX1262_ReadBuffer(uint8_t offset, uint8_t *data, uint8_t len);

/* --- Operational mode commands --- */

void SX1262_SetStandby(sx1262_standby_t mode);
void SX1262_SetSleep(uint8_t config);
void SX1262_SetFs(void);
void SX1262_SetTx(uint32_t timeout_us);
void SX1262_SetRx(uint32_t timeout_us);
void SX1262_SetRxDutyCycle(uint32_t rx_period_us, uint32_t sleep_period_us);
void SX1262_SetCad(void);

/* --- Configuration commands --- */

void SX1262_SetPacketType(sx1262_packet_type_t type);
void SX1262_SetRfFrequency(uint32_t freq_hz);
void SX1262_SetPaConfig(uint8_t pa_duty_cycle, uint8_t hp_max, uint8_t device_sel);
void SX1262_SetTxParams(int8_t power_dbm, sx1262_ramp_time_t ramp);
void SX1262_SetRegulatorMode(sx1262_regulator_t mode);
void SX1262_SetBufferBaseAddress(uint8_t tx_base, uint8_t rx_base);
void SX1262_SetDio2AsRfSwitchCtrl(bool enable);
void SX1262_SetDio3AsTcxoCtrl(sx1262_tcxo_voltage_t voltage, uint32_t delay_us);
void SX1262_SetRxTxFallbackMode(sx1262_fallback_t mode);

/* --- LoRa-specific configuration --- */

void SX1262_SetLoRaModulationParams(const sx1262_lora_mod_t *mod);
void SX1262_SetLoRaPacketParams(const sx1262_lora_pkt_t *pkt);
void SX1262_SetLoRaSyncWord(uint16_t sync_word);
void SX1262_SetLoRaSymbNumTimeout(uint8_t symb_num);

/* --- IRQ --- */

void     SX1262_SetDioIrqParams(uint16_t irq_mask, uint16_t dio1_mask,
                                uint16_t dio2_mask, uint16_t dio3_mask);
uint16_t SX1262_GetIrqStatus(void);
void     SX1262_ClearIrqStatus(uint16_t mask);

/* --- Calibration --- */

void SX1262_Calibrate(uint8_t calib_param);
void SX1262_CalibrateImage(uint8_t freq1, uint8_t freq2);

/* --- Status / diagnostics --- */

uint8_t  SX1262_GetStatus(void);
void     SX1262_DecodeStatus(uint8_t status_byte, sx1262_status_t *decoded);
void     SX1262_GetRxBufferStatus(uint8_t *payload_len, uint8_t *rx_start_ptr);
void     SX1262_GetPacketStatus(sx1262_pkt_status_t *status);
int16_t  SX1262_GetRssiInst(void);
uint16_t SX1262_GetDeviceErrors(void);
void     SX1262_ClearDeviceErrors(void);

/* ===================================================================
 * High-level convenience functions
 * =================================================================== */

/**
 * @brief  Configure the radio for LoRa TX/RX with sensible defaults.
 *         Uses 915 MHz, SF9, BW125K, CR4/5, +22 dBm, private sync word.
 *         Adjust parameters in the implementation for your needs.
 * @param  freq_hz  Centre frequency in Hz (e.g. 915000000).
 * @param  mod      Pointer to modulation config.
 * @param  pkt      Pointer to packet config.
 */
void SX1262_ConfigureLora(uint32_t freq_hz,
                          const sx1262_lora_mod_t *mod,
                          const sx1262_lora_pkt_t *pkt);

/**
 * @brief  Transmit a LoRa packet (blocking, polls for TxDone / Timeout).
 * @param  data     Payload bytes.
 * @param  len      Payload length (1-255).
 * @param  timeout_ms  Max time to wait for TxDone (0 = no timeout).
 * @retval 0 on success, -1 on timeout, -2 on error.
 */
int SX1262_TransmitLora(const uint8_t *data, uint8_t len, uint32_t timeout_ms);

/**
 * @brief  Start receiving in single-shot mode (blocking, polls for RxDone / Timeout).
 * @param  buf         Buffer to store received payload.
 * @param  buf_size    Size of buf.
 * @param  rx_len      [out] actual number of bytes received.
 * @param  timeout_ms  Max time in RX (0 = continuous, no timeout).
 * @retval 0 on success, -1 on timeout, -2 on CRC error.
 */
int SX1262_ReceiveLora(uint8_t *buf, uint8_t buf_size, uint8_t *rx_len,
                       uint32_t timeout_ms);

#endif /* SX1262_H */