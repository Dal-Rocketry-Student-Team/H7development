/**
 * @file    telemetry.h
 * @brief   Rocket telemetry packet definition for LoRa downlink.
 *
 * Packet layout (32 bytes total, packed):
 *
 *   Offset  Field              Type      Bytes  Notes
 *   ------  -----------------  --------  -----  --------------------------------
 *    0      packet_id          uint16_t   2     Rolling counter — detect dropped packets
 *    2      timestamp_ms       uint32_t   4     HAL_GetTick() at sample time
 *    6      ax                 int16_t    2     Raw accel X  — divide by 2048.0 for g  (±16 g FS)
 *    8      ay                 int16_t    2
 *   10      az                 int16_t    2
 *   12      gx                 int16_t    2     Raw gyro X — divide by 16.4 for dps (±2000 dps FS)
 *   14      gy                 int16_t    2
 *   16      gz                 int16_t    2
 *   18      temperature_cdeg   int16_t    2     Baro temp × 100  (e.g. 2315 = 23.15 °C)
 *   20      pressure_Pa        int32_t    4     Raw pressure in Pascal
 *   24      gps_lat            float      4     GPS latitude  — placeholder, set 0.0f until GPS ready
 *   28      gps_lon            float      4     GPS longitude — placeholder, set 0.0f until GPS ready
 *
 * Altitude is intentionally omitted from the packet.  The ground station has
 * its own MS5607 reading local pressure and temperature at the launch site.
 * Using the differential between rocket and ground readings gives a more
 * accurate altitude above ground level (AGL) than any formula the rocket
 * could run using a hardcoded sea-level reference.  Omitting the field also
 * saves 4 bytes and removes a powf() call from the flight loop.
 *
 * No software checksum field — the SX1262 appends a hardware CRC16 to every
 * LoRa packet automatically (controlled by crc_on in sx1262_lora_pkt_t).
 *
 * SX1262 FIFO buffer split (256 bytes total):
 *   TX: bytes   0–215  (216 bytes) — large downlink telemetry packets, rocket → ground
 *   RX: bytes 216–255  ( 40 bytes) — small uplink command packets,   ground → rocket
 */

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>

/*
-----------------------------------------------------------------------
SX1262 FIFO buffer base addresses
Must match SX1262_TX_BASE / SX1262_RX_BASE in sx1262.h and the call to
SetBufferBaseAddress() at init time.
----------------------------------------------------------------------- 
*/

#define TELEMETRY_TX_BASE 0x00u     /* 0 decimal */
#define TELEMETRY_RX_BASE 0xD8u     /* 216 decimal */

/*
-----------------------------------------------------------------------
Telemetry downlink packet (rocket → ground)

__attribute__((packed)) tells GCC not to insert any padding bytes between
fields for alignment.  Without it, a struct that is 32 bytes on the STM32
might be 36 bytes on an x86 ground-station decoder, and your field offsets
will be wrong at the receiving end.  Always pack structs that cross
hardware or language boundaries.
----------------------------------------------------------------------- 
*/

typedef struct __attribute__((packed)) {
    uint16_t packet_id;             // packet counter
    uint32_t timestamp_ms;          // HAL tick time stamp for packet
    int16_t ax;                     // accel x-axis raw
    int16_t ay;                     // accel y-axis raw
    int16_t az;                     // accel z-axis raw
    int16_t gx;                     // gyro x-axis raw
    int16_t gy;                     // gyro y-axis raw
    int16_t gz;                     // gyro z-axis raw
    int16_t temperature_cdeg;       // barometer temp reading - float formatted as int Eg: 4122 means 41.22 deg celsius
    int32_t pressure_pa;            // barometer pressure reading
    float gps_lat;                  // gps latitude reading, leave it as 0.0 until gps configured
    float gps_lon;                  // gps longitude reading, leave it as 0.0 until gps configured
} rocket_telemetry_t;

/* Compile time size guard. If padding is added to the data packet, compilation will fail */
_Static_assert(sizeof(rocket_telemetry_t) == 32, "rocket_telemetry_t must be exactly 32 bytes — check packing!");

/*
Ground uplink command packet (ground → rocket) — fits in 40-byte RX region
Minimal placeholder - make a command protocol!
*/
typedef struct __attribute__((packed)) {
    uint8_t cmd;            // Command opcode
    uint8_t value;          // Argument for command opcode to be acted on by receiver
} ground_command_t;

#endif /* TELEMETRY_H */