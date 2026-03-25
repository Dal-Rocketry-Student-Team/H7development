/**
 * @file    telemetry.h
 * @brief   Rocket telemetry packet definition for LoRa downlink.
 *
 * Packet layout (52 bytes total, packed):
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
 *   20      pressure_pa        int32_t    4     Raw pressure in Pascal
 *   24      gps_lat            float      4     Latitude  in decimal degrees (negative = South)
 *   28      gps_lon            float      4     Longitude in decimal degrees (negative = West)
 *   32      gps_utc_time       uint32_t   4     UTC time packed as HHMMSS  (e.g. 143022 = 14:30:22)
 *   36      gps_utc_date       uint32_t   4     UTC date packed as DDMMYY  (e.g. 210326 = 21 Mar 2026)
 *   40      gps_speed_cms      uint16_t   2     Speed over ground in cm/s  (divide by 100 for m/s)
 *   42      gps_course_cd      uint16_t   2     Course over ground in centidegrees (divide by 100 for °)
 *   44      gps_alt_cm         int32_t    4     GPS altitude above MSL in cm (divide by 100 for m)
 *   48      gps_hdop           uint16_t   2     HDOP × 100 (e.g. 120 = 1.20 — lower is better)
 *   50      gps_sats           uint8_t    1     Satellites used in fix (from $GNGGA field 7)
 *   51      gps_fix_type       uint8_t    1     Fix quality: 0=no fix, 1=GPS, 2=DGPS (from $GNGGA field 6)
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
    uint32_t gps_utc_time;          // gps utc time in hhmmss format, e.g. 231523 means 23:15:23 UTC
    uint32_t gps_utc_date;          // gps utc date in ddmmyy format, e.g. 150623 means 15 June 2023
    uint16_t gps_speed_cms;         // gps speed in cm/s, e.g. 5144 means 51.44 cm/s
    uint16_t gps_course_cd;         // gps course over ground in centidegrees, e.g. 12345 means 123.45°
    int32_t  gps_alt_cm;            // GPS altitude above MSL in cm (divide by 100 for m)
    uint16_t gps_hdop;              // HDOP × 100 (e.g. 120 = 1.20 — lower is more accurate)
    uint8_t  gps_sats;              // satellites used in fix (from $GNGGA field 7)
    uint8_t  gps_fix_type;          // fix quality: 0=no fix, 1=GPS, 2=DGPS (from $GNGGA field 6)
} rocket_telemetry_t;

/* Compile time size guard. If padding is added to the data packet, compilation will fail */
_Static_assert(sizeof(rocket_telemetry_t) == 52, "rocket_telemetry_t must be exactly 52 bytes — check packing!");

/*
Ground uplink command packet (ground → rocket) — fits in 40-byte RX region
Minimal placeholder - make a command protocol!
*/
typedef struct __attribute__((packed)) {
    uint8_t cmd;            // Command opcode
    uint8_t value;          // Argument for command opcode to be acted on by receiver
} ground_command_t;

#endif /* TELEMETRY_H */