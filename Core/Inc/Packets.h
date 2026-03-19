#ifndef PACKETS_H
#define #PACKETS_H

#include <stdint.h>
#indluce <stdbool.h>

/*
We will send a 77 byte (max) package to the radio which will then transmit this to the ground station
the ground station will decode the packet.
*/

//Packet sync values, indicates where a packet starts
#define PKT_SYNC_0 0xAA 
#define PKT_SYNC_1 0x55

/*
Presence Mask Bits

This mask will indicate what data each packet contains
if a bit is 1, that means the sensor it is associated with is sending data in the packet
the receiver will use this mask before parsing the packet
extra 10 bits could be used for more information in the future
*/ 

#define PKT_HAS_ADXL375   (1u << 0) //bit 0
#define PKT_HAS_LIS2MDL   (1u << 1) //bit 1
#define PKT_HAS_ICM40609  (1u << 2) //bit 2
#define PKT_HAS_LSM6DSV   (1u << 3) //bit 3
#define PKT_HAS_MS5607    (1u << 4) //bit 4
#define PKT_HAS_GNSS      (1u << 5) //bit 5
#define PKT_FLAG_ERROR    (1u << 15) //bit 15

//sensor data structs
//what data each sensor will be sending
//as of now, this will be raw data, converted by the ground station to limit source of error
typedef struct{
    int16_t x, y, z;
} ADXL375_Data;

typedef struct{
    int16_t x, y, z
} LIS2MDL_Data;

typedef struct{
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;
    int16_t temp;
} ICM40609_Data;

typedef struct {
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;
    int16_t temo;
} LSM6DSV_Data;

typedef struct{
    int32_t pressure_pa;
    int32_t temp_centideg // degrees*100
} MS5607_Data;

typedef struct {
    int32_t lat_1e7; //degrees * 1e7
    int32_t lon_1e7;
    int32_t alt_mm; //altitude in mm above sea level
    int16_t speed_cms //ground speed in cm/s
    int16_t heading_cdeg; //heading in degrees*100
    uint8_t fix_type; // 0 = none, 1 = dead reckoning, 2 = 2D, 3 =3D
    uint8_t n_sats;
} GNSS_Data;

// Packet Container

typedef struct{
    uint8_t seq; //sequence number to track packets, if we receive pkt 1 and 3, we know we lost 2. also useful for duplicates
    uint32_t timestamp_ms; //time sinnce microcontroller booted, helps preserve a data timeline
    uint16_t present_mask;
    ADXL375_Data adxl;
    LIS2MDL_Data lis;
    ICM40609_Data icm;
    LSM6DSV_Data lsm;
    MS5607_Data ms5607;
    GNSS_Data gnss;
} AvionicsPacket;

//Buffer size
#define AVPKT_MAX_LEN 80u //2 sync+1 seq + 4 ts +2 mas +66 payload + 2crc

/*
Serialises an avionics packet to send
only fields whose bit in pkt->present_mask are written
returns numebr of bytes ritten or -1 on error
*/
int avpkt_encode(const AvionicsPacket *pkt, uint8_t *buf, uint16_t buf_len);

/*
Parse received byte buffer back into AvionicsPacket
validates sync word and CRC before parsing
returns 0 on success, negative error code on failure
*/

int avpkt_decode(const uint8_t *buf, uint16_t len, AvionicsPacket *pkt_out);


/*
encode and transmit over LoRa in one call
returns 0 on success, negative on LoRa send failure
*/

int avpkt_send(const AvionicsPacket *pkt);

/*
CRC-16/CCITT over a byte buffer
used internally by encode/decde
this is an error checker to make sure the received data is not corrupted
the packet will contain the result of this algorithm, then the ground station runs the same one
if they match, data is good, if not, discard data
*/
uint16_t avpkt_crc16(const uint8_t *data, uint16_t len);

#endif /* PACKETS_H */
