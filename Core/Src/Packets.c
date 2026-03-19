#include "Packets.h"

int avpkt_encode(const AvionicsPacket *pkt, uint8_t *buf, uint16_t buf_len){
    uint16_t cursor = 0;
    //sync
    buf[cursor++] = 0xAA;
    buf[cursor++] = 0x55;
    //sequence
    buf[cursor++] = pkt->seq;
    //timestamp
    memcpy(&buf[cursor], &pkt->timestamp_ms, sizeof(pkt->timestamp_ms));
    cursor += sizeof(pkt->timestamp_ms); //increment cursor by size of timestamp
    //presence mask
    memcpy(&buf[cursor], &pkt->present_mask, sizeof(pkt->present_mask));
    cursor += sizeof(pkt->present_mask);

    /*
    Copying raw data signals into buffer, then incrementing cursor appropriate amount
    */
    if(pkt->present_mask & PKT_HAS_ADXL375){
        memcpy(&buf[cursor], &pkt->adxl, sizeof(pkt->adxl));
        cursor += sizeof(pkt->adxl);
    }

    if(pkt->present_mask & PKT_HAS_LIS2MDL){
        memcpy(&buf[cursor], &pkt->lis, sizeof(pkt->lis));
        cursor += sizeof(pkt->lis);
    }

    if(pkt->present_mask & PKT_HAS_ICM40609){
        memcpy(&buf[cursor], &pkt->icm, sizeof(pkt->icm));
        cursor += sizeof(pkt->icm);
    }

    if(pkt->present_mask & PKT_HAS_LSM6DSV){
        memcpy(&buf[cursor], &pkt->lsm, sizeof(pkt->lsm));
        cursor += sizeof(pkt->lsm);
    }

    if(pkt->present_mask & PKT_HAS_MS5607){
        memcpy(&buf[cursor], &pkt->ms5607, sizeof(pkt->ms5607));
        cursor += sizeof(pkt->ms5607);
    }

    if(pkt->present_mask & PKT_HAS_GNSS){
        memcpy(&buf[cursor], &pkt->gnss, sizeof(pkt->gnss));
        cursor += sizeof(pkt->gnss);
    }

    uint16_t crc = avpkt_crc16(&buf[2], cursor-2);
    memcpy(&buf[cursor], &crc, sizeof(crc));
    cursor += sizeof(crc);

    return cursor;
}

int avpkt_decode(const uint8_t *buf, uint16_t len, AvionicsPacket *pkt_out){
   int cursor = 0;
    if((buf[0] == 0xAA && buf[1] == 0x55) == 0){
        return -1; //invalid packet (wrong sync)
    }
    cursor += 2; //past sync bits
    pkt_out->seq = buf[cursor];
    cursor ++;

    memcpy(&pkt_out->timestamp_ms, &buf[cursor], sizeof(pkt_out->timestamp_ms));
    cursor += sizeof(pkt_out->timestamp_ms);

    memcpy(&pkt_out->present_mask, &buf[cursor], sizeof(pkt_out->present_mask));
    cursor += sizeof(pkt_out->present_mask);

    if(pkt_out->present_mask & PKT_HAS_ADXL375){
        memcpy(&pkt_out->adxl, &buf[cursor], sizeof(pkt_out->adxl));
        cursor += sizeof(pkt_out->adxl);
    }

    if(pkt_out->present_mask & PKT_HAS_LIS2MDL){
        memcpy(&pkt_out->lis, &buf[cursor], sizeof(pkt_out->lis));
        cursor += sizeof(pkt_out->lis);
    }

    if(pkt_out->present_mask & PKT_HAS_ICM40609){
        memcpy(&pkt_out->icm, &buf[cursor], sizeof(pkt_out->icm));
        cursor += sizeof(pkt_out->icm);
    }

    if(pkt_out->present_mask & PKT_HAS_LSM6DSV){
        memcpy(&pkt_out->lsm, &buf[cursor], sizeof(pkt_out->lsm));
        cursor += sizeof(pkt_out->lsm);
    }

    if(pkt_out->present_mask & PKT_HAS_MS5607){
        memcpy(&pkt_out->ms5607, &buf[cursor], sizeof(pkt_out->ms5607));
        cursor += sizeof(pkt_out->ms5607);
    }

    if(pkt_out->present_mask & PKT_HAS_GNSS){
        memcpy(&pkt_out->gnss, &buf[cursor], sizeof(pkt_out->gnss));
        cursor += sizeof(pkt_out->gnss);
    }

    uint16_t crc_computed = avpkt_crc16(&buf[2], cursor - 2);
    uint16_t crc_received;
    memcpy(&crc_received, &buf[cursor], sizeof(crc_received));

    if(crc_computed != crc_received){
        return -2; // corrupted packet
    }

    return 0; // success
}

int avpkt_send(const AvionicsPacket *pkt){
    
}

uint16_t avpkt_crc16(const uint8_t *data, uint16_t len){
    uint16_t crc = 0xFFFF;
    const uint16_t polynomial = 0x1021; //generator polynomial

    for(size_t i = 0; i < len; i++){
        crc ^= (uint16_t)data[i] << 8; //byte put into 'upper half' of crc register

        for(uint8_t j =0; j <8; j++){
            if(crc & 0x8000){ //if MSB of crc is 1

                crc = (uint16_t)((crc << 1) ^ polynomial); //shift crc left one bit, then perform the polynomial division
            }
            else{
                crc = (uint16_t)(crc << 1); //shift crc by one bit
            }
        }
    }
    return crc;
}