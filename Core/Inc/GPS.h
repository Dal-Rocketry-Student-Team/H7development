#ifndef GPS_H
#define GPS_H
#include <stdint.h>

#define GPS_CB_SIZE 8

/* Things to be done on GPS side: 
1. Implement true form of circular buffer - the push() needs to wrap around after reaching the maximum size
2. Implement application-facing parsing functions for NMEA sentences - most important are GGA, GLL, GSA, GSV, RMC
3. Refactor code such for cleaner API via GPS header */

typedef struct {
    uint8_t  valid;
    int32_t  lat_e7;
    int32_t  lon_e7;
    uint32_t utc_time;
    uint32_t utc_date;
    uint16_t speed_cms;
    uint16_t course_cd;
} GPS_RMC_t;

uint8_t GPS_pop(GPS_RMC_t *out);

void GPS_init(void);

void GPS_push_line(const char* line, int length);

#endif 