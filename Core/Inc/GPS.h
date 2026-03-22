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

/* GPS acquisition status, updated continuously from GNGSA and GSV sentences.
 * Populated even when there is no valid fix, so the display can show
 * "Searching (N in view)" rather than a plain "No fix". */
typedef struct {
    uint8_t fix_type;     // 1 = no fix,  2 = 2D fix,  3 = 3D fix  (GNGSA field 2)
    uint8_t sats_in_use;  // satellites actively used in the solution (GNGSA PRN fields, all constellations)
    uint8_t sats_in_view; // total satellites visible (GPGSV + GLGSV field 3 sum)
} GPS_status_t;

uint8_t GPS_pop(GPS_RMC_t *out);

/* Returns a snapshot of the current GPS acquisition status.
 * Safe to call from main-loop context (fields are single-byte, atomically readable on Cortex-M). */
void GPS_GetStatus(GPS_status_t *out);

void GPS_init(void);

void GPS_push_line(const char* line, int length);

#endif