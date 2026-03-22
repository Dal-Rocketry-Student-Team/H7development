#include "GPS.h"
#include "USART_driver.h"
#include <stdint.h>
#include <string.h>

#define GPS_BUFFER_MAX_SIZE 8

typedef struct
{
    GPS_RMC_t buffer[GPS_CB_SIZE];
    volatile uint8_t head;
    volatile uint8_t tail;
} GPS_circular_buffer_t;

/* Main circular buffer for GPS messages */
static GPS_circular_buffer_t gps_cbuf;

/* -----------------------------------------------------------------------
 * Internal helpers
 * ----------------------------------------------------------------------- */

/* Parse a decimal float from a field pointer.  Stops at ',' '*' or '\0'.
 * Returns the value and advances *pp past the last digit consumed. */
static float parse_field_float(const char **pp)
{
    const char *p = *pp;
    float int_part = 0.0f;
    while (*p && *p != '.' && *p != ',' && *p != '*')
        int_part = int_part * 10.0f + (float)(*p++ - '0');
    float frac = 0.0f;
    float scale = 0.1f;
    if (*p == '.') {
        p++;
        while (*p && *p != ',' && *p != '*') {
            frac += (float)(*p++ - '0') * scale;
            scale *= 0.1f;
        }
    }
    *pp = p;
    return int_part + frac;
}

/* Parse a GNRMC sentence into a GPS_RMC_t.
 *
 * GNRMC field layout (comma-separated):
 *   0  $GNRMC
 *   1  HHMMSS.SS   UTC time
 *   2  A/V         Status  (A = active fix, V = void)
 *   3  DDMM.MMMMM  Latitude
 *   4  N/S
 *   5  DDDMM.MMMMM Longitude
 *   6  E/W
 *   7  Speed over ground, knots
 *   8  Course over ground, degrees
 *   9  DDMMYY      Date
 *  10+ (magnetic variation, mode indicator — not needed)
 *
 * Returns 1 on success (valid fix), 0 if void or parse error.
 */
static uint8_t parse_rmc_fast(const char *line, GPS_RMC_t *fix)
{
    /* ---- Tokenise: build pointer array for each field ---- */
    const char *f[12];
    uint8_t nf = 0;
    f[nf++] = line;
    for (const char *p = line; *p && *p != '*' && nf < 12; p++) {
        if (*p == ',')
            f[nf++] = p + 1;
    }
    if (nf < 10)
        return 0;

    /* ---- Field 2: status (must be 'A' for valid fix) ---- */
    if (f[2][0] != 'A') {
        fix->valid = 0;
        return 0;
    }
    fix->valid = 1;

    /* ---- Field 1: UTC time HHMMSS.SS → utc_time as HHMMSS integer ---- */
    {
        const char *p = f[1];
        if (p[0] >= '0' && p[1] >= '0' && p[2] >= '0' && p[3] >= '0' &&
            p[4] >= '0' && p[5] >= '0') {
            uint32_t h = (uint32_t)(p[0]-'0')*10 + (uint32_t)(p[1]-'0');
            uint32_t m = (uint32_t)(p[2]-'0')*10 + (uint32_t)(p[3]-'0');
            uint32_t s = (uint32_t)(p[4]-'0')*10 + (uint32_t)(p[5]-'0');
            fix->utc_time = h*10000u + m*100u + s;
        } else {
            fix->utc_time = 0;
        }
    }

    /* ---- Field 3+4: latitude DDMM.MMMMM N/S → lat_e7 ---- */
    {
        const char *p = f[3];
        /* Degrees: first two digits */
        float deg = (float)((p[0]-'0')*10 + (p[1]-'0'));
        /* Minutes: remainder starting at index 2 */
        p += 2;
        float minutes = parse_field_float(&p);
        float lat = deg + minutes / 60.0f;
        if (f[4][0] == 'S') lat = -lat;
        fix->lat_e7 = (int32_t)(lat * 1e7f);
    }

    /* ---- Field 5+6: longitude DDDMM.MMMMM E/W → lon_e7 ---- */
    {
        const char *p = f[5];
        /* Degrees: first three digits */
        float deg = (float)((p[0]-'0')*100 + (p[1]-'0')*10 + (p[2]-'0'));
        /* Minutes: remainder starting at index 3 */
        p += 3;
        float minutes = parse_field_float(&p);
        float lon = deg + minutes / 60.0f;
        if (f[6][0] == 'W') lon = -lon;
        fix->lon_e7 = (int32_t)(lon * 1e7f);
    }

    /* ---- Field 7: speed in knots → cm/s (1 knot = 51.4444 cm/s) ---- */
    {
        const char *p = f[7];
        float knots = parse_field_float(&p);
        fix->speed_cms = (uint16_t)(knots * 51.4444f);
    }

    /* ---- Field 8: course over ground → centidegrees ---- */
    {
        const char *p = f[8];
        float cog = parse_field_float(&p);
        fix->course_cd = (uint16_t)(cog * 100.0f);
    }

    /* ---- Field 9: date DDMMYY → utc_date integer ---- */
    {
        const char *p = f[9];
        if (p[0] >= '0' && p[1] >= '0' && p[2] >= '0' &&
            p[3] >= '0' && p[4] >= '0' && p[5] >= '0') {
            uint32_t d = (uint32_t)(p[0]-'0')*10 + (uint32_t)(p[1]-'0');
            uint32_t mo = (uint32_t)(p[2]-'0')*10 + (uint32_t)(p[3]-'0');
            uint32_t y  = (uint32_t)(p[4]-'0')*10 + (uint32_t)(p[5]-'0');
            fix->utc_date = d*10000u + mo*100u + y;
        } else {
            fix->utc_date = 0;
        }
    }

    return 1;
}

/* Push a parsed fix into the circular buffer.
 * Called ONLY from ISR context — must not block. */
static uint8_t gps_cbuf_push_isr(const GPS_RMC_t *fix)
{
    uint8_t next = (gps_cbuf.head + 1) % GPS_BUFFER_MAX_SIZE;
    if (next == gps_cbuf.tail)
        return 0; /* buffer full — drop the fix */
    gps_cbuf.buffer[gps_cbuf.head] = *fix;
    gps_cbuf.head = next;
    return 1;
}

/* -----------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------- */

/* Called ONLY from UART ISR (via USART_driver HAL_UART_RxCpltCallback) */
void GPS_push_line(const char *line, int length)
{
    if (length < 7)
        return;
    if (memcmp(line, "$GNRMC", 6) != 0)
        return;

    GPS_RMC_t fix;
    if (!parse_rmc_fast(line, &fix))
        return;

    gps_cbuf_push_isr(&fix);
}

void GPS_init(void)
{
    GPS_arm_receive_interrupt();
}

/* Pop the oldest fix from the buffer.
 * Called from main-loop context only.
 * Returns 1 if a fix was available, 0 if the buffer was empty. */
uint8_t GPS_pop(GPS_RMC_t *out)
{
    if (gps_cbuf.head == gps_cbuf.tail)
        return 0; /* empty */

    *out = gps_cbuf.buffer[gps_cbuf.tail];
    gps_cbuf.tail = (gps_cbuf.tail + 1) % GPS_BUFFER_MAX_SIZE;
    return 1;
}