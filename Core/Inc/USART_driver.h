#ifndef USART_DRIVER_H
#define USART_DRIVER_H

// volatile uint8_t gps_rx_ch;
// volatile uint8_t gps_line_ready = 0;
// char gps_line[GPS_LINE_MAX];
// volatile uint16_t gps_idx = 0;
#include <stdint.h>

#define GPS_LINE_MAX 128


void send_host_message(const char* message);
void receive_host_message(char* buffer, int max_len);

void GPS_arm_receive_interrupt(void);
//void print_GPS(void);

#endif /* USART_DRIVER_H */