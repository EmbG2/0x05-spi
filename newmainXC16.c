#include "xc.h"
#include "timer.h"
#include "spi.h"
#include "uart.h"
#include <stdio.h>
#include <math.h>

#define FCY         72000000 
#define BAUDRATE    9600
#define BRGVAL      (FCY / BAUDRATE) / 16 - 1

#define MAG_CS LATDbits.LATD6
#define NUM_READINGS 6
char buff[35];

#define MOVING_AVERAGE_SIZE 10 
int16_t moving_average_buffer_x[MOVING_AVERAGE_SIZE];
int16_t moving_average_buffer_y[MOVING_AVERAGE_SIZE];
int16_t moving_average_buffer_z[MOVING_AVERAGE_SIZE];
uint8_t buffer_x_index = 0;
uint8_t buffer_y_index = 0;
uint8_t buffer_z_index = 0;

uint8_t readings[NUM_READINGS];

int16_t calculate_moving_average(int16_t new_value, int16_t buffer[MOVING_AVERAGE_SIZE], uint8_t *idx) {
    buffer[*idx] = new_value;
    *idx = (*idx + 1) % MOVING_AVERAGE_SIZE;
    int16_t sum = 0; 
    for (uint8_t i = 0; i < MOVING_AVERAGE_SIZE; i++) {
        sum += buffer[i];
    }
    return (int16_t)(sum / MOVING_AVERAGE_SIZE);
}

int16_t merge_significant_bits(uint8_t low, uint8_t high, int axis) {
    int16_t data;
    if (axis == 1 || axis == 2) {
        uint8_t masked_low = low & 0xF8;
        data = (int16_t)((high << 8) | masked_low);
        data = data / 8;
    } else {
        uint8_t masked_low = low & 0xFE;
        data = (int16_t)((high << 8) | masked_low);
        data = data / 8;
    }
    return data;
}

int main(void) {
    ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = 0x0000;

    UART_Init(UART_1);
    
    send_uart_char(UART_1, 'S');
    send_uart_char(UART_1, '\n');

    spi_init();
    tmr_wait_ms(TIMER1, 100);

    MAG_CS = 0;
    spi_write(0x4B, 0x01);
    MAG_CS = 1;
    tmr_wait_ms(TIMER1, 5);

    MAG_CS = 0;
    spi_write(0x4C, 0x30);
    MAG_CS = 1;
    tmr_wait_ms(TIMER1, 5);
    
    MAG_CS = 0;
    uint8_t chip_id = spi_read(0x40);
    MAG_CS = 1;
    tmr_wait_ms(TIMER1, 10);

    // Send chip ID over UART
    U1TXREG = chip_id / 16 + '0';
    while (U1STAbits.UTXBF);
    U1TXREG = chip_id % 16 + '0';
    while (U1STAbits.UTXBF);
    U1TXREG = 'E';
    while (U1STAbits.UTXBF);
    
    send_uart_char(UART_1, '\n');

    while (1) {
        MAG_CS = 0;
        spi_read_multiple(readings, 0x42);
        MAG_CS = 1;
        
        int16_t x_data = merge_significant_bits(readings[0], readings[1], 1);
        int16_t average_x = calculate_moving_average(x_data, moving_average_buffer_x, &buffer_x_index);

        int16_t y_data = merge_significant_bits(readings[2], readings[3], 2);
        int16_t average_y = calculate_moving_average(y_data, moving_average_buffer_y, &buffer_y_index);

        int16_t z_data = merge_significant_bits(readings[4], readings[5], 3);
        int16_t average_z = calculate_moving_average(z_data, moving_average_buffer_z, &buffer_z_index);
        
        tmr_wait_ms(TIMER1, 10);

        int heading_deg = atan2(average_y, average_x) * (180.0 / M_PI);

        char buff[32];

        sprintf(buff, "MAGX:%d\n", average_x);
        send_uart_string(buff);

        sprintf(buff, "MAGXY:%d\r\n", average_y);
        send_uart_string(buff);

        sprintf(buff, "MAGXZ:%d\r\n", average_z);
        send_uart_string(buff);

        sprintf(buff, "MAGXD:%d\r\n", heading_deg);
        send_uart_string(buff);


        tmr_wait_ms_3(TIMER2, 1000);
    }

    return 0;
}
