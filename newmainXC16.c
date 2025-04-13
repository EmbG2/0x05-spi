// (X-axis)
#include "xc.h"
#include "timer.h"
#include <stdio.h>
#include <math.h>

#define FCY         72000000 
#define BAUDRATE    9600
#define BRGVAL      (FCY / BAUDRATE) / 16 - 1

#define MAG_CS LATDbits.LATD6

char buff[35];

#define MOVING_AVERAGE_SIZE 10 
int16_t moving_average_buffer_x[MOVING_AVERAGE_SIZE];
int16_t moving_average_buffer_y[MOVING_AVERAGE_SIZE];
int16_t moving_average_buffer_z[MOVING_AVERAGE_SIZE];
uint8_t buffer_x_index = 0;
uint8_t buffer_y_index = 0;
uint8_t buffer_z_index = 0;

int16_t calculate_moving_average(int16_t new_value, int16_t buffer[MOVING_AVERAGE_SIZE], uint8_t * idx) {
    buffer[*idx] = new_value;
    *idx = (*idx + 1) % MOVING_AVERAGE_SIZE;
    int16_t sum = 0; 
    for (uint8_t i = 0; i < MOVING_AVERAGE_SIZE; i++) {
        sum += buffer[i];
    }
    return (int16_t)(sum / MOVING_AVERAGE_SIZE);
}

void uart_setup(int UART_n, int stop, int parity) {
    switch (UART_n) {
        case 1:
            U1MODEbits.UARTEN = 0;
            U1MODEbits.STSEL = stop;
            U1MODEbits.PDSEL = parity;
            U1MODEbits.BRGH = 0;
            U1BRG = BRGVAL;
            IFS0bits.U1RXIF = 0;
            IEC0bits.U1RXIE = 1;
            U1MODEbits.UARTEN = 1;
            U1STAbits.UTXEN = 1;
            break;
        default:
            return;
    }
}

uint8_t spi_transfer(uint8_t byte) {
    while (SPI1STATbits.SPITBF);
    SPI1BUF = byte;
    while (!SPI1STATbits.SPIRBF);
    return SPI1BUF;
}

void spi_write(uint8_t reg, uint8_t value) {
    spi_transfer(reg & 0x7F); // MSB=0 for write
    spi_transfer(value);
}

uint8_t spi_read(uint8_t reg) {
    spi_transfer(reg | 0x80); // MSB=1 for read
    uint8_t val = spi_transfer(0x00);
    return val;
}

int16_t merge_significant_bits(uint8_t low, uint8_t high, int axis){
    int16_t data;
    if (axis == 1 || axis == 2){                        // X and Y
        uint8_t masked_low = low & 0xF8; // 0xF8 = 1111 1000
        data = (int16_t)((high << 8) | masked_low);
        data = data / 8;
    } else {                                            // Z
        uint8_t masked_low = low & 0xFE; // 0xFE = 1111 1110
        data = (int16_t)((high << 8) | masked_low);
        data = data / 8;
    }
    return data;
}

int main(void) {
    ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = 0x0000;

    // UART-1 setup
    RPOR0bits.RP64R = 1;
    RPINR18bits.U1RXR = 75;
    uart_setup(1, 1, 0);

    U1TXREG = 'S'; 
    while (U1STAbits.UTXBF);

    // SPI-1 setup
    TRISAbits.TRISA1 = 1;                   // MISO
    TRISFbits.TRISF12 = 0;                  // SCK
    TRISFbits.TRISF13 = 0;                  // MOSI
    TRISBbits.TRISB3 = 0;                   // CS2
    LATBbits.LATB3 = 1;                     // Disable
    TRISBbits.TRISB4 = 0;                   // CS1
    LATBbits.LATB4 = 1;                     // Disable
    TRISDbits.TRISD6 = 0;                   // CS3
    LATDbits.LATD6 = 1;                     // Disable
    RPINR20bits.SDI1R = 0b0010001;          // MISO (SDI1) - RPI17
    RPOR12bits.RP109R = 0b000101;           // MOSI (SDO1) - RF13
    RPOR11bits.RP108R = 0b000110;           // SCK1
    SPI1CON1bits.MSTEN = 1;                 // master mode
    SPI1CON1bits.MODE16 = 0;                // 8-bit mode
    SPI1CON1bits.PPRE = 0;                  // 64:1 primary prescaler
    SPI1CON1bits.SPRE = 7;                  // 1:1 secondary prescaler
    SPI1CON1bits.CKP = 1;                   // clock polarity
    SPI1STATbits.SPIROV = 0;                //
    SPI1STATbits.SPIEN = 1;                 // enable SPI

    tmr_wait_ms(TIMER1, 100);

    MAG_CS = 0;
    spi_write(0x4B, 0x01);                  // Suspend X
    MAG_CS = 1;
    tmr_wait_ms(TIMER1, 5);

    MAG_CS = 0;
    spi_write(0x4C, 0x30);                  // Set X to normal mode
    MAG_CS = 1;
    tmr_wait_ms(TIMER1, 5);
    
    // Read chip ID from 0x40
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

    while (1) {
        // (X-axis)
        MAG_CS = 0;
        uint8_t low_x = spi_read(0x42);
        uint8_t high_x = spi_read(0x43);
        MAG_CS = 1;
        int16_t x_data = merge_significant_bits(low_x, high_x, 1);
        int16_t average_x = calculate_moving_average(x_data, moving_average_buffer_x, &buffer_x_index);

        // (Y-axis)
        MAG_CS = 0;
        uint8_t low_y = spi_read(0x44);
        uint8_t high_y = spi_read(0x45);
        MAG_CS = 1;
        int16_t y_data = merge_significant_bits(low_y, high_y, 2);
        int16_t average_y = calculate_moving_average(y_data, moving_average_buffer_y, &buffer_y_index);

        // (Z-axis)
        MAG_CS = 0;
        uint8_t low_z = spi_read(0x46);
        uint8_t high_z = spi_read(0x47);
        MAG_CS = 1;
        int16_t z_data = merge_significant_bits(low_z, high_z, 3);
        int16_t average_z = calculate_moving_average(z_data, moving_average_buffer_z, &buffer_z_index);

        tmr_wait_ms(TIMER1, 10);
        
        // calculating the direction
        int heading_deg = atan2(average_y, average_x) * (180.0 / M_PI);

        // Send data over UART
        sprintf(buff, "X:%d Y:%d Z:%d D:%d    ", average_x, average_y, average_z, heading_deg);
        for (char i = 0; i < 35; i++) {
            U1TXREG = buff[i];
            while (U1STAbits.UTXBF);
        }
        U1TXREG = ' ';
        while (U1STAbits.UTXBF);
        U1TXREG = '-';
        while (U1STAbits.UTXBF);
        U1TXREG = ' ';
        while (U1STAbits.UTXBF);

        tmr_wait_ms_3(TIMER2, 1000);
    }

    return 0;
}
