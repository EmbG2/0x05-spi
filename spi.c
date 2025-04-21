/*
 * File:   spi.c
 * Author: EMBG2
 *
 * Created on April 20, 2025, 1:39 PM
 */

#include "spi.h"

void spi_init(void) {
    TRISAbits.TRISA1 = 1;          // MISO
    TRISFbits.TRISF12 = 0;         // SCK
    TRISFbits.TRISF13 = 0;         // MOSI
    
    TRISBbits.TRISB3 = 0;          // CS1 Accelerometer
    LATBbits.LATB3 = 1;
    TRISBbits.TRISB4 = 0;          // CS2 Gyroscope
    LATBbits.LATB4 = 1;
    TRISDbits.TRISD6 = 0;          // CS3 Magnetometer
    LATDbits.LATD6 = 1;

    RPINR20bits.SDI1R = 0b0010001;    // MISO (SDI1) - RPI17
    // SDI1R: Selects the input pin for SPI1 data input (MISO).
    // 0b0010001 = 17, 
    // so: Pin RPI17 is mapped as SDI1 (SPI1 MISO ? Master In Slave Out).
    // This means the SPI1 module will receive data through pin 17.
    
    RPOR12bits.RP109R = 0b000101;     // MOSI (SDO1) - RF13
    // RP109R: Selects the output function for Remappable Pin 109.
    // 0b000101 = 5, 
    // and function 5 is SDO1 (SPI1 MOSI ? Master Out Slave In).
    // So, pin RP109 (RF13) will act as the SPI1 MOSI output.
    
    RPOR11bits.RP108R = 0b000110;     // SCK1
    // RP108R: Selects the output function for Remappable Pin 108.
    // 0b000110 = 6, 
    // and function 6 is SCK1 (SPI1 Clock Output).
    // So, pin RP108 (usually RF12) will generate the SPI clock signal (SCK).
        
    SPI1CON1bits.MSTEN = 1;        // master mode
    SPI1CON1bits.MODE16 = 0;       // 8-bit mode
    SPI1CON1bits.PPRE = 0;         // 64:1 primary prescaler     
    SPI1CON1bits.SPRE = 7;         // 1:1 secondary prescaler 0111
    SPI1CON1bits.CKP = 1;          // idle state high, active state low
    SPI1STATbits.SPIROV = 0;       // clear the overflow flag
    SPI1STATbits.SPIEN = 1;        // enable spi
}

uint8_t spi_transfer(uint8_t byte) {
    while (SPI1STATbits.SPITBF);      // waits until the transmit buffer is not full
    SPI1BUF = byte;
    while (!SPI1STATbits.SPIRBF);     // Waits until the received byte is ready
    return SPI1BUF;
}

void spi_write(uint8_t reg, uint8_t value) {
    spi_transfer(reg & 0x7F);         // send the address and set MSB to 0
                                      // 0x7F = 0111 1111
    spi_transfer(value);              // send the value
}

uint8_t spi_read(uint8_t reg) {
    spi_transfer(reg | 0x80);         // send the address and set MSB to 1 
                                      // 0x80 = 1000 0000
    return spi_transfer(0x00);        // send 0x00 to enable the generation of the clock
}

void spi_read_multiple(uint8_t *readings, uint8_t first_addr) {
    readings[0] = spi_read(first_addr);
    for (int i = 1; i < 6; i++) {
        readings[i] = spi_transfer(0x00);
    }
}
