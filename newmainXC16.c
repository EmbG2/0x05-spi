#include "xc.h"
#include "timer.h"

#define FCY         72000000 
#define BAUDRATE    9600
#define BRGVAL      (FCY / BAUDRATE) / 16 - 1

#define START_READ  0x80
#define START_WRITE 0x00
#define GET_READING 0x00
#define MAG_ADDR    0x40

#define REG_POWER   0x4B
#define REG_OPMODE  0x4C
#define CHIP_ID_REG 0x40

#define BUFFER_SIZE 4

char uart_buffer[BUFFER_SIZE];
int buffer_head = 0;
int buffer_tail = 0;
int missed_deadlines = 0;

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

unsigned int spi_write(unsigned int data) {
    while (SPI1STATbits.SPITBF);
    SPI1BUF = data;
    while (!SPI1STATbits.SPIRBF);
    return SPI1BUF;
}

void set_magnetometer_mode(unsigned int reg, unsigned int mode) {
    LATBbits.LATB3 = 0;
    spi_write(reg & 0x7F);
    spi_write(mode);
    LATBbits.LATB3 = 1;
}

int main(void) {
    ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = 0x0000;
    TRISAbits.TRISA0 = 0;
    TRISGbits.TRISG9 = 0;
    TRISEbits.TRISE8 = 1;
    TRISEbits.TRISE9 = 1;
    
    RPOR0bits.RP64R = 1;
    RPINR18bits.U1RXR = 75;
    uart_setup(1, 1, 0);
    
    U1TXREG = 'S'; 
    
    TRISBbits.TRISB3 = 0;
    LATBbits.LATB3 = 1;
    TRISBbits.TRISB4 = 0;
    LATBbits.LATB4 = 0;
    TRISDbits.TRISD6 = 0;
    LATDbits.LATD6 = 0;
    
    TRISAbits.TRISA1 = 1;
    TRISFbits.TRISF12 = 0;
    TRISFbits.TRISF13 = 0;
    RPINR20bits.SDI1R = 0b0010001;
    RPOR12bits.RP109R = 0b000101;
    RPOR11bits.RP108R = 0b000110;
    
    SPI1CON1bits.MSTEN = 1;
    SPI1CON1bits.CKE = 0;
    SPI1CON1bits.CKP = 1;
    SPI1STATbits.SPIEN = 1;
    
    set_magnetometer_mode(REG_POWER, 0x01);
    tmr_wait_ms(TIMER1, 3);
    set_magnetometer_mode(REG_OPMODE, 0x00);
    tmr_wait_ms(TIMER1, 3);
    
    LATBbits.LATB3 = 0;
//    spi_write(CHIP_ID_REG | START_READ);
    spi_write(START_READ | CHIP_ID_REG);
    unsigned int chipid_msb = spi_write(GET_READING);
    unsigned int chipid_lsb = spi_write(GET_READING);
    LATBbits.LATB3 = 1;
    
//    U1TXREG = chipid_msb;
//    while (U1STAbits.UTXBF);
//    U1TXREG = chipid_lsb;
    
    unsigned int chipid = (chipid_msb << 8) | chipid_lsb;
    
    U1TXREG = (chipid >> 8) & 0xFF;
    while (U1STAbits.UTXBF);
    U1TXREG = chipid & 0xFF;
    
    while(1);
    
    return 0;
}