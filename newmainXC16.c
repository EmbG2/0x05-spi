#include "xc.h"
#include "timer.h"
#include <stdio.h>

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

int16_t magx, tempm, templ;
#define MAG_CS LATDbits.LATD6
#define CS LATDbits.LATD6

int16_t data, data2;
char buff[10];


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

int16_t spi_rewr(int16_t reg){
    while (SPI1STATbits.SPITBF);   
    SPI1BUF = reg;
    while (!SPI1STATbits.SPIRBF);  
    int16_t ret = SPI1BUF;
    return ret;
}

int main(void) {
    ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = 0x0000;
    
    // SETUP UART-1
    RPOR0bits.RP64R = 1;
    RPINR18bits.U1RXR = 75;
    uart_setup(1, 1, 0);
    
    U1TXREG = 'S'; 
    while (U1STAbits.UTXBF);
    
    // SETUP SPI-1
    TRISAbits.TRISA1 = 1;                   // MISO
    TRISFbits.TRISF12 = 0;                  // SCK
    TRISFbits.TRISF13 = 0;                  // MOSI
    TRISBbits.TRISB3 = 0;                   // CS2
    LATBbits.LATB3 = 1;                     //  disabled
    TRISBbits.TRISB4 = 0;                   // CS1
    LATBbits.LATB4 = 1;                     //  disabled
    TRISDbits.TRISD6 = 0;                   // CS3
    LATDbits.LATD6 = 1;                     //  enabled
    
    RPINR20bits.SDI1R = 0b0010001;          // MISO (SDI1) - RPI17
    RPOR12bits.RP109R = 0b000101;           // MOSI (SDO1) - RF13
    RPOR11bits.RP108R = 0b000110;           // SCK1
    
    SPI1CON1bits.MSTEN = 1;                 // master mode
//    SPI1CON1bits.MODE16 = 0;                // use 8-bit mode
    SPI1CON1bits.MODE16 = 1;                // use 16-bit mode
    SPI1CON1bits.PPRE = 0;                  // 64:1 primary prescaler
    SPI1CON1bits.SPRE = 7;                  // 1:1 secondary prescaler
    SPI1CON1bits.CKP = 1;                   // idle high
    SPI1STATbits.SPIROV = 0;                // clear overflow flag
    SPI1STATbits.SPIEN = 1;                 // enable spi
    
    tmr_wait_ms(TIMER1, 100);
    
    
    // chip select is active low. to read/write, clear chip select, do the job, 
    // and disable it.
    
    /* to READ from SPI 1 1000010 00000000 00000000
     * start a sequence with a byte write: MSB is 1 + ADDR (7 bits) 
     * then send a dummy value 0x00 to generate clock
     * and read the returning value  
     */
    /* to WRITE from SPI
     * start a sequence with a byte write: MSB is 0 + ADDR (7 bits)
     * then send the intended value
     */
    
    // anything sent is answered with a response. if the response is not needed, 
    // clear it (trash here).
    
    // MAGNETOMETER SUSPENDED TO SLEEP
    CS = 0;                 // enable
    unsigned int trash = spi_rewr(0x4B01);
    CS = 1;                 // disable
    tmr_wait_ms(TIMER1, 5);
    
    // MAGNETOMETER SLEEP TO ACTIVE 
    CS = 0;
    trash = spi_rewr(0x4C00);
    CS = 1;
    tmr_wait_ms(TIMER1, 5);
        
    // MAGNETOMETER REQUEST FOR CHIP ID
    CS = 0;             
    int16_t chip_id = spi_rewr(0x4000 | 0x8000);
    CS = 1;            
    
    tmr_wait_ms(TIMER1, 10);

    // SEND MAGNETOMETER'S CHIP ID OVER UART1
    U1TXREG = chip_id / 16 + '0';
    while (U1STAbits.UTXBF);
    U1TXREG = chip_id % 16 + '0';
    while (U1STAbits.UTXBF);
    U1TXREG = 'E';
    while (U1STAbits.UTXBF);
    
    while(1){
        // MAGNETOMETER X: Request 0x42/read and Request 0x43/read
        CS = 0;
        data = spi_rewr(0x4200 | 0x8000);
        CS = 1;
        data  = (data&0xFF) >>3;

        CS = 0;
        data2 = spi_rewr(0x4300 | 0x8000);
        CS = 1;
        data2 = (data2 & 0xFF)<<5;
        
        int16_t x_data = (data2|data)<<3;
        x_data = x_data / 8;

        tmr_wait_ms(TIMER1, 10);

        sprintf(buff, "X:%d", x_data);
        for (char i = 0; i < 10; i++) {
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
    };
    
    return 0;
}