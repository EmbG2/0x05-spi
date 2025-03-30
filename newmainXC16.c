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
    LATDbits.LATD6 = 0;                     //  enabled
    
    RPINR20bits.SDI1R = 0b0010001;          // MISO (SDI1) - RPI17
    RPOR12bits.RP109R = 0b000101;           // MOSI (SDO1) - RF13
    RPOR11bits.RP108R = 0b000110;           // SCK1
    
    SPI1CON1bits.MSTEN = 1;                 // master mode
    SPI1CON1bits.MODE16 = 0;                // use 8-bit mode
    SPI1CON1bits.PPRE = 0;                  // 64:1 primary prescaler
    SPI1CON1bits.SPRE = 7;                  // 1:1 secondary prescaler
    SPI1CON1bits.CKP = 1;                   // idle high
    SPI1STATbits.SPIROV = 0;                // clear overflow flag
    SPI1STATbits.SPIEN = 1;                 // enable spi
    
    tmr_wait_ms(TIMER1, 100);
    
    
    // chip select is active low. to read/write, clear chip select, do the job, 
    // and disable it.
    
    /* to READ from SPI
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
    LATDbits.LATD6 = 0;
    unsigned int trash = SPI1BUF; 
    while (SPI1STATbits.SPITBF);
    SPI1BUF = 0x4B01;
    while (!SPI1STATbits.SPIRBF);
    trash = SPI1BUF;
    LATDbits.LATD6 = 1;
    tmr_wait_ms(TIMER1, 5);
    
    // MAGNETOMETER SLEEP TO ACTIVE 
    LATDbits.LATD6 = 0;
    while (SPI1STATbits.SPITBF);
    SPI1BUF = 0x4C00;                     
    while (!SPI1STATbits.SPIRBF);
    trash = SPI1BUF;
    LATDbits.LATD6 = 1;
    tmr_wait_ms(TIMER1, 5);
        
    // MAGNETOMETER REQUEST FOR CHIP ID
    LATDbits.LATD6 = 0;            
    while (SPI1STATbits.SPITBF);   
    SPI1BUF = CHIP_ID_REG | 0x80;
    while (!SPI1STATbits.SPIRBF);  
    trash = SPI1BUF;
    while (SPI1STATbits.SPITBF);
    SPI1BUF = 0x00;  
    while (!SPI1STATbits.SPIRBF);
    uint8_t chip_id = SPI1BUF;  
    LATDbits.LATD6 = 1;            
    tmr_wait_ms(TIMER1, 2);
    
    // SEND MAGNETOMETER'S CHIP ID OVER UART1
    U1TXREG = chip_id / 16 + '0';
    while (U1STAbits.UTXBF);
    U1TXREG = chip_id % 16 + '0';
    while (U1STAbits.UTXBF);
    U1TXREG = 'E';
    while (U1STAbits.UTXBF);
    
    // MAGNETOMETER X: Request 0x42 and read two times (for 0x42 and 0x43)
    LATDbits.LATD6 = 0;            
    while (SPI1STATbits.SPITBF);   
    SPI1BUF = 0x42 | 0x80;
    while (!SPI1STATbits.SPIRBF);  
    trash = SPI1BUF;
    while (SPI1STATbits.SPITBF);
    SPI1BUF = 0x00;  
    while (!SPI1STATbits.SPIRBF);
    uint16_t magx_low  = SPI1BUF;
    while (SPI1STATbits.SPITBF);
    SPI1BUF = 0x00;  
    while (!SPI1STATbits.SPIRBF);
    uint16_t magx_high = SPI1BUF;
    LATDbits.LATD6 = 1;            
    tmr_wait_ms(TIMER1, 2);

    U1TXREG = magx_high;
    while (U1STAbits.UTXBF);
    U1TXREG = magx_low;
    while (U1STAbits.UTXBF);
    U1TXREG = 'E';
    while (U1STAbits.UTXBF);
    
    templ = (magx_low & 0x00F8);
    tempm = (magx_high | 0x0000) << 8;
    magx = (tempm | templ) >> 3;
    
    U1TXREG = magx / 256;
    while (U1STAbits.UTXBF);
    U1TXREG = magx % 256;
    while (U1STAbits.UTXBF);
    
    while(1);
    
    return 0;
}