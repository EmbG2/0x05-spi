//#include "xc.h"
//#include "timer.h"
//#include <stdio.h>
//
//#define FCY         72000000 
//#define BAUDRATE    9600
//#define BRGVAL      (FCY / BAUDRATE) / 16 - 1
//
//#define START_READ  0x80
//#define START_WRITE 0x00
//#define GET_READING 0x00
//#define MAG_ADDR    0x40
//
//#define REG_POWER   0x4B
//#define REG_OPMODE  0x4C
//#define CHIP_ID_REG 0x40
//
//#define BUFFER_SIZE 4
//
//
//char uart_buffer[BUFFER_SIZE];
//int buffer_head = 0;
//int buffer_tail = 0;
//int missed_deadlines = 0;
//
//int16_t data;
//
//void uart_setup(int UART_n, int stop, int parity) {
//    switch (UART_n) {
//        case 1:
//            U1MODEbits.UARTEN = 0;
//            U1MODEbits.STSEL = stop;
//            U1MODEbits.PDSEL = parity;
//            U1MODEbits.BRGH = 0;
//            U1BRG = BRGVAL;
//            IFS0bits.U1RXIF = 0;
//            IEC0bits.U1RXIE = 1;
//            U1MODEbits.UARTEN = 1;
//            U1STAbits.UTXEN = 1;
//            break;
//        default:
//            return;
//    }
//}
//
//int main(void) {
//    ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = 0x0000;
//    TRISAbits.TRISA0 = 0;
//    TRISGbits.TRISG9 = 0;
//    TRISEbits.TRISE8 = 1;
//    TRISEbits.TRISE9 = 1;
//    
//    RPOR0bits.RP64R = 1;
//    RPINR18bits.U1RXR = 75;
//    uart_setup(1, 1, 0);
//    
//    U1TXREG = 'T'; 
//    
//    TRISAbits.TRISA1 = 1;           // MISO
//    TRISFbits.TRISF12 = 0;          // SCK
//    TRISFbits.TRISF13 = 0;          // MOSI
//    TRISBbits.TRISB3 = 0;           // CS2
//    LATBbits.LATB3 = 1;             //      disabled
//    TRISBbits.TRISB4 = 0;           // CS1
//    LATBbits.LATB4 = 1;             //      disabled
//    TRISDbits.TRISD6 = 0;           // CS3
//    LATDbits.LATD6 = 0;             //      enabled
//    
//    RPINR20bits.SDI1R = 0b0010001;
//    RPOR12bits.RP109R = 0b000101;
//    RPOR11bits.RP108R = 0b000110;
//    
//    SPI1CON1bits.MSTEN = 1; // Set SPI to master mode
//    SPI1CON1bits.MODE16 = 1; // Set to 16-bit mode
//    SPI1CON1bits.PPRE = 0; // Set 64:1 primary prescaler
//    SPI1CON1bits.SPRE = 7; // Set 1:1 secondary prescaler
//    SPI1CON1bits.CKP = 1; // Set clock idle state to high
//    SPI1STATbits.SPIROV = 0; // Clear overflow flag
//    SPI1STATbits.SPIEN = 1; // Enable SPI
//    
//    tmr_wait_ms(TIMER1, 100);
//    
//    // MAGNETOMETER SUSPENDED TO SLEEP
//    LATDbits.LATD6 = 0;
//    unsigned int trash = SPI1BUF; 
//    while (SPI1STATbits.SPITBF == 1);   // wait for transmit buffer to clear
//    SPI1BUF = 0x4B01;                   // 0b 0(write) 100 1011(address)
//    while (SPI1STATbits.SPITBF == 0);
//    tmr_wait_ms(TIMER1, 5);
//    trash = SPI1BUF;                    // trash dummy receive
//    LATDbits.LATD6 = 1;
//
//    tmr_wait_ms(TIMER1, 5);
//
//    // MAGNETOMETER SLEEP TO ACTIVE (normal)
//    LATDbits.LATD6 = 0;
//    while (SPI1STATbits.SPITBF == 1);   // wait for transmit buffer to clear
//    SPI1BUF = 0x4C00;                     
//    while (SPI1STATbits.SPITBF == 0);
//    trash = SPI1BUF;
//    LATDbits.LATD6 = 1;
//    tmr_wait_ms(TIMER1, 5);
//    
//    U1TXREG = 'H';
//    // MAGNETOMETER CHIP ID CHECK
//    LATDbits.LATD6 = 0;            // Select the chip
//    while (SPI1STATbits.SPITBF == 1);  
//    SPI1BUF = (0x40 | 0x80) << 8;  
//    while (!SPI1STATbits.SPIRBF);  
//    trash = SPI1BUF;   
//    while (SPI1STATbits.SPITBF == 1);
//    SPI1BUF = 0x0000;               
//    while (!SPI1STATbits.SPIRBF);   
//    int16_t value_from_chip = SPI1BUF; 
//    LATDbits.LATD6 = 1;            
//    tmr_wait_ms(TIMER1, 2);     
//    
//    // SEND CHIP ID OVER UART
//    U1TXREG = trash / 16 + '0';  // Send high nibble as ASCII
//    while (U1STAbits.UTXBF);
//    U1TXREG = trash % 16 + '0';  // Send low nibble as ASCII
//    while (U1STAbits.UTXBF);
//    
//    // Extract the correct 8-bit chip ID
//    uint8_t chip_id = (value_from_chip >> 8) & 0xFF; // Adjust based on your SPI data format
//    uint8_t high_nibble = (chip_id >> 4) & 0x0F;
//    uint8_t low_nibble  = chip_id & 0x0F;
//    U1TXREG = (high_nibble + '0');
//    while (U1STAbits.UTXBF);
//    U1TXREG = (low_nibble + '0');
//    while (U1STAbits.UTXBF);
//
//    U1TXREG = 'E';
//    while (U1STAbits.UTXBF);
//        
//    while(1);
//    
//    return 0;
//}

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
    TRISAbits.TRISA0 = 0;
    TRISGbits.TRISG9 = 0;
    TRISEbits.TRISE8 = 1;
    TRISEbits.TRISE9 = 1;
    
    RPOR0bits.RP64R = 1;
    RPINR18bits.U1RXR = 75;
    uart_setup(1, 1, 0);
    
    U1TXREG = 'S'; 
    while (U1STAbits.UTXBF);
    
    TRISAbits.TRISA1 = 1;
    TRISFbits.TRISF12 = 0;
    TRISFbits.TRISF13 = 0;
    TRISBbits.TRISB3 = 0;
    LATBbits.LATB3 = 1;
    TRISBbits.TRISB4 = 0;
    LATBbits.LATB4 = 1;
    TRISDbits.TRISD6 = 0;
    LATDbits.LATD6 = 0;
    
    RPINR20bits.SDI1R = 0b0010001;
    RPOR12bits.RP109R = 0b000101;
    RPOR11bits.RP108R = 0b000110;
    
    SPI1CON1bits.MSTEN = 1;  
    SPI1CON1bits.MODE16 = 0; 
    SPI1CON1bits.PPRE = 0;   
    SPI1CON1bits.SPRE = 7;   
    SPI1CON1bits.CKP = 1;    
    SPI1STATbits.SPIROV = 0; 
    SPI1STATbits.SPIEN = 1;  
    
    tmr_wait_ms(TIMER1, 100);
    
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
        
    while(1);
    
    return 0;
}