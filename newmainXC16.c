#include "xc.h"
#include "timer.h"

#define FCY 72000000 
#define BAUDRATE 9600
#define BRGVAL (FCY / BAUDRATE) / 16 - 1

#define BUFFER_SIZE 4

char uart_buffer[BUFFER_SIZE];
int buffer_head = 0;
int buffer_tail = 0;
int a           = 0;
int char_count  = 0;
int missed_deadlines    = 0;
int blink_enabled       = 1;


void uart_setup(int UART_n, int stop, int parity){
    switch (UART_n){
        case 1:
            // symbol configuration
            U1MODEbits.UARTEN   = 0;
            U1MODEbits.STSEL    = 1;        // stop bit
            U1MODEbits.PDSEL    = 0;        // parity
            U1MODEbits.ABAUD    = 0;        // auto-baud
            U1MODEbits.BRGH     = 0;        // speed mode
            U1BRG               = BRGVAL;   // BAUD Rate Setting for 9600
            // interrupts for receiving
            IFS0bits.U1RXIF = 0;
            IEC0bits.U1RXIE = 1;
            // activating transmission
            U1MODEbits.UARTEN   = 1;        // enable UART
            U1STAbits.UTXEN     = 1;        // enable u1tx
            break;
        case 2:
            // symbol configuration
            U2MODEbits.UARTEN   = 0;
            U2MODEbits.STSEL    = 1;        // stop bit
            U2MODEbits.PDSEL    = 0;        // parity
            U2MODEbits.ABAUD    = 0;        // auto-baud
            U2MODEbits.BRGH     = 0;        // speed mode
            U2BRG               = BRGVAL;   // BAUD Rate Setting for 9600
            // interrupts for receiving
            IFS1bits.U2RXIF = 0;
            IEC1bits.U2RXIE = 1;
            // activating transmission
            U2MODEbits.UARTEN   = 1;        // enable UART
            U2STAbits.UTXEN     = 1;        // enable u1tx
            break;
        default: 
            return;
    }
}

unsigned int spi_write(unsigned int data) {
    while (SPI1STATbits.SPITBF);  // Wait if buffer is full
    SPI1BUF = data;               // Send data
    while (!SPI1STATbits.SPIRBF); // Wait for response
    return SPI1BUF;               // Read received data
}

uint8_t SPI_Read() {
    while (!SPI1STATbits.SPIRBF); // Wait for the data to be received
    return SPI1BUF;               // Read received data from the SPI buffer
}

void set_magnetometer_mode(unsigned int reg, unsigned int mode) {
    MAG_CS = 0;
    spi_write(reg & 0x7F);  // Write command (MSB=0)
    spi_write(mode);
    MAG_CS = 1;
}


int main(void) {
    
    // LED/BUTTON CONFIGURATION
    ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = 0x0000;
    TRISAbits.TRISA0 = 0;
    TRISGbits.TRISG9 = 0;
    TRISEbits.TRISE8 = 1;
    TRISEbits.TRISE9 = 1;
    
    // UART CONFIGURATION
    // assign to the appropriate pins (TX 64, RX 75)
    RPOR0bits.RP64R     = 1;        // uart 1: 1, uart 2: 3
    RPINR18bits.U1RXR   = 75;
    uart_setup(1, 1, 0);
    
    // something sent to indicate start
    U1TXREG             = 'S'; // TODO remove
    
    TRISBbits.TRISB3 = 0;  // CS as output
    LATBbits.LATB3 = 1;    // NOT SURE
    
    TRISAbits.TRISA1    = 1;
    TRISFbits.TRISF12   = 0;
    TRISFbits.TRISF13   = 0;
    RPINR20bits.SDI1R   = 0b0010001; // MISO(SDA) input  to RP17
    RPOR12bits.RP109R   = 0b000101;  // MOSI(SDO) output PPS
    RPOR11bits.RP108R   = 0b000110;  // SCK       output PPS 
    
    SPI1CON1bits.MSTEN = 1;      // Master Mode
    SPI1CON1bits.CKE = 1;        // Data is transmitted on the rising edge of SCK
    SPI1CON1bits.CKP = 1;        // Clock Polarity (idle high)

    SPI1STATbits.SPIEN = 1;      // Enable SPI module    
    
    // read chip id
    LATBbits.LATB3 = 0;
    spi_write(0x40 | 0x80);  // Read command (MSB=1)
    unsigned int chipid = spi_write(0x00); // Clock out zeros to receive data
    LATBbits.LATB3 = 1;
    
    return 0;
}


/*

void __attribute__((__interrupt__)) _U1RXInterrupt(void) {
    // TODO check whether to read first or clear-overflow-flag first
    // == Critical Section ==
    char rec = U1RXREG; 
    if (U1STAbits.OERR) {  
        U1STAbits.OERR = 0;
        char_count = 0;
    }
    char_count++;
    IFS0bits.U1RXIF = 0;
    
    // == Non-critical Section ==
    // TODO move to a function in main or something, out of interrupt
    uart_buffer[buffer_tail] = rec;
    buffer_tail = (buffer_tail + 1) % BUFFER_SIZE;
    if (buffer_tail >= 3) {
        int i = (buffer_tail - 3 + BUFFER_SIZE) % BUFFER_SIZE;
        if (uart_buffer[i] == 'L' &&
            uart_buffer[(i + 1) % BUFFER_SIZE] == 'D' &&
            uart_buffer[(i + 2) % BUFFER_SIZE] == '1') {
            LATAbits.LATA0 ^= 1;            // toggle LED1
            buffer_head = buffer_tail = 0;  // reset buffer
        }
        if (uart_buffer[i] == 'L' &&
            uart_buffer[(i + 1) % BUFFER_SIZE] == 'D' &&
            uart_buffer[(i + 2) % BUFFER_SIZE] == '2') {
            blink_enabled ^= 1;             // toggle LED2-blinking
            buffer_head = buffer_tail = 0;  // reset buffer
        }
    }
}

void __attribute__((__interrupt__, auto_psv)) _T1Interrupt(void) {
    IFS0bits.T1IF = 0;
    a ++;
    if (blink_enabled && a >= 20) {
        a = 0;
        LATGbits.LATG9 ^= 1;
    }
}

void __attribute__((__interrupt__, auto_psv)) _INT1Interrupt(void) {
    IFS1bits.INT1IF = 0;            // Reset the flag of the interrupt
    IFS0bits.T3IF = 0;              // Reset the interrupt's flag
    IEC0bits.T3IE = 1;              // Activate TIMER1's interrupt
    tmr_turn(TIMER3, 1);
}

void __attribute__((__interrupt__, auto_psv)) _INT2Interrupt(void) {
    IFS1bits.INT2IF = 0;            // Reset the flag of the interrupt
    IFS0bits.T3IF = 0;              // Reset the interrupt's flag
    IEC0bits.T3IE = 1;              // Activate TIMER1's interrupt
    tmr_turn(TIMER3, 1);
}

void __attribute__((__interrupt__, auto_psv)) _T3Interrupt(void) {
    IFS0bits.T3IF = 0;              // Reset the flag of the interrupt
    tmr_turn(3, 0);
    if (PORTEbits.RE8){
        U1TXREG             = 'C';
        U1TXREG             = '=';
        U1TXREG             = '0' + char_count / 10;
        U1TXREG             = '0' + char_count % 10;
    }
    if (PORTEbits.RE9){
        U1TXREG             = 'D';
        U1TXREG             = '=';
        U1TXREG             = '0' + missed_deadlines / 10;
        U1TXREG             = '0' + missed_deadlines % 10;
    }
    IEC0bits.T3IE = 0;
}
*/
