/* 
 * File:   
 * Author: EMBG2
 * Comments:
 * Revision history: 
 */

#ifndef UART_H
#define	UART_H

#include <xc.h> // include processor files - each processor file is guarded.  

#define UART_1 1
#define UART_2 2

void UART_Init(unsigned char uart);
void send_uart_char(unsigned char uart, char data);
void send_uart_string(const char *buffer);

// interrupt function declarations
extern void __attribute__((__interrupt__, auto_psv)) _U1RXInterrupt(void);
extern void __attribute__((__interrupt__, auto_psv)) _U2RXInterrupt(void);

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* UART_H */

