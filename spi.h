/* 
 * File:   spi.h
 * Author: EMBG2
 * Comments:
 * Revision history: 
 */

#ifndef SPI_H
#define	SPI_H

#include <xc.h> 
#include <stdint.h>

#ifdef	__cplusplus
extern "C" {
#endif

void spi_init(void);
uint8_t spi_transfer(uint8_t byte);
void spi_write(uint8_t reg, uint8_t value);
uint8_t spi_read(uint8_t reg);
void spi_read_multiple(uint8_t *readings, uint8_t first_addr);

#ifdef	__cplusplus
}
#endif

#endif	/* SPI_H */
