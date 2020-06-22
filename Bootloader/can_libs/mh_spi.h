#ifndef _SPI_H_
#define _SPI_H_
#include <avr/io.h>

#define PORT_SPI    PORTB
#define DDR_SPI     DDRB
#define DD_MISO     DDB4
#define DD_MOSI     DDB3
#define DD_SS       DDB2
#define DD_SCK      DDB5

#define SPI_SPCR   0x50         // SPI enable, Master, MSB first, Clock phase = 0, clock polarity = 0, fosc/2
#define SPI_SPSR   0x01         // fosc/2

extern void spi_init();

inline static uint8_t spi_master_transmit(uint8_t data) {
    SPDR = data;

    __asm__ volatile("nop");
    while (!(SPSR & (1 << (SPIF)))) ; // wait
    return SPDR;
}


#endif /* _SPI_H_ */
