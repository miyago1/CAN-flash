#include <avr/interrupt.h>
#include "mh_spi.h"

void spi_init()
// Initialize pins for spi communication
{
    uint8_t sreg = SREG;
    cli();
    /* Set MOSI and SCK output, all others input */
    DDR_SPI &= ~(_BV(DD_MOSI)|_BV(DD_MISO)|_BV(DD_SS)|_BV(DD_SCK));
    // Define the following pins as output
    DDR_SPI |= _BV(DD_MOSI)|_BV(DD_SS)|_BV(DD_SCK);
    
    /* ATmega328p requires default slave select (PORTB PIN2)
     * to be configred as output no matter what slave select 
     * pin is used */
    
    DDRD |= 1 << DDD7;   // Chip Select (D7) as output 
    PORTD |= 1 << PIN7;  // SS high

    /* Hastighet: 10Mhz, MSB first, etc
     * SPCR – SPI Control Register: 0x51 bits: 01010001
        Bit 7 – SPIE: SPI Interrupt Enable: 0
        Bit 6 – SPE: SPI Enable: 1
        Bit 5 – DORD: Data Order: 0
        Bit 4 – MSTR: Master/Slave Select: 1
        Bit 3 – CPOL: Clock Polarity: 0
        Bit 2 – CPHA: Clock Phase: 0
        Bit 1 – SPR1: 0
        Bit 0 – SPR0: 1
     * SPSR – SPI Status Register: 0x00 bits: 00000000
        Bit 7 – SPIF: SPI Interrupt Flag: 0
        Bit 6 – WCOL: Write COLlision Flag: 0
        Bit 0 – SPI2X: Double SPI Speed Bit: 0
    */

    SPCR = SPI_SPCR;    
    SPSR = SPI_SPSR;

    SREG = sreg;    
}


