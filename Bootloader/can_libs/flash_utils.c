#include "flash_utils.h"

#include <avr/eeprom.h>
#include <avr/boot.h>
#include <avr/interrupt.h>

void boot_program_page ( uint16_t page, uint8_t *buf, uint8_t data_size )
{
    uint16_t i;
    uint8_t sreg;
    // Disable interrupts.
    sreg = SREG;
    cli();
    eeprom_busy_wait();
    boot_page_erase ( page );
    boot_spm_busy_wait ();      // Wait until the memory is erased.
    for ( i=0; i<data_size; i+=2 ) {
        // Set up little-endian word.
        uint16_t w = *buf++;
        w += ( *buf++ ) << 8;

        boot_page_fill ( page + i, w );
    }
    boot_page_write ( page );   // Store buffer in flash page.
    boot_spm_busy_wait();       // Wait until the memory is written.
    // Reenable RWW-section again. We need this if we want to jump back
    // to the application after bootloading.
    boot_rww_enable ();
    // Re-enable interrupts (if they were ever enabled).
    SREG = sreg;
}

uint32_t crc32c(uint32_t crc, const unsigned char *buf, size_t len)
{
#define POLY 0xedb88320
    int k;

    crc = ~crc;
    while (len--) {
        crc ^= *buf++;
        for (k = 0; k < 8; k++)
            crc = crc & 1 ? (crc >> 1) ^ POLY : crc >> 1;
    }
    return ~crc;
}

// void watchdog_off(void)
// {
//     cli();
//     watchdog_reset();
//     /* Clear WDRF in MCUSR */
//     MCUSR &= ~(1<<WDRF);
//     /* Write logical one to WDCE and WDE */
//     /* Keep old prescaler setting to prevent unintentional time-out */
//     WDTCSR |= (1<<WDCE) | (1<<WDE);
//     /* Turn off WDT */
//     WDTCSR = 0x00;
//     sei();
// }

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

void led_d5_init()
{
    // Port B5 as output
    DDRD = (1 << DDD5);
    // PIN B5 LOW
    PORTD &= ~(1 << PD5);
}

void led_d5_low()
{
    // PIN B5 LOW
    PORTD &= ~(1 << PD5);
}


void led_d5_high()
{
    // PIN B5 HIGH
    PORTD |= 1 << PD5;
}


void led_d4_init()
{
    // Port B5 as output
    DDRD = (1 << DDD4);
    // PIN B5 LOW
    PORTD &= ~(1 << PD4);
}

void led_d4_low()
{
    // PIN B5 LOW
    PORTD &= ~(1 << PD4);
}


void led_d4_high()
{
    // PIN B5 HIGH
    PORTD |= 1 << PD4;
}

void led_d4_toggle() {
    PORTD ^= (1 << PD4);

}

