/**
 ***********************************************************************************************
 @brief Flash utilities
 @details Utilities for flashing the MCU via CAN.
 @file flash_utils.c
 @author Magnus Roos and Henrik Sellén Wikström
 @version 1.0
 @date 29-May-2020
 ***********************************************************************************************
 */

#include <avr/eeprom.h>
#include <avr/boot.h>
#include <avr/interrupt.h>

#include "flash_utils.h"

#define POLY 0xedb88320

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
    int k;

    crc = ~crc;
    while (len--) {
        crc ^= *buf++;
        for (k = 0; k < 8; k++)
            crc = crc & 1 ? (crc >> 1) ^ POLY : crc >> 1;
    }
    return ~crc;
}

void watchdog_off(void)
{
    cli();
    watchdog_reset();
    /* Clear WDRF in MCUSR */
    MCUSR &= ~(1<<WDRF);
    /* Write logical one to WDCE and WDE */
    /* Keep old prescaler setting to prevent unintentional time-out */
    WDTCSR |= (1<<WDCE) | (1<<WDE);
    /* Turn off WDT */
    WDTCSR = 0x00;
    sei();
}
