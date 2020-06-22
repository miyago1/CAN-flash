#ifndef FLASH_UTILS_H
#define FLASH_UTILS_H

#include <avr/io.h>
#include <string.h>

#define FLASH_STATUS_ADDR       0x0000
#define CAN_BAUDRATE_ADDR       0x0001
#define CAN_DA_ADDR             0x0002
#define CAN_SA_ADDR             0x0003
#define FLASH_ACTIVE            1
#define FLASH_INACTIVE          0

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


void boot_program_page ( uint16_t page, uint8_t *buf, uint8_t data_size );
uint32_t crc32c(uint32_t crc, const unsigned char *buf, size_t len);

static inline void watchdog_reset()
{
  __asm__ __volatile__(
      "wdr\n");
}

// void watchdog_off(void);




// LED FOR DEBUGGING

void led_d5_init();
void led_d5_low();
void led_d5_high();


void led_d4_init();
void led_d4_low();
void led_d4_high();
void led_d4_toggle();


#endif // FLASH_UTILS_H
