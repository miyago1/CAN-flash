#ifndef FLASH_UTILS_H
#define FLASH_UTILS_H

#include <avr/io.h>
#include <string.h>

#define FLASH_STATUS_ADDR           0x0000
#define CAN_BAUDRATE_ADDR           0x0001
#define J1939_SA_ADDR               0x0002
#define FLASH_ACTIVE                1
#define FLASH_INACTIVE              0

#define PA_FLASH_REQUEST            0x00
#define PA_FLASH_INFO               0x01
#define PA_FLASH_VERIFY             0x02
#define PA_FLASH_DONE               0x03

// MACROS
#define CHECK_BIT(var,pos)          ((var) & (1<<(pos)))
#define LEAST_BITS(n)               ( n & 0x0000FF)
#define MID_BITS(n)                 (( n & 0x00FF00 ) >> 8)
#define MOST_BITS(n)                (( n & 0xFF0000 ) >> 16)


void boot_program_page ( uint16_t page, uint8_t *buf, uint8_t data_size );
uint32_t crc32c(uint32_t crc, const unsigned char *buf, size_t len);

static inline void watchdog_reset()
{
    __asm__ __volatile__(
        "wdr\n");
}

void watchdog_off(void);

#endif // FLASH_UTILS_H
