#ifndef FLASH_UTILS_H
#define FLASH_UTILS_H

#include <avr/io.h>
#include <avr/eeprom.h>
#include "can.h"

#define FLASH_STATUS_ADDR           0x0000
#define CAN_BAUDRATE_ADDR           0x0001
#define J1939_SA_ADDR               0x0002
#define FLASH_ACTIVE                1
#define FLASH_INACTIVE              0

#define PA_FLASH_REQUEST            0x00
#define PA_FLASH_INFO               0x01
#define PA_FLASH_VERIFY             0x02
#define PA_FLASH_DONE               0x03

#define CB_ACK                      0x00
#define CB_NACK                     0x01
#define CB_ACCESS_DENIED            0x02
#define CB_CANNOT_RESPOND           0x03

#define CAN125kbps                  3
#define CAN500kbps                  6

#define FLASH_TOOL_ADDRESS          0x7F

#define CAN_ID_PROP_A               0x18EF0000
#define PGN_PROP_A                  61184

#define SAE_RESERVED                0XFF

// MACROS
#define CHECK_BIT(var,pos)          ((var) & (1<<(pos)))
#define LEAST_BITS(n)               ( n & 0x0000FF)
#define MID_BITS(n)                 (( n & 0x00FF00 ) >> 8)
#define MOST_BITS(n)                (( n & 0xFF0000 ) >> 16)



static inline void watchdog_reset()
{
  __asm__ __volatile__(
      "wdr\n");
}

void watchdog_off(void);
void watchdog_on(void);
void create_ack(struct can_frame* can_message, uint8_t node_address, uint8_t control_byte);

#endif
