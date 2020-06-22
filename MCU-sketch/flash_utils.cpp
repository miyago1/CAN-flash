#include "flash_utils.h"

#include <avr/interrupt.h>

void watchdog_off(void)
{
  cli();
  watchdog_reset();
  /* Clear WDRF in MCUSR */
  MCUSR &= ~(1 << WDRF);
  /* Write logical one to WDCE and WDE */
  /* Keep old prescaler setting to prevent unintentional time-out */
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  /* Turn off WDT */
  WDTCSR = 0x00;
  sei();
}

void watchdog_on()
{
  // Watchdog
  cli();
  watchdog_reset();
  // Enable watchdog
  WDTCSR = _BV(WDCE) | _BV(WDE);
  WDTCSR = _BV(WDE);
  sei();
}

void create_ack(struct can_frame *can_message, uint8_t node_address, uint8_t control_byte)
{
  can_message->data[4] = (can_message->can_id & 0xFF);
  can_message->can_id = CAN_EFF_FLAG | 0x18E8FF00 | node_address;
  can_message->can_dlc = 8;
  can_message->data[1] = can_message->data[0]; // prop control byte = ack group function
  can_message->data[0] = control_byte;         // ack, nack, access denied or busy
  can_message->data[2] = SAE_RESERVED;
  can_message->data[3] = SAE_RESERVED;
  can_message->data[5] = LEAST_BITS(PGN_PROP_A); // prop A pgn
  can_message->data[6] = MID_BITS(PGN_PROP_A); // prop A pgn
  can_message->data[7] = MOST_BITS(PGN_PROP_A); // prop A pgn
}
