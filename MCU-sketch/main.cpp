#include <mcp2515.h>
#include <SPI.h>

#include "flash_utils.h"

struct can_frame can_message;
MCP2515 mcp2515(7);

uint8_t address_this_node;

void setup()
{
  pinMode(5, OUTPUT);
  digitalWrite(5, HIGH);
  MCUSR = 0;
 // Watchdog is on after flash, turning it off
  watchdog_off();

  Serial.begin(115200);

  SPI.begin();
  mcp2515.reset();

  while (mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ) < 0)
  {
    Serial.println("CAN BUS init Failed");
  };

  Serial.println("CAN BUS init succeeded");

  mcp2515.setNormalMode();

  //eeprom_update_byte((uint8_t*)J1939_SA_ADDR, 0x44);
  eeprom_update_byte((uint8_t*)FLASH_STATUS_ADDR, FLASH_INACTIVE);
  address_this_node = eeprom_read_byte((uint8_t*)J1939_SA_ADDR);
}

void loop()
{
  if (mcp2515.readMessage(&can_message) == MCP2515::ERROR_OK)
  {
    if ((can_message.can_id == (CAN_EFF_FLAG | CAN_ID_PROP_A | (address_this_node << 8) | FLASH_TOOL_ADDRESS)) && (can_message.data[0] == PA_FLASH_REQUEST))
    {
      Serial.println("Received flash request. Sending response..");
      create_ack(&can_message, address_this_node, CB_ACK);
      mcp2515.sendMessage(&can_message);

      eeprom_update_byte((uint8_t*)FLASH_STATUS_ADDR, FLASH_ACTIVE);
      eeprom_update_byte((uint8_t*)CAN_BAUDRATE_ADDR, CAN125kbps);

      Serial.println("Reseting MCU with watchdog reset");
      Serial.println("--------------------------------");
      MCUSR = 0;

      watchdog_on();
    
      while (1) {} // Wait for reset
    }
  }
}
