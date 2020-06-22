# Skeleton application runnning on MCU

Reads the canbus and if it receives a j1939 Proprietary A message it performs a watchdog reset. After reset stays in bootloader awaiting flash data.


## Dependency
Uses the MCP2515 arduino library written by loovee (luweicong@seeed.cc) for seeed studio.

https://github.com/autowp/arduino-mcp2515/
