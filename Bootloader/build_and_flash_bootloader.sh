#!/bin/sh
# needs to be run in optiboot folder

USB_PORT=/dev/ttyUSB1
PROGRAMMER=stk500v2
MCU=atmega328p
OPTIBOOT_DIR=${PWD}
CAN_LIBS_DIR="${OPTIBOOT_DIR}/can_libs/"
AVRDUDE_CONF="/usr/share/arduino/hardware/tools/avrdude.conf"

# CAN and J1939 libraries
echo "Compiling CAN and J1939 libraries"
make clean -C ${CAN_LIBS_DIR}
make -C ${CAN_LIBS_DIR}
echo "Done compiling libraries"

echo "Compiling OPTIBOOT"
# optiboot
make clean
make can_atmega328 boot_timeout=2
echo "Completed compilation, time to flash!"

/usr/bin/avrdude -v -p${MCU} -c${PROGRAMMER} -P${USB_PORT} -Uflash:w:optiboot_atmega328.hex:i -Ulock:w:0xFF:m

echo "Information about size of code"
avr-size optiboot_atmega328.elf


