# Modified optiboot bootloader
Original bootloader: https://github.com/Optiboot/optiboot
Modified to accept flashing via CAN-bus and J1939-21 standard

# Compile and flash

Modify build_and_flash_bootloader.sh to use right usb port etc.
Run it.

# Requirements

ATmega 328p microcontroller, Arduino Uno or Nano for example.
Some kind of ICSP, AVRISP, buspirate. or https://github.com/nickgammon/arduino_sketches
