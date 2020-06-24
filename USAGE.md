# Optiboot with CAN-flash-support

## Installation of new bootloader
1. Inside Bootloader/optibootv2 run **make -C can_libs/** to create object files and then **make can_atmega328** to generate HEX file. 
2. Burn optiboot_atmega328.hex to MCU and set High-Fuse to 0xD0. Example guide https://learn.sparkfun.com/tutorials/installing-an-arduino-bootloader/all.
3. Modify the sketch [store_node_id.ino](Flashtool/MCU-sketch/store_node_id.ino) with the Node-ID that should be assigned to the target node, upload it to node with Arduino IDE and let it run. 

## Procedure of flashing over CAN
### The CAN-flash can be initialized in two ways
1. During Power-on-reset, the node will listen after CAN-update requests during 2 seconds before it starts the sketct located in flashmemory.
2. For starting flash from running sketch, the sketch need to contain initialization code for CAN-flash. Example sketch which only listens for flash requests can be found in [main.cpp](Flashtool/MCU-sketch/main.cpp). 


## Usage of flash-tool
Run Flashtool/flash-tool/main.py with following arguments
### Mandatory
-d [Node-ID to flash]<br/>
-f [Intel Hex-file to flash]

### Optional
-i [CAN-interface] "default can0" <br/>
-s [Node-ID of flashtool] "default 0x7F"


