# CAN-Flash
A custom verison of [Optiboot](https://github.com/Optiboot/optiboot) with support for flashing firmware over CAN with SAE J1939.<br/>
* Still supports flashing over USB 
* CRC32 for verifying firmware integrity.<br/>
* Only a successfull firmware update will launch the sketch.
# Bootloader
Possibilities to initialize CAN flash from Power-on reset and from a running sketch, see [description](USAGE.MD)

[comment]: <> (Modified optiboot bootloader to support flashing via CAN-bus)

# Flash tool
[Python script](Flash-tool) for flashing a specific node in a CAN-network using J1939 addressing scheme.

# Authors
Created by Magnus Roos[(miyago1)](https://github.com/miyago1) and Henrik Sellén Wikström[(henkewikstrom)](https://github.com/henkewikstrom)
