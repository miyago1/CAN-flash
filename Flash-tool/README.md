# Flash tool python script

Sends the data in a intelhex file through the CANbus using J1939-21 standard

## Usage:

Run main.py with following arguments

Mandatory

-d [Node-ID to flash]
-f [Intel Hex-file to flash]

Optional

-i [CAN-interface] "default can0"
-s [Node-ID of flashtool] "default 0x7F"


## Python packages

Intelhex: https://pypi.org/project/IntelHex/

Progressbar2: https://pypi.org/project/progressbar2/

Python-can: https://pypi.org/project/python-can/

Python-statemachine: https://pypi.org/project/python-statemachine/


Use requirements.txt to install: pip install -r requirements.txt
