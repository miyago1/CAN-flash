#!/bin/sh

#flash size 930 bytes, 8 pages, app id 3256, app version 52 

CAN_INTERFACE=vcan0

# 00EA00
REQUEST=59904
# 00E800
ACK=59392
# reset request pgn, 00FF13
RESET_REQUEST=65299
# 00FF14
PAGE_INFO=65300
# 00FF15
PAGE_DATA=65301


######################
#INITIERING#########
####################

# skicka reset request
cansend ${CAN_INTERFACE} 18EA6633#13FF00

# svar ACK SA=66

cansend ${CAN_INTERFACE} 18E8FF66#0001FFFF3313FF00

# från bootloader ACK

cansend ${CAN_INTERFACE} 18E8FF66#0002FFFF3313FF00

# skicka flash info

cansend ${CAN_INTERFACE} 18EF6633#A20308B80C34FFFF

# Skickar ack på flash info

cansend ${CAN_INTERFACE} 18E8FF66#0003FFFF3300EF00

######################
# Första pagen
######################

cansend ${CAN_INTERFACE} 18E8FF66#0002FFFF3313FF00

cansend vcan0 1CEC3366#110101FFFF15FF00
