#!/bin/sh
# A comment

VCAN_INTERFACE_NAME="vcan0"
echo Starting virtual can

sudo modprobe vcan

# Creating interface
sudo ip link add dev $VCAN_INTERFACE_NAME type vcan
# Start interface
sudo ip link set $VCAN_INTERFACE_NAME up

#echo Starting candump
#candump $VCAN_INTERFACE_NAME

echo Starting socketcand listener
socketcand -l enp0s31f6 -i vcan0

