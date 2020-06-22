#!/usr/bin/python
from flasher import Flasher
from mh_j1939 import J1939Client
from pgn import PGN

import time
import argparse


def auto_int(x):
    return int(x, 0)


parser = argparse.ArgumentParser()
parser.add_argument('-i', default='can0', help='CAN interface')
parser.add_argument('-d', required=True, type=auto_int, help='Destination address for node')
parser.add_argument('-s', default=0x7F, type=auto_int, help='Source address for flash tool (default is 0x7F)')
parser.add_argument('-f', required=True, help='Intelhex file to flash')
# sudo ip link set can0 type can bitrate 125000
# sudo ip link set can0 up
args = parser.parse_args()

THIS_NODE = args.s
CTS_LIMIT = 0xFF
FLASH_TARGET = args.d

j1939 = J1939Client(can_interface=args.i, node_address=THIS_NODE, state='start')
j1939.set_one_to_one_filter(FLASH_TARGET)
updater = Flasher(j1939)
j1939.pgn_list = [PGN.PROP_A, PGN.ACK, PGN.REQUEST, PGN.TP_CM, PGN.TP_DT]

updater.hex_file = args.f
#updater.hex_file = "/home/magnus/Documents/PlatformIO/Projects/CAN-nano/.pio/build/nanoatmega328new/firmware.hex"
updater.print_info()

if __name__ == '__main__':

    # Sending request to flash
    j1939.send_prop_a(update_phase=0,
                      flash_size=updater.flash_size,
                      page_count=updater.page_count,
                      extra_info=0x12345678)
    time.sleep(1)
    updater.request_flash()

    while True:
        if updater.is_wait_page_write:
            updater.pages_sent += 1
            if updater.pages_sent == updater.page_count:
                j1939.send_prop_a(update_phase=2,
                                  flash_size=updater.flash_size,
                                  page_count=updater.page_count,
                                  extra_info=updater.crc)
                updater.update_sent()
            else:
                j1939.tp_cm_initiate_connection(data=updater.page_data, pgn=0x00EF00, cts_limit=CTS_LIMIT)
                j1939.send_cm_rts_pdu()
                updater.next_page()

        while updater.is_sending_packets:
            j1939.send_cm_data_pdu()
            j1939.tp_cm_packets_sent += 1
            j1939.tp_cm_next_packet += 1
            j1939.tp_cm_cts_limit -= 1
            if j1939.tp_cm_cts_limit == 0 or j1939.tp_cm_packets_sent == j1939.tp_cm_packets:
                updater.send_limit_reached()

        # Listening for new can messages
        received_msg = j1939.read_can_bus()
        if received_msg is None:
            continue
        # Transport Protocol frame check
        if received_msg.pgn.is_connection_management and received_msg.source_address == j1939.tp_cm_address:
            if received_msg.data[0] == 0x11:
                if received_msg.data[2] <= j1939.tp_cm_packets_sent + 1:
                    if updater.packets_sent == 0:
                        j1939.tp_cm_connected = True
                        if received_msg.data[1] > 0:
                            j1939.tp_cm_cts_limit = received_msg.data[1]
                            j1939.tp_cm_next_packet = received_msg.data[2]
                        elif received_msg.data[1] == 0:
                            pass
                        updater.clear_to_send()
                    elif j1939.tp_cm_connected:
                        if received_msg.data[1] > 0:
                            j1939.tp_cm_cts_limit = received_msg.data[1]
                            j1939.tp_cm_next_packet = received_msg.data[2]
                        elif received_msg.data[1] == 0:
                            pass
                else:
                    pass
            elif received_msg.data[0] == 0x13:       # End of message ACK
                if j1939.tp_cm_packets_sent == j1939.tp_cm_packets:
                    j1939.tp_cm_connected = False
                    updater.page_sent()
            elif received_msg.data[0] == 0xFF:        # Connection abort
                pass
        # Acknowledge frame check
        elif received_msg.pgn.is_ack and received_msg.source_address == j1939.tp_cm_address:
            pass
            if received_msg.data[0] == 0x00 and received_msg.data[4] == THIS_NODE:
                if received_msg.data[1] == 0:
                    # send_prop_a(1)    FLASH INFO
                    j1939.send_prop_a(update_phase=1,
                                      flash_size=updater.flash_size,
                                      page_count=updater.page_count,
                                      extra_info=0x12345678)
                    updater.flash_confirmed()
                elif received_msg.data[1] == 1:
                    j1939.tp_cm_initiate_connection(data=updater.page_data, pgn=0x00EF00, cts_limit=CTS_LIMIT)
                    j1939.send_cm_rts_pdu()
                    updater.request_to_send()
                elif received_msg.data[1] == 2:
                    j1939.send_prop_a(update_phase=3,
                                      flash_size=updater.flash_size,
                                      page_count=updater.page_count,
                                      extra_info=0x12345678)
                    updater.flash_done()
                elif received_msg.data[1] == 3:
                    pass
            elif received_msg.data[0] == 0x01:
                pass
