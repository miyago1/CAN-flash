import zlib
from timeit import default_timer as timer

import progressbar
from intelhex import IntelHex
from statemachine import StateMachine, State
import math


class Flasher(StateMachine):
    MAX_BLOCK_SIZE = 128
    MAX_PACKET_SIZE = 7

    def __init__(self, obj: object):
        super().__init__(obj)

        self.widgets = [progressbar.Bar(marker="#", left="[", right="]"),
                        progressbar.Percentage(), " | ",
                        progressbar.FileTransferSpeed(), " | ",
                        progressbar.SimpleProgress(), " | ",
                        progressbar.ETA()]

        self.bar = None
        self.block_size = 0
        self.packets_sent = 0
        self.pages_sent = 0
        self.packet_count = 0
        self.page_count = 0
        self.flash_size = 0
        self.current_page = 0
        self.data_start = 0
        self.data_end = 0
        self.data = None
        self.page_address = 0
        self.crc = 0
        self.start_time = 0
        self.end_time = 0
        self.duration = 0

    @property
    def hex_file(self):
        return self._hex_file

    @hex_file.setter
    def hex_file(self, file_name):
        if file_name is not None:
            try:
                self._hex_file = IntelHex()
                self.hex_file.loadfile(file_name, format='hex')
            except FileNotFoundError:
                print("Can't find file", file_name)
                exit(-1)  # Something other then an exit perhaps?
            self.flash_size = len(self.hex_file)
            self.page_count = math.ceil(self._flash_size / Flasher.MAX_BLOCK_SIZE)
            self.data = memoryview(self.hex_file.tobinarray(start=0, size=self._flash_size))
            self.data_end = self.flash_size
            self.crc = self.do_crc(self.data)
            print("CRC:", hex(self.crc))

    @property
    def data(self):
        return self._data

    @data.setter
    def data(self, data):
        self._data = data

    @property
    def page_data(self):
        address_msb, address_lsb = (self.page_address & 0xFFFF).to_bytes(2, 'little')
        data_bytes = [address_msb, address_lsb]
        data_bytes.extend((self.data[self.data_start:self.data_end]))
        return data_bytes

    @property
    def flash_size(self):
        return self._flash_size

    @flash_size.setter
    def flash_size(self, flash_size):
        self._flash_size = flash_size

    @property
    def page_count(self):
        return self._page_count

    @page_count.setter
    def page_count(self, data_blocks):
        self._page_count = data_blocks

    @property
    def packet_count(self):
        return self._packet_count

    @packet_count.setter
    def packet_count(self, packets):
        self._packet_count = packets

    def print_info(self):
        print("Flash size:", self.flash_size)
        print("Page count:", self.page_count)
        # print("Data:", self.data.tolist())

    # Statemachine https://pypi.org/project/python-statemachine/
    start = State('Start update', initial=True)
    wait_flash = State('Waiting for Flash answer')
    wait_reset = State('Waiting for MCU reset')
    wait_to_send = State('Waiting for CTS')
    sending_packets = State('Sending packets')
    wait_page_write = State('Waiting for page write')
    verify_data = State('Verify data')
    end = State('Update completed')

    request_flash = start.to(wait_flash)
    flash_confirmed = wait_flash.to(wait_reset)
    request_to_send = wait_reset.to(wait_to_send)
    send_limit_reached = sending_packets.to(wait_to_send)
    clear_to_send = wait_to_send.to(sending_packets)
    page_sent = wait_to_send.to(wait_page_write)
    next_page = wait_page_write.to(wait_to_send)
    update_sent = wait_page_write.to(verify_data)
    flash_done = verify_data.to(end)

    def on_enter_wait_reset(self):
        print("Flash confirmed. Waiting for MCU to reset.")
        if self.flash_size >= self.MAX_BLOCK_SIZE:
            self.block_size = self.MAX_BLOCK_SIZE
        else:
            self.block_size = self.flash_size
        self.current_page = 1
        self.data_start = 0
        self.data_end = self.block_size
        self.page_address = 0x0000

    def on_exit_wait_reset(self):
        print("Transfer initiated.")
        self.bar = progressbar.ProgressBar(widgets=self.widgets, maxval=self.flash_size).start()

    def on_exit_sending_packets(self):
        pass
        # Some checkss

    def on_enter_wait_to_send(self):
        pass
        # or some checks here

    def on_enter_wait_page_write(self):
        self.current_page += 1
        total_bytes = self.current_page * self.MAX_BLOCK_SIZE
        if total_bytes > self.flash_size:
            total_bytes = self.flash_size
        self.bar.update(total_bytes)
        if self.current_page == self.page_count:
            self.block_size = self.flash_size % (self.page_count - 1)
        else:
            self.block_size = self.MAX_BLOCK_SIZE

        if self.flash_size >= self.MAX_BLOCK_SIZE:
            self.block_size = self.MAX_BLOCK_SIZE
        else:
            self.block_size = self.flash_size

        self.data_start = (self.current_page - 1) * self.MAX_BLOCK_SIZE
        self.data_end = self.current_page * self.MAX_BLOCK_SIZE
        self.page_address += self.MAX_BLOCK_SIZE

    def on_enter_start(self):
        pass

    def on_enter_sending_packets(self):
        pass

    def on_enter_wait_flash(self):
        pass

    def on_exit_wait_flash(self):
        self.start_time = timer()

    def on_enter_verify_data(self):
        print("Transfer complete. Waiting for verification")
        pass

    def on_enter_end(self):
        print("CRC verified. Telling MCU to jump to application. Update is finished, exiting...")
        self.end_time = timer()
        duration = self.end_time - self.start_time
        print("Flashing duration:", '{0:.3f}'.format(duration), end=' ')
        print("seconds.")
        exit(0)

    def do_crc(self, s):
        n = zlib.crc32(s)
        return n & 0xffffffff
