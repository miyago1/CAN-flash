from pdu import PDU
import can
import math


class J1939Client:
    MASK_EXTENDED_DATA_PAGE = 0x02000000
    MAX_PACKET_SIZE = 7
    CAN_MASK = 0x1BFB00FF
    CAN_FILTER = 0x18E80000

    def __init__(self, can_interface, node_address, state):
        self.can_bus = can_interface
        self.node_address = node_address
        self.node_send_limit = 0xFF
        self.pgn_list = None
        self.state = state

        # Transport protocol session variables
        self.tp_cm_connected = False
        self.tp_cm_address = 0
        self.tp_cm_packets = 0
        self.tp_cm_packets_sent = 0
        self.tp_cm_cts_limit = 0
        self.tp_cm_pgn = 0
        self.tp_cm_message_size = 0
        self.tp_cm_data = None
        self.tp_cm_next_packet = 0

    @property
    def can_bus(self):
        return self._can_bus

    @can_bus.setter
    def can_bus(self, channel):
        self._can_bus = can.interface.Bus(bustype='socketcan', channel=channel)

    @property
    def pgn_list(self):
        return self._pgn_list

    @pgn_list.setter
    def pgn_list(self, pgn_list: list):
        self._pgn_list = pgn_list

    def read_can_bus(self):
        can_msg = self.can_bus.recv()
        if self.is_j1939_frame(can_msg) and can_msg.dlc == 8:
            return PDU(can_msg)
        else:
            return None

    def send_j1939_pdu(self, j1939_frame: PDU):
        can_id = j1939_frame.priority << 26    \
                | j1939_frame.data_page << 24  \
                | j1939_frame.pdu_format << 16 \
                | j1939_frame.pdu_specific << 8\
                | self.node_address
        can_frame = can.Message(arbitration_id=can_id, extended_id=True, data=j1939_frame.data)
        try:
            self.can_bus.send(can_frame)
            return True
        except can.CanError as e:
            return False

    def send_can_frame(self, can_frame: can.Message):
        try:
            self.can_bus.send(can_frame)
            return True
        except can.CanError as e:
            return False

    # J1939 Transport protocol connection management
    # Address of the other party

    @classmethod
    def is_j1939_frame(cls, received_frame: can.Message):
        if received_frame.is_remote_frame or received_frame.is_error_frame:  # J1939 does not use RTR
            return False
        elif received_frame.is_extended_id:
            if received_frame.arbitration_id & cls.MASK_EXTENDED_DATA_PAGE:  # J1939 extended data page always 0
                return False
            else:
                return True
        else:
            return False

    def send_cm_rts_pdu(self):
        # Prepare ID
        can_id = PDU.ID_TP_CM | self.tp_cm_address << 8 | self.node_address
        # Data fields
        pgn_msb, pgn_mid, pgn_lsb = (self.tp_cm_pgn & 0xFFFFFF).to_bytes(3, 'big')
        msb_msg_size, lsb_msg_size = (self.tp_cm_message_size & 0xFFFF).to_bytes(2, 'big')
        data_bytes = [0x10,                     # 1     Control byte
                      lsb_msg_size,             # 2     Message size LSB
                      msb_msg_size,             # 3     Message size MSB
                      self.tp_cm_packets,       # 4     Number of packets to send
                      self.tp_cm_cts_limit,     # 5     Clear to send limit
                      pgn_lsb,                  # 6     PGN LSB
                      pgn_mid,                  # 7     PGN NSB
                      pgn_msb]                  # 8     PGN MSB

        can_msg = can.Message(arbitration_id=can_id, extended_id=True, data=data_bytes)

        return self.send_can_frame(can_frame=can_msg)

    def send_cm_data_pdu(self):
        # Prepare ID
        can_id = PDU.ID_TP_DT | self.tp_cm_address << 8 | self.node_address
        # Data
        data_bytes = [self.tp_cm_next_packet]  # 1
        data_start = (self.tp_cm_next_packet - 1) * self.MAX_PACKET_SIZE
        data_end = (self.tp_cm_next_packet * self.MAX_PACKET_SIZE)
        if data_end > self.tp_cm_message_size:
            data_end = self.tp_cm_message_size
        message_data = self.tp_cm_data[data_start:data_end]

        for i in range(len(message_data), self.MAX_PACKET_SIZE):
            message_data.append(0xFF)

        data_bytes.extend(message_data)         # 2-8

        can_msg = can.Message(arbitration_id=can_id, extended_id=True, data=data_bytes)
        return self.send_can_frame(can_frame=can_msg)

    def send_prop_a(self, update_phase: int, flash_size, page_count, extra_info):
        # Prepare ID
        can_id = PDU.ID_PROP_A | self.tp_cm_address << 8 | self.node_address
        # Data
        data_bytes = [update_phase]
        data_bytes.extend((flash_size & 0xFFFF).to_bytes(2, 'little'))
        data_bytes.append(page_count)
        data_bytes.extend((extra_info & 0xFFFFFFFF).to_bytes(4, 'little'))

        can_msg = can.Message(arbitration_id=can_id, extended_id=True, data=data_bytes)
        return self.send_can_frame(can_frame=can_msg)

    def tp_cm_initiate_connection(self, data, cts_limit: int, pgn: int):
        self.tp_cm_data = data
        self.tp_cm_message_size = len(data)
        self.tp_cm_packets = math.ceil(len(data)/7)
        self.tp_cm_packets_sent = 0
        self.tp_cm_pgn = pgn
        self.tp_cm_cts_limit = cts_limit

    def set_one_to_one_filter(self, destination_address):
        self.tp_cm_address = destination_address
        j1939_filter = self.CAN_FILTER | self.tp_cm_address
        da_sa_filter = {"can_id": j1939_filter, "can_mask": self.CAN_MASK, "extended": True}
        self.can_bus.set_filters([da_sa_filter])

    abort_reason = {
        1: "Already in one or more connection managed sessions and cannot support another.",
        2: "System resources were needed for another task so this connection managed session was terminated.",
        3: "A timeout occurred and this is the connection abort to close the session.",
        4: "CTS messages received when data transfer is in progress.",
        5: "Maximum retransmit request limit reached."
    }

    ack_type = {
        0: "Positive Acknowledgment",
        1: "Negative Acknowledgment",
        2: "Access Denied",
        3: "Cannot Respond"
    }
