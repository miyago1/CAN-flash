import can

from pgn import PGN


class PDU:
    def __init__(self, can_frame: can.Message = None):
        self.pgn = (can_frame.arbitration_id & 0x01FF0000) >> 8
        self.value = can_frame

    @property
    def value(self):
        return self.value

    @property
    def pgn(self):
        return self._pgn

    @pgn.setter
    def pgn(self, pgn):
        self._pgn = PGN(pgn)

    @property
    def is_pdu_format_1(self):
        return self.pdu_format < 240

    @property
    def is_pdu_format_2(self):
        return self.pdu_format > 240

    @property
    def is_destination_specific(self):
        if self.is_pdu_format_1 and self.pdu_specific != 255:
            return True
        else:
            return False

    @value.setter
    def value(self, value):
        self.priority = (value.arbitration_id & 0x1C000000) >> 26
        self.data_page = (value.arbitration_id & 0x01000000) >> 24
        self.pdu_format = (value.arbitration_id & 0x00FF0000) >> 16
        self.pdu_specific = (value.arbitration_id & 0x0000FF00) >> 8
        self.source_address = (value.arbitration_id & 0x000000FF)
        self.data = value.data

    @staticmethod
    def from_value(value):
        pdu = PDU()
        pdu.value = value
        return pdu

    def __str__(self):
        field_strings = ["P: {0:01X}".format(self.priority)]
        pdu_format_string = "PF: {0:02X}".format(self.pdu_format)
        field_strings.append(pdu_format_string)
        pdu_specific_string = "PS: {0:02X}".format(self.pdu_specific)
        field_strings.append(pdu_specific_string)
        source_address_string = "SA: {0:02X}".format(self.source_address)
        field_strings.append(source_address_string)
        pgn_string = "PGN: {:d}".format(self.pgn.value)
        field_strings.append(pgn_string)

        for index in range(0, min(8, len(self.data))):
            field_strings.append("{0:02x}".format(self.data[index]))

        return " ".join(field_strings)

    ID_TP_CM = 0x1CEC0000  # Prio: 7 DP: 0 PF: 236
    ID_TP_DT = 0x1CEB0000  # Prio: 7 DP: 0 PF: 235
    ID_ACK = 0x18E80000  # Prio: 6 DP: 0 PF: 232
    ID_REQUEST = 0x18EA0000  # Prio: 6 DP: 0 PF: 234
    ID_PROP_A = 0x18EF0000  # Prio: 6 DP: 0 PF: 239
