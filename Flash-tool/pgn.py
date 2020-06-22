class PGN:

    def __init__(self, pgn = 0):
        self.value: int = pgn

    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, value):
        self._value = value

    @property
    def is_ack(self):
        return self.value == PGN.ACK

    @property
    def is_connection_management(self):
        return self.value == PGN.TP_CM

    @property
    def is_data_packet(self):
        return self.value == PGN.TP_DT

    @property
    def is_request(self):
        return self.value == PGN.REQUEST

    @property
    def is_proprietary_a(self):
        return self.value == PGN.PROP_A

    @property
    def __repr__(self):
        self._value

    ACK = 59392
    TP_CM = 60416
    TP_DT = 60160
    REQUEST = 59904
    PROP_A = 61184
