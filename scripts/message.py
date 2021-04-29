class Message:
    def __init__(self, raw_msg = None):
        if raw_msg:
            self.decode_message(raw_msg)

    def decode_message(self, raw_msg):
        self.seq = int(raw_msg[0])
        self.time = float(raw_msg[1])
        self.type = int(raw_msg[3], base =16)
        self.size = int(raw_msg[5])

        data = []
        for i in range(self.size):
            d = []
            tmp = int(raw_msg[6+i], base=16)
            for j in range(8):
                d.append(tmp%2)
                tmp =tmp/2
            data.append(d)
        self.data = data

    def get_message(self):
        return {"seq": self.seq,
                "time": self.time,
                "type": self.type,
                "size": self.size,
                "data": self.data}