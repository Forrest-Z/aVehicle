from pyModbusTCP.client import ModbusClient


class ModbusClientRS:
    def __init__(self):
        self.client = ModbusClient()

    def writeRegister(self, address, value):
        if self.client.is_open():
            return self.client.write_single_register(address, value)

        return None

    def readRegister(self, address, value):
        if self.client.is_open():
            return self.client.read_holding_registers(address, value)

        return None

    def connect(self, host, port):
        # self.client.debug(True)
        self.client.host(host)
        self.client.port(port)

        if not self.client.is_open():
            if not self.client.open():
                print("unable to connect to " + str(host) + ":" + str(port))

    def is_open(self):
        return self.client.is_open()

    def disconnect(self):
        return self.client.close()
