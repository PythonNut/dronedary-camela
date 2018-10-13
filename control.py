import socket
import time
import operator
from functools import reduce

class DroneController(object):
    UDP_IP = "192.168.0.1"
    UDP_PORT = 50000
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("", self.UDP_PORT))
        self.fly(0, 0, 0)
    
    def __del__(self):
        self.sock.close()

    def float_to_byte_unsigned(self, f):
        return round(f * 255)

    def float_to_byte_signed(self, f):
        return round(f * 127) + 128

    def fly(self, t, x, y):
        t_b = self.float_to_byte_unsigned(t)
        x_b = self.float_to_byte_signed(x)
        y_b = self.float_to_byte_signed(x)
        
        params = [x_b, y_b, t_b, 0x80, 0x80, 0x80, 0x80, 0x0e]
        checksum = reduce(operator.xor, params)
        command = [102, *params, checksum, 153]
        message = bytearray(command)
        self.sock.sendto(message, (self.UDP_IP, self.UDP_PORT))

