import socket

UDP_IP = "192.168.0.1"
UDP_PORT = 50000
#MESSAGE = b"\x66\x80\x80\x30\x80\x80\x80\x80"
MESSAGE = bytearray([102, 128, 128, 255, 128, 128, 128, 128, 12, 243, 153])

print("UDP target IP:", UDP_IP)
print("UDP target port:", UDP_PORT)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))

