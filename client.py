import socket
import time

def set_up_client_socket(host,port):
    #please close the socket later
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((host, port))
    return s

def send_data(socket, data):
    socket.sendall(bytes(data, 'utf-8'))
