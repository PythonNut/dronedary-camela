import sys
import socket
import selectors
import types
import time
import threading

def set_up_server_socket(host, port):
    #Please close sel
    sel = selectors.DefaultSelector()
    lsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    lsock.bind((host, port))
    lsock.listen()
    print("listening on", (host, port))
    lsock.setblocking(False)
    sel.register(lsock, selectors.EVENT_READ, data=None)
    return sel

def accept_wrapper(sock,sel):
    conn, addr = sock.accept()  # Should be ready to read
    print("accepted connection from", addr)
    conn.setblocking(False)
    data = types.SimpleNamespace(addr=addr, inb=b"", outb=b"")
    events = selectors.EVENT_READ | selectors.EVENT_WRITE
    sel.register(conn, events, data=data)


def service_connection(key, mask, sel):
    sock = key.fileobj
    data = key.data
    data.outb = ''
    if mask & selectors.EVENT_READ:
        recv_data = sock.recv(1024)  # Should be ready to read
        if recv_data:
            data.outb = recv_data
        else:
            #print("closing connection to", data.addr)
            print("ERROR")
            print("ERROR")
            print("SOMETHING probably WENT WRONG")
    if mask & selectors.EVENT_WRITE:
        if data.outb:
            print(repr(data.outb))
            return(repr(data.outb))
            #print("echoing", repr(data.outb), "to", data.addr)
            #return repr(data.outb)
            #following could send back to client...
            #sent = sock.send(data.outb)  # Should be ready to write
            #data.outb = data.outb[sent:]

def accept_connection(sel):
    events = sel.select(timeout=0)
    for key, mask in events:
        if key.data is None:
            accept_wrapper(key.fileobj,sel)

def get_data(sel):
    events = sel.select(timeout=0)

    for key, mask in events:
        if key.data is None:
            #should never run
            accept_wrapper(key.fileobj,sel)
        else:
            return service_connection(key, mask,sel)

def close_connection(sock, sel):
    sel.unregister(sock)
    sock.close()


threads = []

t = threading.Thread(target = handle_camera, args=(client,address))
while True:

for i in range(len())
    t = threading.Thread(target = handle_camera, args=(client,address,len(threads)))
    t.start()
    threads.append(t)

def cameras(host, port):
    sel = set_up_server_socket(host, port):
    while True:
        accept_connection(sel)
        get_data(sel)
