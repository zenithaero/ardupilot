#!/usr/bin/env python
import socket

HOST = "127.0.0.1"  # The server's hostname or IP address
PORT = 5503  # The port used by the server

with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
    s.connect((HOST, PORT))
    # s.sendall(b"Hello, world")
    print("connected")
    # while True:
    data = s.recvfrom(4096)
    print("Data", data)

# print("Received", repr(data))
