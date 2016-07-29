#!/usr/bin/env python

import socket
import sys

TCP_IP = '127.0.0.1'
TCP_PORT = 9999
BUFFER_SIZE = 20  # Normally 1024, but we want fast response

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((TCP_IP, TCP_PORT))
s.listen(1)
print( "Server Started.!")
while True:
    try:
        conn, addr = s.accept()
        print('Connection address:', addr)
        while 1:
            data = conn.recv(BUFFER_SIZE)
            if not data:
                break
            print( "received data:", data)
            conn.send(data)  # echo
        print("Connection Closed.!")
        conn.close()
        break
    except:
        print("Unexpected error:",sys.exc_info()[0])
        print("Resetting connection.!")
print("Exit.!")