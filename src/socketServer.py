#!/usr/bin/env python

import socket
import sys
import serial
import time

print( "Loading warper..!")
try:
    ser = serial.Serial('/dev/ttyACM0', 9600)
    #response = ser.readline()
    #print("read data: " + response)
except Exception, e:
    print "error open serial port: " + str(e)
    exit()
print( "Serial port opened.!")
TCP_IP = '192.168.1.102'
TCP_PORT = 2323
BUFFER_SIZE = 50  # Normally 1024, but we want fast response

print( "Creating Socket..")
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print( "Listening for Connection")
s.bind((TCP_IP, TCP_PORT))
s.listen(1)
print( "Server Started.!")
while True:
    try:
        conn, addr = s.accept()
        print('Connection address:', addr)
        ser.flushInput()
        ser.flushOutput()
        while 1:
            print("waiting for data...")
            data = conn.recv(BUFFER_SIZE)
            print("Data received.!")
            if not data:
                break
            ser.write(data);
            print( "received data:", data)
            conn.send(data)  # echo
            print("Ack Sent.!")
            #response = ser.readline()
            #print("read data: "+ response)
        print("Connection Closed.!")
        ser.close()
        conn.close()
        break
    except:
        print("Unexpected error:",sys.exc_info()[0])
        print("Resetting connection.!")
        break
print("Exit.!")
