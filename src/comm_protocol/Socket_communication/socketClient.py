import socket
import sys

TCP_IP = "192.168.1.101"
TCP_PORT = 2326
BUFFER_SIZE = 50
MESSAGE = "HOST:777"        # Connection request

print("Creating Socket")
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print("Connecting to server..")
s.connect((TCP_IP, TCP_PORT))
print("Connected.!")
#bytes = str.encode(MESSAGE)
#s.send(bytes)
#data = s.recv(BUFFER_SIZE)
#print("Response: ",data)

print("Use a(left)/d(right)/w(front)/s(back) to control the car.")
print("Ready...!")

while True:
    inDat = input('Input? ')
    if inDat=="a":
        s.send(str.encode("030R-"))
        data = s.recv(BUFFER_SIZE)
        print("Response: ",data)
    elif inDat == "s":
        s.send(str.encode("003T-"))
        data = s.recv(BUFFER_SIZE)
        print("Response: ",data)
    elif inDat == "w":
        s.send(str.encode("010T+"))
        data = s.recv(BUFFER_SIZE)
        print("Response: ",data)
    elif inDat == "d":
        s.send(str.encode("030R+"))
        data = s.recv(BUFFER_SIZE)
        print("Response:",data)
    else:
        print("Invalid input.!")
    #except:
    #    print("Unexpected error:",sys.exc_info()[0])
    #    break
s.close()
print("Exit.!")