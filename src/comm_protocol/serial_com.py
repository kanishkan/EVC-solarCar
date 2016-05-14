import serial
import time

try:
    ser = serial.Serial('/dev/ttyACM0', 115200)
    response = ser.readline()
    print("read data: " + response)

except Exception, e:
    print "error open serial port: " + str(e)
    exit()

if ser.isOpen():

    try:
        ser.flushInput() #flush input buffer, discarding all its contents
        ser.flushOutput()#flush output buffer, aborting current output 
                 #and discard all that is in buffer

        #write data
        ser.write(unicode("3\n"))
        print("write data: 3")

        time.sleep(1)  #give the serial port sometime to receive the data

        numOfLines = 0

        response = ser.readline()
        print("read data: "+ response)

        ser.close()
    except Exception, e1:
        print "error communicating...: " + str(e1)

else:
    print "cannot open serial port "

print "Program End.!"
