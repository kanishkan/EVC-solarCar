import smbus
import time

bus = smbus.SMBus(1)

# Arduino Address
address = 0x04

def writeNumber(value):
	bus.write_byte(address, value)
	return -1

def readNumber():
	number = bus.read_byte(address)
	return number

while True:
	var = input(“Enter 1 – 9: “)
	if not var:
		break;
	writeNumber(var)
	print "PI: Data = ", var
	time.sleep(1)
	
	number = readNumber()
	print "Arduino: Data = ", number

print "Program Exit..!"