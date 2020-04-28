import time
import smbus

i2c_ch = 1
bus = smbus.SMBus(i2c_ch)

address = 0x04

def writeData(data):
    bus.write_i2c_block_data(address, 0, data)
    return -1

def readNumber():
    number = bus.read_byte(address)
    return number

while True:
    pan = input("Pan Coordinate: ")
    tilt = input("Tilt Coordinates: ")
    
    if not pan:
        continue
    if not tilt:
        continue
    
    panTiltData = [pan, tilt]
    writeData(panTiltData)
    
    print("RPI: Hi Arduino, I sent you pan: ", pan, "and tilt: ", tilt)
    time.sleep(0.5) # sleep 1 second
    
##    number = readNumber()
##    print("Arduino: Hey RPI, I received a digit ", number)
    