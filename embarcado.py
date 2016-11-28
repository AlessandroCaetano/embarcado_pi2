#!/usr/bin/python3

'''
usage ./embarcado
'''
import time
import math
import json
import smbus
import serial
import asyncio
import requests
from   serial.tools import list_ports

'''
    CONSTANTS(ADAPT FOR YOUR HARDWARE)
'''

ARDUINO_VID_PID = '2341:0001'
PCF8591_ADDRESS = 0x70

'''
Find and open a serial connection with Arduino.
COM port is set with default values.
If needed change timeout values, possible timeouts:

None : Wait forever, block call
0    : Non-blocking mode, return immediately
X    : Any number, float included, timeout block call
'''
portName           = list(list_ports.grep(ARDUINO_VID_PID))[0][0]
serialCon          = serial.Serial()
serialCon.port     = portName
serialCon.baudrate = 9600
serialCon.bytesize = serial.EIGHTBITS
serialCon.parity   = serial.PARITY_NONE
serialCon.stopbits = serial.STOPBITS_ONE
serialCon.timeout  = 1
serialCon.xonxoff  = False #disable software flow control
serialCon.rtscts   = False #disable hardware (RTS/CTS) flow control
serialCon.dsrdtr   = False #disable hardware (DSR/DTR) flow control
try:
    serialCon.open()
except Exception as e:
    print('Error opening serial port: ' + str(e))

if serialCon.isOpen():
    try:
        serialCon.flushInput() #Flush input buffer
        serialCon.flushOutput() #Flush output buffer
    except Exception as e:
        print('Error flushing serial port: ' + str(e))

def calcAngle(x,y,z):
    x = float(x)
    y = float(y)
    z = float(z)
    m = math.sqrt(x*x+y*y+z*z)
    angleX=math.acos(x/m)
    angleY=math.acos(y/m)
    angleZ=math.acos(z/m)
    print('X: '+ str(angleX))
    print('Y: ' + str(angleY))
    print('Z: ' + str(angleZ))


#Open i2c connection
bus = smbus.SMBus(0)

'''
    Write to i2c raspberry pin port
'''
def busWrite(value):
    bus.write_block_data(PCF8591_ADDRESS, 0, value)

'''
    Read from i2c raspberry pin port
'''
def busRead():
    reader = bus.read_byte_data(PCF8591_ADDRESS, 1)
    return reader

'''
    Writes data on device on serial port. Called by assyncronous task to read from server
'''
def serialWriter(data):
    global serialCon
    if serialCon.isOpen():
        try:
            serialCon.write(data)
        except Exception as e:
            print('Could not write data on serial con: ' + str(e))

'''
    Assyncronous task to read data from serial port
'''
@asyncio.coroutine
def serialReader():
    global serialCon
    if serialCon.isOpen():
        try:
            bytes = serialCon.inWaiting()
            writeToWebServer(serialCon.read(bytes))
        except Exception as e:
            print('Could not read data from serial con: ' + str(e))
    time.sleep(1)
    asyncio.ensure_future(serialReader())

'''
    Method to close serial connection when needed
'''
def closeSerialConnection():
    global serialCon
    if serialCon.isOpen():
        try:
            serialCon.close()
        except Exception as e:
            print('Error closing serial connection: ' + str(e))

'''
    Assyncronous task to read from web server
'''
@asyncio.coroutine
def readWebServer():
    #data = requests.get('', verify=False).json()
    data = bytes('asdasdas', 'UTF-8')
    serialWriter(data)
    time.sleep(1)
    asyncio.ensure_future(readWebServer())

'''
    Writing to web server via post api, currently only printing on terminal
'''
def writeToWebServer(data):
    #request = requests.post('', data=data)
    #print(request.status_code, request.reason)
    print(data)
    #angles = data.split(' ')
    #if len(angles) > 2:
    #    x = angles[1]
    #    y = angles[3]
    #    z = angles[5].rstrip('\n').rstrip('\r')
    #    calcAngle(x,y,z)

'''
    main funtcion
'''
def main():
    loop = asyncio.get_event_loop()
    try:
        asyncio.ensure_future(serialReader())
        asyncio.ensure_future(readWebServer())
        loop.run_forever()
    except KeyboardInterrupt:
        pass
    finally:
        loop.close()
        closeSerialConnection()

if __name__  == '__main__':
    main()
