#!/usr/bin/python3

'''
usage ./embarcado
'''
import time
import math
import json
import smbus
import serial
from concurrent.futures import ProcessPoolExecutor
import requests
from   serial.tools import list_ports

'''
CONSTANTS(ADAPT FOR YOUR HARDWARE)
'''
ARDUINO_VID_PID = '2341:0001'
PCF8591_ADDRESS = 0x70

'''
Global variables for engine and angle normalization
'''
engine_vel = {}
angle = {}
calibration = True
d1 = d2 = d3 = velocity = angle = 0.0

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

#Open i2c connection
bus = smbus.SMBus(1)

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


'''
Write to i2c raspberry pin port
'''
def busWrite():
    global engine_vel, velocity
    value = engine_vel.get(velocity)
    bus.write_byte_data(PCF8591_ADDRESS, 0x44, value)

'''
Writes data on device on serial port. Called by assyncronous task to read from server
'''

#def serialWriter():
#    global serialCon
#    while True:
#        if serialCon.isOpen():
#            try:
#                serialCon.write(data)
#            except Exception as e:
#                print('Could not write data on serial con: ' + str(e))
#

'''
Assyncronous task to read data from serial port
'''
def serialReader():
    global serialCon, d1, d2, d3, angle
    while True:
        if serialCon.isOpen():
            try:
                inc_bytes = serialCon.inWaiting()
            except Exception as e:
                print('Could not read data from serial con: ' + str(e))
        time.sleep(1)

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
def readWebServer():
    global velocity
    while True:
        data = requests.get('', verify=False).json()
        time.sleep(1)


'''
Writing to web server via post api, currently only printing on terminal
'''
def writeToWebServer(values):
    global velocity, angle, d1, d2, d3
    data = {"test":{"velocity": values[0], "angle": values[1]}}
    json_data = json.dumps(data)
    headers = {'Content-type': 'application/json'}
    request = requests.put('http://192.168.15.11:3000/tests/1', data=json_data, headers=headers)
    print(request.status_code, request.reason)
    print(data)

#def calibration():
#    global calibration
#    global engine_vel
#    if callibrarion == False:
#            break
#    elif calibration == True:
#        for i in range(0,255):


'''
main funtcion
'''
def main():
    pool = ProcessPoolExecutor(4);
    try:
        serial_reader_future = pool.submit(serialReader)
        bus_writer_future = pool.submit(busWrite)
        web_writer_future = pool.submit(writeToWebServer)
        web_reader_future = pool.submit(readWebServer)
    except KeyboardInterrupt:
        pass
    finally:
        serial_reader_future.done()
        bus_writer_future.done()
        web_writer_future.done()
        web_reader_future.done()

    #loop = asyncio.get_event_loop()
    #try:
    #    asyncio.ensure_future(serialReader())
    #    asyncio.ensure_future(readWebServer())
    #    loop.run_forever()
    #except KeyboardInterrupt:
    #    pass
    #finally:
    #    loop.close()
    #    closeSerialConnection()

if __name__  == '__main__':
    main()
