#!/usr/bin/python3

'''
usage ./embarcado
'''
import time
import math
import json
#import smbus
import serial
import numpy as np
from scipy.linalg import solve
from threading import Thread
import random
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
drag = lift = d1 = d2 = d3 = velocity = engine_vel = angle = 0.0

serialCon = bus = None

'''
Set to true to run simmulation
'''
test = True

'''
Find and open a serial connection with Arduino.
COM port is set with default values.
If needed change timeout values, possible timeouts:

None : Wait forever, block call
0    : Non-blocking mode, return immediately
X    : Any number, float included, timeout block call
'''
def openSerialCon():
    global serialCon
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
def openI2C():
    global bus
    bus = smbus.SMBus(1)

def calcAngle(x,y,z):
    global angle
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
    angle = angleY

'''
Write to i2c raspberry pin port
'''
def busWrite():
    global bus, velocity 
    value = velocity
    bus.write_byte_data(PCF8591_ADDRESS, 0x44, value)

'''
Calculates lift and drag for parameters
'''
def calculateLiftDrag(d1,d2,d3):
    global lift, drag, angle
    a = np.array([[math.sin(angle), math.cos(angle)],[(59.5*math.sin(angle) + 357.5*math.cos(angle)), 357.5*math.sin(angle)]])
    b =  np.array([d2, (102.91*d1 + 45.3*d3)])
    lift, drag = solve(a, b)
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
    global serialCon, d1, d2, d3, angle, engine_vel
    while True:
        if serialCon.isOpen():
            try:
                inc_bytes = serialCon.inWaiting()
                data = inc_bytes.split()
                angle = calcAngle(data[1],data[3],data[5])
                d1, d2, d3 = data[7], data[9], data[11]
                lift, drag = calculate_lift_drag(d1,d2,d3)
                engine_vel = data[13]
            except Exception as e:
                print('Could not read data from serial con: ' + str(e))
        time.sleep(0.5)

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
        data = requests.get('https://ancient-beach-55720.herokuapp.com/tests/1', verify=False).json()
        time.sleep(0.5)
        velocity = data['intended_velocity']
        print('Read velocity from APP is: ' + str(velocity))


'''
Writing to web server via post api, currently only printing on terminal
'''
def writeToWebServer():
    global engine_vel, angle, lift, drag
    while True:
        data = {"test":{"velocity": engine_vel, "angle": angle, "drag": drag, "lift": lift}}
        json_data = json.dumps(data)
        headers = {'Content-type': 'application/json'}
        request = requests.put('https://ancient-beach-55720.herokuapp.com/tests/1', data=json_data, headers=headers)
        print(request.status_code, request.reason)
        print(data)

'''
Run simulation of parameters
'''
def runSimulation():
    global drag, lift, d1, d2, d3, velocity, engine_vel, angle
    t1 = Thread(target = readWebServer)
    t2 = Thread(target = writeToWebServer)

    t1.setDaemon(True)
    t2.setDaemon(True)
    
    t1.start()
    t2.start()
    while True:
        with open('sample_data', 'r+') as infile: 
            for line in infile:
                line = line.split()
                drag = line[1] 
                lift = line[3]
                d1 = line[5]
                d2 = line[7]
                d3 = line[9]
                velocity = line[11]
                engine_vel = line[13] 
                angle = line[15]
              

'''
main funtcion
'''
def main():
    if test is False:
        try:
            openSerialCon()
            openI2C()
            t1 = Thread(target = serialReader)
            t2 = Thread(target = readWebServer)
            t3 = Thread(target = writeToWebServer)
            t4 = Thread(target = busWrite)
            
            t1.setDaemon(True)
            t2.setDaemon(True)
            t3.setDaemon(True)
            t4.setDaemon(True)
        
            t1.start()
            t2.start()
            t3.start()
            t4.start()
        
        except KeyboardInterrupt:
            pass
        finally:
            closeSerialConnection()
    else:
        runSimulation()

if __name__  == '__main__':
    main()