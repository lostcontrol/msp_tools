#!/usr/bin/python

from __future__ import print_function

import math
import random
import time
import sys
import serial
import struct
import matplotlib.pyplot as plt
import argparse
from pprint import pprint

class AccelData(object):

    def __init__(self):
        self.__x = []
        self.__y = []
        self.__z = []

    def insert(self, x, y, z):
        self.__x.append(x)
        self.__y.append(y)
        self.__z.append(z)

    def reset(self):
        self.__x = []
        self.__y = []
        self.__z = []

    @staticmethod
    def rms(data):
        return math.sqrt(sum([m * m for m in data]) / len(data))

    @staticmethod
    def mean(data):
        return sum(data) / len(data)

    def getX(self):
        return self.__x

    def getY(self):
        return self.__y

    def getZ(self):
        return self.__z


class FakeMsp(object):

    def setMotor(self, index, pwm):
        print("setMotor(%d)=%d" % (index, pwm))

    def stopMotors(self):
        print("setMotor(all)=stop")

    def readAccel(self):
        time.sleep(1./300)
        return [512*(2*m-1) for m in (random.random(), random.random(), random.random())]

class Msp(object):

    RAW_IMU = 102
    SET_MOTOR = 214

    def __init__(self, port, baudrate):
        self.__ser = serial.Serial(port, baudrate, timeout=1)

    def sendCmd(self, data_length, code, data):
        checksum = 0
        total_data = ['$', 'M', '<', data_length, code] + data
        for i in struct.pack('<2B%dh' % len(data), *total_data[3:len(total_data)]):
            checksum ^= ord(i)
        total_data.append(checksum)
        try:
            #pprint(struct.pack('<3c2B%dhB' % len(data), *total_data))
            self.__ser.write(struct.pack('<3c2B%dhB' % len(data), *total_data))
        except Exception, error:
            print("Error in sendCmd(" + str(error) + ")")

    def receive(self):
        # header
        if self.__ser.read() != "$":
            raise Exception("Expected $")
        if self.__ser.read() != "M":
            raise Exception("Expected M")
        if self.__ser.read() != ">":
            raise Exception("Expected >")
        datalength = struct.unpack('<B', self.__ser.read())[0]
        code = struct.unpack('<B', self.__ser.read())[0]
        data = [] if datalength < 1 else self.__ser.read(datalength)
        checksum = struct.unpack('<B', self.__ser.read())[0]
        # Checksum check
        c = datalength
        c ^= code
        for d in data: c ^= ord(d)
        if checksum != c:
            raise Exception("Bad checksum")
        return (datalength, code, data)

    def stopMotors(self):
        motors = [1000] * 4 + [0] * 4
        self.sendCmd(len(motors), Msp.SET_MOTOR, motors)
        self.sendCmd(0, Msp.SET_MOTOR, [])
        self.receive() # Acknowledge

    def setMotor(self, index, pwm):
        motors = [1000] * 4 + [0] * 4
        motors[index] = pwm
        self.sendCmd(len(motors), Msp.SET_MOTOR, motors)
        self.sendCmd(0, Msp.SET_MOTOR, [])
        self.receive() # Acknowledge

    def readAccel(self):
        self.sendCmd(0, Msp.RAW_IMU, [])
        (datalength, code, data) = self.receive()
        if datalength > 0:
            temp = struct.unpack('<' + 'h' * (datalength / 2), data)
            return [float(m) for m in temp[:3]]
        else:
            raise Exception("No data response")

class Vibration(object):

    def __init__(self, msp, accel, verbose):
        self.__msp = msp
        self.__accel = accel
        self.__verbose = verbose

    def __ramp(self, motor, start, stop, delay = 0.1):
        for i in range(start, stop, int(math.copysign(200, stop - start))):
            self.__msp.setMotor(motor, i)
            time.sleep(delay)
        self.__msp.setMotor(motor, stop)
    
    def run(self, motor, pwm, duration):
        print("Calibrating...")
        for i in range(50):
            self.__accel.insert(*[m / 512. for m in self.__msp.readAccel()]) # Convert to 1G        
        mean = [AccelData.mean(self.__accel.getX()), AccelData.mean(self.__accel.getY()), AccelData.mean(self.__accel.getZ())]

        self.__accel.reset()

        bold = "\033[1m{0}\033[0m"
        if self.__verbose > 0:
            print("mean(x,y,z)=%.2f %.2f %.2f" % (mean[0], mean[1], mean[2]))
        
        print("Starting motor...")
        #self.__msp.setMotor(motor, pwm)
        self.__ramp(motor, 1000, pwm)
        time.sleep(0.1)
        
        print("Measuring vibrations...")
        start = time.time()
        counter = 0
        while (time.time() - start < duration):
            accel = [m / 512. for m in self.__msp.readAccel()] # Convert to 1G
            for i in range(3):
                accel[i] -= mean[i]
            self.__accel.insert(*accel)
            counter += 1
        end = time.time()
        
        if self.__verbose > 1:
            print("samples=%d freq=%d Hz" % (counter, round(counter / (end - start))))
        
        print("Stopping motor...")
        self.__msp.setMotor(motor, 1000)
        
        if self.__verbose > 0:
            print("rms(x,y,z)=%.2f %.2f %.2f" % (AccelData.rms(self.__accel.getX()),
                AccelData.rms(self.__accel.getY()),
                AccelData.rms(self.__accel.getZ())))
        data = self.__accel.getX() + self.__accel.getY() + self.__accel.getZ()
        print("rms(total)=%s" % bold.format("%.2f" % AccelData.rms(data)))

    def plot(self):
        samples = range(0, len(self.__accel.getX()))
        plt.plot(samples, self.__accel.getX())
        plt.plot(samples, self.__accel.getY())
        plt.plot(samples, self.__accel.getZ())
        plt.grid(True)
        plt.title("Acceleration samples")
        plt.xlabel("sample")
        plt.ylabel("accel")
        plt.show()

    def print(self):
        self.__accel.printRms()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("motor", type=int, choices=xrange(4), help="motor to be tested (starting from 0)")
    parser.add_argument("pwm", type=int, help="PWM value used during the test")
    parser.add_argument("duration", type=int, default=1, nargs="?", help="duration of the test (default: %(default)s)")
    parser.add_argument("--plot", action='store_true', help="plot sensors data at the end of the test")
    parser.add_argument('--verbose', '-v', action='count')
    parser.add_argument('--port', '-p', type=str, default="/dev/ttyUSB0", help="serial port (default: %(default)s)")
    parser.add_argument('--baudrate', '-b', type=int, default=115200, help="baudrate (default: %(default)s)")
    parser.add_argument('--props-are-removed', action='store_true', required=True, help="remove your props and set this flag")

    args = parser.parse_args()

    #msp = FakeMsp()
    msp = Msp(args.port, args.baudrate)
    accel = AccelData()
    vibration = Vibration(msp, accel, args.verbose)
    try:
        vibration.run(args.motor, args.pwm, args.duration)
        if args.plot:
            vibration.plot()
    except Exception, e:
        msp.stopMotors()
        raise e

if __name__ == "__main__":
    main()

