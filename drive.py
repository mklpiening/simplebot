#!/usr/bin/python3

import serial
import time
import sys

port = serial.Serial('/dev/ttyUSB0', 115200)

print('waiting for arduino to reboot')
time.sleep(2)

print('driving forward')

speed_l = int(sys.argv[1])
speed_r = int(sys.argv[2])

port.write(bytes([255, (2 if speed_l < 0 else 0) + (1 if speed_r < 0 else 0), abs(speed_l), abs(speed_r)]))

port.close()
