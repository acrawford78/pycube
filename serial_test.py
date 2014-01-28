#!/usr/bin/python

import serial

s = serial.Serial('/dev/rfcomm0',115200)
s.setTimeout(1.0)
s.setWriteTimeout(0.2)

while s.inWaiting() >= 14:
