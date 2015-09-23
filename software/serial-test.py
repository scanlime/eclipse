#!/usr/bin/env python

import serial, sys

port = serial.Serial("/dev/ttyAMA0", 115200)

while True:
    port.write('U' * 1000)

