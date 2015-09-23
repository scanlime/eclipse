#!/usr/bin/env python

import serial, sys

port = serial.Serial("/dev/ttyAMA0", 115200)
rbf = open("../sensor-fpga/eclsensor.rbf", 'rb').read()

# Reset FPGA
port.sendBreak()

# Use serial port to send configuration pulses, one byte per bit
pulses = []
for byte in rbf:
	for bit in range(8):
		pulses.append("\xff\xfe"[1 & (ord(byte) >> bit)])

print "Sending..."
port.write(''.join(pulses))
print "Programmed."

while True:
    sys.stdin.read()

