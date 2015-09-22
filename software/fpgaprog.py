#!/usr/bin/env python

import serial, sys

port = serial.Serial("/dev/cu.usbserial", 115200)
rbf = open("eclsensor.rbf", 'rb').read()

# Reset FPGA
port.sendBreak()

# Use serial port to send configuration pulses, one byte per bit
pulses = []
for byte in rbf:
	for bit in range(8):
		pulses.append("\xff\xfe"[1 & (ord(byte) >> bit)])

port.write(''.join(pulses))
