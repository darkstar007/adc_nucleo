#!/usr/bin/env python

#
# This code is licenced under the GPL version 2, a copy of which is attached
# in the files called 'LICENSE'
#
#
# Copyright Matt Nottingham, 2015
#
#


import os
import os.path as osp
import numpy as np
import sys
import platform

import serial
import struct
import datetime
import time
import sys

if len(sys.argv) < 2:
    sport = '/dev/ttyACM0'
else:
    sport = sys.argv[1]

fp = serial.Serial(sport, 115200, timeout=4)

now = datetime.datetime.now()
fp_out = open('magsensor_'+now.strftime('%Y%m%d_%H%M%S')+'.txt', 'w')

# Dump whats in the buffer both locally and on microcontroller
fp.flushInput()
for x in xrange(40):
    ln = fp.readline()
    print ln

while True:
    ln = fp.readline()
    now = datetime.datetime.now()
    fp_out.write(now.strftime('%Y/%m/%d %H:%M:%S.%f,')+ln)
    fp_out.flush()
