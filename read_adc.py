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

fp = serial.Serial(sport, 1152000, timeout=4)

now = datetime.datetime.now()
fp_out = open('adc_'+now.strftime('%Y%m%d_%H%M%S')+'.txt', 'wb')


while True:
    data = fp.read(fp.inWaiting())
    fp_out.write(data)
    fp_out.flush()
