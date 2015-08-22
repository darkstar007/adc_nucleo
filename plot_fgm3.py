#!/usr/bin/python

from matplotlib import pyplot as plt

import sys

fp = open(sys.argv[1], 'r')

data = []

for ln in fp:
    splt = ln.split(',')
    data.append(splt[1])

plt.plot(data)
plt.show()

