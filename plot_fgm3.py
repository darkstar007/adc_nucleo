#!/usr/bin/python

from matplotlib import pyplot as plt
from matplotlib.patches import Rectangle
import datetime
import sys
import numpy as np

fp = open(sys.argv[1], 'r')

data = []
day = []
day0 = None
xval = []

for ln in fp:
    splt = ln.split(',')
    data.append(int(splt[-1].strip()))
    if splt[0].find('/') != -1:
	date = datetime.datetime.strptime(splt[0], '%Y/%m/%d %H:%M:%S.%f')
	if day0 == None:
	    day0 = datetime.datetime(date.year, date.month, date.day, 0, 0, 0)
	    
	dt = date - day0

	xval.append(dt.total_seconds() / 3600.0)
	if len(xval) == 1:
	    print dt, xval
	    
	day.append(int(date.strftime('%j')))
    else:
	day.append(0)

print data[0]
data = ((1.0 / (15e3 + np.array(data)) - 12e-6) / ((15.5 - 8.0) * 1.0e-6)) * 1.0 * 100 * 1e3
print data[0]
mn = np.mean(data)

data -= mn

plt.plot(xval, data, label = 'Mean '+format(mn/1e3, '.3f')+'$\mu$T')
num_days = day[-1] - day[0] + 1
min_y = np.min(data)
max_y = np.max(data)
spread_y = max_y - min_y
dxval = (xval[-1] - xval[0]) / float(len(xval))
for dy in xrange(num_days):
    start = day.index(day[0] + dy)
    num = day.count(day[0] + dy)

    if dy % 2 == 0:
	col = 'r'
    else:
	col = 'b'
    #print day[start], day[start+num-1], day[44221], len(day), start, num
    rect = Rectangle((xval[start], min_y), xval[start + num - 1] - xval[start] + dxval, spread_y, alpha=0.5, color = col)
    plt.gca().add_patch(rect)

if day0 == None:
    plt.xlabel('Time (s)')
else:
    plt.xlabel('Time (hours) from '+day0.strftime('%Y/%m/%d 00:00:00'))

plt.ylabel('nT')
plt.legend()
plt.show()

