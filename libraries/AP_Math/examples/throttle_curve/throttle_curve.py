# -*- coding: utf-8 -*-
"""
Created on Sat Apr  4 18:04:14 2020

@author: markw
"""
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import csv

# read CSV file consisting of x axis followed by 2N rows of curve data
data = []
with open('throttle_curve_data.csv', 'rb') as csvfile:
    datareader = csv.reader(csvfile, quoting=csv.QUOTE_NONNUMERIC)
    for row in datareader:
        data.append(np.asarray(row[:-1]))

# first row specs alpha
alpha = data[0]

# second row is out_mid
out_mid = data[1]

# third row is x axis for plot
x = data[2]

fig, ax = plt.subplots()
#lines = ax.plot(x, plane, 'g-') #, x, copter, 'b-')
for j in range(len(out_mid)):
    ax.plot(x, data[3+2*j], 'g-', x, data[4+2*j], 'b-')

ticks = [0]
for j in range(len(out_mid)):
    ticks.append(out_mid[j])
    label = 'mid %.2f'%out_mid[j]
    plt.annotate(label, [-0.04, .02+out_mid[j]])
ticks.append(1)

plt.yticks(ticks)
plt.xticks([0,.5,1])

ax.plot([.5,.5], [0,1], 'k-')
for j in range(len(out_mid)):
    ax.plot([0,1],[out_mid[j],out_mid[j]], 'k--')
plt.xlabel('stick position')
plt.ylabel('throttle demand')

plt.title('throttle curves, alpha: %5.3f\nplane:green, copter:blue' % (alpha))
plt.savefig('throttle_curves.png')
plt.show()
