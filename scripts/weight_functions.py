#!/usr/bin/python3

from math import exp

import numpy as np
import matplotlib.pyplot as plt

def sigmoid_func(x):
    #return 1.0 / (1.0 + np.exp(-15.0*(x-0.5)))
    return 1.0-np.exp(-5.0*x)

x = np.arange(0.0, 1.0, 0.01)

v13w = 1.0 - sigmoid_func(x)
v3w = v13w * (x*(1.0-x))
v1w = v13w - v3w
v2w = sigmoid_func(x)

fig, ax = plt.subplots()
ax.plot(x, v1w, label='v1')
ax.plot(x, v2w, label='v2')
ax.plot(x, v3w, label='v3')
ax.grid()
ax.legend()
plt.show()
