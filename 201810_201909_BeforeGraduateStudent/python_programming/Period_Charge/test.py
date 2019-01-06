# encoding: utf-8

import numpy as np
import random
import time
import math
start = time.time()

alpha = random.randint(0, 360)
# alpha = 90
print alpha
end = time.time()
v = 5 # 速度等于5m/s
t = round((end -start), 2)
t = 1
print "t =",t  
coordinate = [1, 1]
coordinate[0] = round((coordinate[0] + v*t*math.cos(alpha/180.0*math.pi)), 2)
coordinate[1] = round((coordinate[1] + v*t*math.sin(alpha/180.0*math.pi)), 2)
print "coordinate =", coordinate
coordinate = [1.0, 1.0]
coordinate[0] = round((coordinate[0] + v*t*np.cos(alpha/180.0*np.pi)), 2)
coordinate[1] = round((coordinate[1] + v*t*np.sin(alpha/180.0*np.pi)), 2)
print "coordinate =", coordinate