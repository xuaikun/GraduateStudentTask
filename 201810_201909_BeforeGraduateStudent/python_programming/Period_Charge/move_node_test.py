# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import TourConstructionProblem as T
N_i = [2,1,3,4,5]

Es_i = np.empty([1, 6], float)
Es_i[0] = [0, 9,1,3,7,2]
N_distance = np.empty([1, 6], float)
N_distance[0] = [0, 9,1,2,0,8]

Es_sort = []
N_distance_sort = []
for i in range(1, len(N_i)):
    # 只获取节点神域能量
    Node_number_Es = []
    Node_number_Es.append(N_i[i])
    Node_number_Es.append(round(Es_i[0][N_i[i]],2))
    Es_sort.append(Node_number_Es)
   
    Node_number_distance= []
    Node_number_distance.append(N_i[i])
    Node_number_distance.append(round(N_distance[0][i], 2))
    N_distance_sort.append(Node_number_distance)

print "Es_i[0] =", Es_i[0]
print "N_distance =", N_distance[0]
print "Es_sort =", Es_sort
print "N_distance_sort =", N_distance_sort
T.select_sort_Numsort(Es_sort)
T.select_sort_Numsort(N_distance_sort)
print "Es_sort =", Es_sort
print "N_distance_sort =", N_distance_sort