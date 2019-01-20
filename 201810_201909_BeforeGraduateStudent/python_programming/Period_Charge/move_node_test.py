# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import TourConstructionProblem as T
'''
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


'''
N_i = [2,1,3,4,5]

Es_i = np.empty([1, 6], float)
Es_i[0] = [0, 9,1,3,7,2]
N_distance = np.empty([1, 6], float)
N_distance[0] = [0, 9,1,2,0,8]

y = []
y.append(Es_i[0])
y.append(N_distance[0])

# f1 = open('test.txt','w')
# f1.write(str(y))
# f1.close()
result =[['MCV0', 'Es_sum =', 808.52, 'Em =', 150000.0, 'Time_sum =', 20.21, 'T =', 7200.0, 'Throughput =', '10'], 
         ['MCV1', 'Es_sum =', 2714.39, 'Em =', 150000.0, 'Time_sum =', 67.86, 'T =', 7200.0, 'Throughput =', '7'], 
         ['MCV2', 'Es_sum =', 1442.23, 'Em =', 150000.0, 'Time_sum =', 36.06, 'T =', 7200.0, 'Throughput =', '3']]
result1 =[[808.52, 150000.0, 20.21,  7200.0, 10],[808.52, 150000.0, 20.21,  7200.0, 10]]
np.savetxt("filename.txt",result1)
# 读取文件的操作
temp = np.loadtxt('filename.txt')
# 将.txt文档中矩阵的边长读取出来，作为我们的n
print ('temp.shape[0] = ', temp.shape[0])
print ('temp.shape[1] = ', temp.shape[1])
n = temp.shape[1]
print ("the length is ", n)
print "temp =", temp
#f1 = open('test.txt','r')
#x = f1.read()
#print "f1 =", x
#print "len(x) =", len(x) 
#f1.close()
pathname = 'MCV' + str(MC_Num) +'.txt'
print "pathname =", pathname