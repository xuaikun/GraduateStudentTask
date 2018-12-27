# encoding: utf-8
import numpy as np
N = 5
i = 1
z = 1
count = 1

R = np.empty([N + 1, N + 1], list)
'''
while i <= N:
    R_list = []
    for k in range(i, N + 1):
        # 应用可调度性判定算法判定再加入Nk后Rz的可调度性
        # 如果可调度，则将Nk加入Rz，否则不加入
        # 这里表达肯定有问题，等下看看如何修改，R[z][i]保存的是一个列表才对
        # R[z][i] = [1,2,3,4]
        R_list.append(k)
    R[z][count] = R_list
    count = count + 1
'''

R_list = []

R_list.append(1)
R_list.append(2)
R_list.append(3)
R_list.append(4)


R[z][count] = R_list

print "R[z][count]= ", R[z][count]
R_list.remove(4)

R[z][count] = R_list

print "R[z][count]= ", R[z][count]