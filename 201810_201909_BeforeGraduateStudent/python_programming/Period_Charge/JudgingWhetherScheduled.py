# encoding: utf-8
import numpy as np
# 算法功能：判定一个充电回路P是否可调度
'''
    Es = 10  # 每个传感器节点总能量为Es = 10kj
    n = 4  # n 表示P中包好的传感器个数P=<N0,N1,……Nk,Nk+1>其中N0 = Nk+1 = S 不用计算
    # p[0][i] = 0.01  # 每个传感器功耗相同为p[0][i] = 0.01W
    p = np.empty([1, n + 1], float)
    for i in range(0, n):
        # N0 = Nk+1 = S 它的功率不用参与计算，直接赋值为0
        if i == 0 or i == n - 1:
            p[0][i] = 0
        else:
            p[0][i] = 0.01
        # print "p[0][", i, "] = ", p[0][i]
    Em = 65  # MC的总能量为x kj
    qm = 8  # Mc移动功耗为qm = 8 J/m
    qc = 4.45  # qc*n 为能量传输率，qc= 4.45 W
    nl = 0.5  # 类似于效率一样，占比多少 n = 0.5
    T = 10  # 充电周期需要知道10s
    vm = 0.3  # MC的移动速度0.3m/s
    N = n
    P = p
    R = [1, 2, 3, 4]
    N_distance = np.empty([N + 1, N + 1], int)
    for i in range(1, N + 1):
        for j in range(i, N + 1):
            if j == i:
                N_distance[i][j] = 0
            else:
                N_distance[i][j] = 2
                N_distance[j][i] = N_distance[i][j]
    '''


def judging_whether_scheduled(N, P, Em, qc, qm, nl, R, vm, N_distance, T):

    # 统计R中节点个数
    R_len = len(R)
    # 求当前回路的总距离
    D = 0
    # 一定要想清楚这里，只用R_len - 1
    for i in range(0, R_len - 1):
        D = D + N_distance[R[i]][R[i+1]]
    # 还要加上一条返回S的路
    D = D + N_distance[R[R_len - 1]][R[0]]

    print "总距离D = ", D
    psum = 0
    for i in range(1, N + 1):
        psum = psum + P[0][i]
    print "总能耗psum =", psum

    factor_1 = (D*qc*nl)/(vm*(qc*nl-psum))
    factor_2 = (psum*((D*qc)/(vm*(qc*nl-psum)))) + (D*qm)
    #  可调度的标志
    # success_flag = True 可调度
    # success_flag = False 不可调度
    print "T =", T
    print "factor_1 =", factor_1
    print "Em =", Em
    print "factor_2 =", factor_2
    if T >= factor_1 and Em >= factor_2:
        success_flag = True
    else:
        success_flag = False
    return success_flag




