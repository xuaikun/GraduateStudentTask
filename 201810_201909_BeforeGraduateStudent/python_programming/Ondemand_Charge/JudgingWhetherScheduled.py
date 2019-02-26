# encoding: utf-8

# 算法功能：判断当前充电回路是否可调度
# 算法输入：电单车数目N，电单车功率列表，移动充电车总能量，充电功率，移动功率
# 充电效率，哈密顿回路，移动速度，节点之间的邻接矩阵，总周期
# 算法输出：调度标志
# 算法功能：判定一个充电回路P是否可调度
def judging_whether_scheduled(P, Em, qc, qm, nl, R, vm, N_distance):

    # 统计R中节点个数
    R_len = len(R)
    print "R", R
    # 求当前回路的总距离
    D = 0
    # 一定要想清楚这里，只用R_len - 1
    for i in range(0, R_len - 1):
        D = D + N_distance[R[i]][R[i+1]]
    # 还要加上一条返回S的路
    # 保留两位小数
    D = round(D + N_distance[R[R_len - 1]][R[0]], 2)

    print "当前子回路的总距离D = ", D
    psum = 0
    pmax = 0.0
    # 只计算当前加入子回路的电单车的功率
    for i in range(1, R_len):
        # S点的psum不计算
        psum = psum + P[0][R[i]]
        # 获取当前回路中的功率最大点
        if P[0][R[i]] > pmax:
            pmax = P[0][R[i]] 
    # 保留两位小数
    psum = round(psum, 2)
    print "总能耗psum =", psum
    # 三个决策条件
    factor_1 = psum
    factor_2 = round((D*qc*nl*pmax)/(qc*nl-psum), 2)
    factor_3 = round(((psum*((D*qc)/(vm*(qc*nl-psum)))) + (D*qm)), 2)
    #  可调度的标志
    # success_flag = True 可调度
    # success_flag = False 不可调度
    print "qc*nl =", qc*nl
    print "factor_1 =", factor_1
    print "vm =", vm
    print "factor_2 =", factor_2
    print "Em =", Em
    print "factor_3 =", factor_3
    if qc*nl > factor_1 and vm > factor_2 and Em >= factor_3:
        success_flag = True
    else:
        success_flag = False
    return success_flag




