# encoding: utf-8
import numpy as np
# 算法功能：判定一个充电回路P是否可调度
# 输入变量：P=<N0,N1,……,Nk,Nk+1>, Es,v,qm,qc,n;
# 输出变量：P的可调度性(Tl,Tu,w)
# 初始化变量，收集并计算判定VPGS条件所需参数值
# 1.计算P的总距离D
# 2.收集所有传感器功耗{pi|1<=i<=k},计算psum和pmx
# 判定VPGS条件是否成立
# 3.if VPGS条件不成立 then
# 4.   程序终止，返回失败状态
# 5.endif
# 6.令Tl = (D*qc*n)/(v*(qc*n - psum))
# 7.令Tu = (Es*qc*n)/(pmx)
# 8.令w(T) = (psum/(qc*n))*T + D/v
# 9.返回成功状态

# 这里都是假设的参数：
r = 1       # 圆半径为 r = 1km, 第五章假设为12个传感器节点，均匀分布在圆心为服务站节点，半径为1km的圆周上
Es = 10     # 每个传感器节点总能量为Es = 10kj
n = 4       # n 表示P中包好的传感器个数P=<N0,N1,……Nk,Nk+1>其中N0 = Nk+1 = S 不用计算
# p[0][i] = 0.01  # 每个传感器功耗相同为p[0][i] = 0.01W
p = np.empty([1, n], float)
for i in range(0, n):
    # N0 = Nk+1 = S 它的功率不用参与计算，直接赋值为0
    if i == 0 or i == n-1:
        p[0][i] = 0
    else:
        p[0][i] = 0.01
    print "p[0][", i, "] = ", p[0][i]
P = []
P.append(p)
print P
Em = 60     # MC的总能量为x kj
qm = 8      # Mc移动功耗为qm = 8 J/m
qc = 4.45      # qc*n 为能量传输率，qc= 4.45 W
nl = 0.5       # 类似于效率一样，占比多少 n = 0.5
T = 10      # 充电周期需要知道10s
v = 0.3      # MC的移动速度0.3m/s


def judging_whether_scheduled(Es, v, qm, qc, nl):
    # P为一个回路其中的Nk为某个传感器节点N0 = Nk+1 = S充电桩(能源站)
    # Pk = (Tl,Tu,wk)
    # Nk(pk,lk,Es) 其中pk表示功率，Es表示能量
    # 移动充电MC(qc,n,qm,v,Em)
    # psum = p1+p2+p3+pk(p0 = pk+1 为S的功率不需要算进来)
    # pmx = max1<=i<=k pi*(qc*n-pi)
    # VPGS条件:(1)qc*n > psum ,(2)v >= (D*pmx)/(Es*(qc*n - psum)) ,(3)Em >= D*qm + (psum*Es*qc)/pmx

    # P_Schedule 保存经过VPGS处理后的结果
    P_Schedule = []
    # 只包含两个传感器节点的时候
    # 对于这个距离，我感觉有蛮多方法计算，比如：在哈密顿图回路，每个节点都用自己的坐标，通过坐标之间的计算就可以得出距离
    # 相邻的两个节点之间的计算，再把所有相邻的距离相加，就是D
    # D = D + sqrt((x1-x2)^2 + (y1-y2)^2)
    D = 2*r +4 * np.sin((15/180)*np.pi)*r
    # 统计psum
    print "D = ", D
    psum = 0.0
    for i in range(1, n - 1):
        psum = psum + p[0][i]
    print "psum = ", psum
    # 获取最大的pmx
    pmx = p[0][1] * (qc * n - p[0][1])
    for i in range(1, n - 1):
        if pmx < (p[0][i]*(qc*n -p[0][i])):
            pmx = p[0][i] * (qc * n - p[0][i])
    print "pmx =", pmx
    print "qc*n = ", qc*n
    print "(D*pmx)/(Es*(qc*n - psum)) = ", (D*pmx)/(Es*(qc*n - psum))
    print "D*qm + (psum*Es*qc)/pmx = ", D*qm + (psum*Es*qc)/pmx
    if (qc*n <= psum) or (v < (D*pmx)/(Es*(qc*n - psum))) or (Em < D*qm + (psum*Es*qc)/pmx):
        # 如果不满足条件，则返回False
        P_Schedule.append(False)
        return P_Schedule
    Tl = (D * qc * n) / (v * (qc * n - psum))
    Tu = (Es*qc*n)/(pmx)
    w = (psum/(qc*n))*T + D/v
    # 如果满足VPGS条件，则返回对应的值
    # 将P的几个参数保存到list中，并进行返回
    P_Schedule.append(Tl)
    P_Schedule.append(Tu)
    P_Schedule.append(w)
    return P_Schedule

if __name__ == "__main__":
    print "begin……"
    print judging_whether_scheduled(Es, v, qm, qc, nl)

