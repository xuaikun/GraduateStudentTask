# encoding: utf-8

import numpy as np
import random
import time
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator
# 数据初始化
N = 20   # 假设我有N辆电单车
edge_n = 500 # 假设定义的二维空间范围是 edge_n * edge_n
# 初始化电单车在二维空间中的坐标
N_x = np.empty([1, N + 1], float)
N_y = np.empty([1, N + 1], float)

# 每辆电单车的能量
Es_i = np.empty([1, N + 1], float)
# 每辆电单车的所要走的方向
alpha = np.empty([1, N + 1], float)
# 定义 每个节点消耗的功率
P_i = np.empty([1, N + 1], float)

# 分别存放所有点的横坐标和纵坐标，一一对应
# 将图中离散的点一一连接
obstacle_coordinate = []
obstacles_Num =20
x_down = np.empty([1, obstacles_Num], int)
x_up = np.empty([1, obstacles_Num], int)
y_down = np.empty([1, obstacles_Num], int)
y_up = np.empty([1, obstacles_Num], int)

for i in range(0, obstacles_Num):
    x_down[0][i] = (random.randint(1, edge_n))
    y_down[0][i] = (random.randint(1, edge_n))
print "x_down =", x_down
print "y_down =", y_down
# 保证障碍在要操作的二维区域内
for i in range(0, obstacles_Num):
    if x_down[0][i] < edge_n - 10:
        x_up[0][i] = x_down[0][i] + 10
    else:
        x_temp = x_down[0][i]
        x_up[0][i] = x_temp
        x_down[0][i] = x_temp - 10
    if y_down[0][i] < edge_n - 10:
        y_up[0][i] = y_down[0][i] + 10
    else:
        y_temp = y_down[0][i]
        y_up[0][i] = y_temp
        y_down[0][i] = y_temp - 10
print "x_down =", x_down
print "x_up =", x_up
print "y_down =", y_down
print "y_up =", y_up
obstacle_coordinate.append(x_down)
obstacle_coordinate.append(x_up)
obstacle_coordinate.append(y_down)
obstacle_coordinate.append(y_up)
print "obstacle_coordinate =", obstacle_coordinate

def AllNodeLink(x, y, obstacle_coordinate_new):
    x_list = x
    y_list = y
    
    # x_obstacle_list = obstacle_coordinate_new[0][0]
    # y_obstacle_list = obstacle_coordinate_new[1][0]
    # 调整生成的图片的大小
    plt.rcParams['figure.figsize'] = (10.0, 10.0) # 设置figure_size尺寸
    #创建图并命名
    plt.figure('Scatter fig')
    plt.title('S & Node')
    ax = plt.gca()
    
    plt.grid()
    # 障碍坐标区域使用绿色表示
    # range(40, 81) 表示选中横坐标的范围
    # 障碍横坐标x的范围
    x_down_list = obstacle_coordinate_new[0]
    x_up_list  = obstacle_coordinate_new[1]
    # 障碍纵坐标y的范围
    y_down_list = obstacle_coordinate_new[2]
    y_up_list = obstacle_coordinate_new[3]
    for j in range(0, obstacles_Num):
        x_down_value= x_down_list[0][j]
        x_up_value= x_up_list[0][j]
        y_down_value= y_down_list[0][j]
        y_up_value= y_up_list[0][j]
        
        plt.fill_between(range(x_down_value, x_up_value), y_down_value , y_up_value, facecolor='green')
    # 图片坐标刻度设置
    # 2000*2000
    # ax.xaxis.set_major_locator(MultipleLocator(100))
    # ax.yaxis.set_major_locator(MultipleLocator(40))
    
    ax.xaxis.set_major_locator(MultipleLocator(40))
    ax.yaxis.set_major_locator(MultipleLocator(20))
    #设置x轴、y轴名称
    ax.set_xlabel('X(m)')
    ax.set_ylabel('Y(m)')
    
    print "x_list =", x_list
    print "y_list =", y_list
    # 每个节点用红圈圈表示出来
    ax.scatter(x_list, y_list,color = 'r', marker = 'o')
    # ax.scatter(x_obstacle_list, y_obstacle_list,color = 'g', marker = '8')
    # S点为红色正方形，并且大一点
    ax.scatter(x_list[0], y_list[0], s = 100, color = 'k', marker = 's')
 
    ax.text(x_list[0], y_list[0], 'S', fontsize=20)
    for i in range(1, len(y_list)):
        ax.text(x_list[i], y_list[i], i, fontsize = 10)
    # 将所有节点与充电桩S连起来
    for j in range(1, len(y_list)):
        # 两两连接
        x_temp = [x_list[0], x_list[j]]
        y_temp = [y_list[0], y_list[j]]
        # 调节途中线条颜色，粗细
        plt.plot(x_temp, y_temp, 'b',linewidth=0.5)
            
    plt.xlim([0 - 1,edge_n + 1]) #设置绘图X边界                                                                                                   
    plt.ylim([0 - 1,edge_n + 1]) #设置绘图Y边界
    plt.show()
    return

# 将节点前后连接起来
def NodeToOtherNodeLink(x, y, label):
    print "x =", x
    print "y =", y
    #分别存放所有点的横坐标和纵坐标，一一对应
    x_list = x
    y_list = y
    
    #创建图并命名
    plt.figure('Scatter fig')
    plt.title('S & Node')
    ax = plt.gca()
    plt.grid()
    # 图片坐标刻度设置
    # 2000*2000
    # ax.xaxis.set_major_locator(MultipleLocator(100))
    # ax.yaxis.set_major_locator(MultipleLocator(40))
    ax.xaxis.set_major_locator(MultipleLocator(40))
    ax.yaxis.set_major_locator(MultipleLocator(20))
    #设置x轴、y轴名称
    ax.set_xlabel('X(m)')
    ax.set_ylabel('Y(m)')
    
    print "x_list =", x_list
    print "y_list =", y_list
    ax.scatter(x_list, y_list,color = 'red', marker = 'o')
    # S点为红色正方形，并且大一点
    ax.scatter(x_list[0], y_list[0], s = 100, color = 'k', marker = 's')
 
    ax.text(x_list[0], y_list[0], 'S', fontsize=20)
    for i in range(1, len(y_list)):
        ax.text(x_list[i], y_list[i], i, fontsize = 10)
    # 将节点连接构成回路
    for i in range(0, N):
        # 前后连接
        x_temp = [x_list[i], x_list[i+1]]
        y_temp = [y_list[i], y_list[i+1]]
        plt.plot(x_temp, y_temp, label,linewidth=1)
        if i == N - 1:
            print 'i =', i
            x_temp = [x_list[i + 1], x_list[0]]
            y_temp = [y_list[i + 1], y_list[0]]
            plt.plot(x_temp, y_temp, label,linewidth=1)
    plt.xlim([0 - 1,edge_n + 1]) #设置绘图X边界                                                                                                   
    plt.ylim([0 - 1,edge_n + 1]) #设置绘图Y边界
    plt.show()
    return 

N_i = []
N_i.append(0)
v = 5   # 初始化电单车的运行速度为 5m/s
for i in range(1, N+1):
    N_x[0][i] = round(random.uniform(1, edge_n), 2)
    N_y[0][i] = round(random.uniform(1, edge_n), 2)
    # 每辆点电单车的电量，初始化为840Kj
    Es_i[0][i] = 840000
    # 电车运行方向的初始化
    alpha[0][i] = random.randint(0, 360)
    N_i.append(i)
    
# 程序开始运行，计时开始
start_time = time.time()

print "N_i =", N_i

 # 将节点坐标复制
N_x_new = N_x
N_y_new = N_y
# 随机生成每辆电单车的功率
for i in range(1, N + 1):
    # 单位为 W
    P_i[0][i] = round(random.uniform(95, 115), 2)
    # P_i[0][i] = round(random.uniform(150, 180), 2)
    # P_i_temp[0][i] = P_i[0][i]

for i in range(1, N + 1):
    print "P_i[0][", i, "]=", P_i[0][i]

# 我们要看看那个节点那个节点剩余的能量最少 反过来说就是功耗最大的
P_i_Max = 0  # 初始化功率最大为P_i_Max = 0
for i in range(1, N + 1):
    if P_i_Max <  P_i[0][N_i[i]] :
        # 更新当前最大的功率的点
        P_i_Max = P_i[0][N_i[i]]
        # 保存功率最大的节点
        P_i_Max_Node = N_i[i]
print "剩余能量最少的节点是：", P_i_Max_Node

# S表示充电桩的位置，它的开始时坐标的确定，主要是，靠近剩余能量最小的节点或者说功率最大的点
S = []
# S 的坐标比较讲究，不能部署到边界上
if N_x_new[0][P_i_Max_Node] < edge_n - 20:
    S_x = N_x_new[0][P_i_Max_Node] + 20
else:
    S_x = N_x_new[0][P_i_Max_Node] - 20
    
if N_y_new[0][P_i_Max_Node] < edge_n - 20:
    S_y = N_y_new[0][P_i_Max_Node] + 20
else:
    S_y = N_y_new[0][P_i_Max_Node] - 20
S.append(S_x)
S.append(S_y)
print "充电桩坐标为：", (S[0], S[1])
# 表示充电桩S的坐标，加入了N中
N_x_new[0][0] = S[0]
N_y_new[0][0] = S[1]
x = []
y = []
for i in range(0, N + 1):
    x.append(N_x[0][i])
    y.append(N_y[0][i])
print "x =", x
print "y =", y
# 将电单车之间两两连接起来画图
AllNodeLink(x, y,obstacle_coordinate)
# 将哈密顿回路连接起来
NodeToOtherNodeLink(x, y, 'g')