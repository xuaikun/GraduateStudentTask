# encoding: utf-8

import numpy as np
import random
import time
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator
import TourConstructionProblem as T
import A_Star_Algorithm as A

# 数据初始化
N = 5   # 假设我有N辆电单车

edge_n = 100 # 假设定义的二维空间范围是 edge_n * edge_n
obstacles_Num = 5 # 障碍 数量
kedu = 10
# 用来保存充电回来的子集
R = np.empty([N + 1, 1], list)
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

x_down = np.empty([1, obstacles_Num], int)
x_up = np.empty([1, obstacles_Num], int)
y_down = np.empty([1, obstacles_Num], int)
y_up = np.empty([1, obstacles_Num], int)

def AllNodeLink(x, y, obstacle_coordinate_new):
    x_list = x
    y_list = y
    
    # 调整生成的图片的大小
    plt.rcParams['figure.figsize'] = (10.0, 10.0) # 设置figure_size尺寸
    #创建图并命名
    plt.figure('Scatter fig')
    plt.title('S & Node')
    ax = plt.gca()
    
    plt.grid()
    # 障碍坐标区域使用绿色表示
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
       
        plt.fill_between(range(x_down_value, x_up_value+1), y_down_value , y_up_value, facecolor='green')
    ax.scatter((x_down_list[0][1]+x_up_list[0][1])/2, (
             y_down_list[0][1]+y_up_list[0][1])/2, color = 'green',label = 'Obstacle', marker = 's')
    # 图片坐标刻度设置
    # 2000*2000
    # ax.xaxis.set_major_locator(MultipleLocator(100))
    # ax.yaxis.set_major_locator(MultipleLocator(40))
    
    ax.xaxis.set_major_locator(MultipleLocator(kedu))
    ax.yaxis.set_major_locator(MultipleLocator(kedu))
    #设置x轴、y轴名称
    ax.set_xlabel('X(m)')
    ax.set_ylabel('Y(m)')
    
    print "x_list =", x_list
    print "y_list =", y_list
    # 每个节点用红圈圈表示出来
    ax.scatter(x_list, y_list,color = 'r',label = 'Node', marker = 'o')
    # ax.scatter(x_obstacle_list, y_obstacle_list,color = 'g', marker = '8')
    # S点为红色正方形，并且大一点
    ax.scatter(x_list[0], y_list[0], s = 100, color = 'k',label = 'S', marker = 's')
 
    ax.text(x_list[0], y_list[0], 'S', fontsize=20)
    for i in range(1, len(y_list)):
        ax.text(x_list[i], y_list[i], i, fontsize = 10)
    # 将所有节点与充电桩S连起来
    # for j in range(1, len(y_list)):
        # 两两连接
        # x_temp = [x_list[0], x_list[j]]
        # y_temp = [y_list[0], y_list[j]]
        # 调节途中线条颜色，粗细
        # plt.plot(x_temp, y_temp, 'b',linewidth=0.5)
    ax.legend(loc='best', edgecolor='black')
    # plt.legend(loc='best',frameon=False) #去掉图例边框
    # plt.legend(loc='best',edgecolor='blue') #设置图例边框颜色
    # plt.legend(loc='best',facecolor='blue') #设置图例背景颜色,若无边框,参数无效
    plt.xlim([0 - 1,edge_n + 1]) #设置绘图X边界                                                                                                   
    plt.ylim([0 - 1,edge_n + 1]) #设置绘图Y边界
    plt.show()
    return

# 将节点前后连接起来
def NodeToOtherNodeLink(x, y, label, obstacle_coordinate_new):
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
     # 障碍坐标区域使用绿色表示
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
       
        plt.fill_between(range(x_down_value, x_up_value+1), y_down_value , y_up_value, facecolor='green')
    ax.scatter((x_down_list[0][1]+x_up_list[0][1])/2, (
             y_down_list[0][1]+y_up_list[0][1])/2, color = 'green',label = 'Obstacle', marker = 's')
    # 图片坐标刻度设置
    # 2000*2000
    # ax.xaxis.set_major_locator(MultipleLocator(100))
    # ax.yaxis.set_major_locator(MultipleLocator(40))
    ax.xaxis.set_major_locator(MultipleLocator(kedu))
    ax.yaxis.set_major_locator(MultipleLocator(kedu))
    #设置x轴、y轴名称
    ax.set_xlabel('X(m)')
    ax.set_ylabel('Y(m)')
    
    print "x_list =", x_list
    print "y_list =", y_list
     # 每个节点用红圈圈表示出来
    ax.scatter(x_list, y_list,color = 'r',label = 'Node', marker = 'o')
    # ax.scatter(x_obstacle_list, y_obstacle_list,color = 'g', marker = '8')
    # S点为红色正方形，并且大一点
    ax.scatter(x_list[0], y_list[0], s = 100, color = 'k',label = 'S', marker = 's')
    
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
    ax.legend(loc='best', edgecolor='black')
    plt.xlim([0 - 1,edge_n + 1]) #设置绘图X边界                                                                                                   
    plt.ylim([0 - 1,edge_n + 1]) #设置绘图Y边界
    plt.show()
    return 

# 随机生成障碍区域

for i in range(0, obstacles_Num):
    x_down[0][i] = (random.randint(1, edge_n))
    y_down[0][i] = (random.randint(1, edge_n))

print "x_down =", x_down
print "y_down =", y_down
# 保证障碍在要操作的二维区域内
p = 1   # 表示障碍的边长 为pm
for i in range(0, obstacles_Num):
    if x_down[0][i] < edge_n - p:
        x_up[0][i] = x_down[0][i] + p
    else:
        x_temp = x_down[0][i]
        x_up[0][i] = x_temp
        x_down[0][i] = x_temp - p
    if y_down[0][i] < edge_n - p:
        y_up[0][i] = y_down[0][i] + p
    else:
        y_temp = y_down[0][i]
        y_up[0][i] = y_temp
        y_down[0][i] = y_temp - p

# x_down[0] = [1,1,1,4,9]
# x_up[0] = [2,2,2,5,10]
# y_down[0] = [3,6,7,8,9]
# y_up[0] = [4,7,8,9,10]
print "x_down =", x_down
print "x_up =", x_up
print "y_down =", y_down
print "y_up =", y_up
obstacle_coordinate.append(x_down)
obstacle_coordinate.append(x_up)
obstacle_coordinate.append(y_down)
obstacle_coordinate.append(y_up)
print "obstacle_coordinate =", obstacle_coordinate

N_i = []
v = 5   # 初始化电单车的运行速度为 5m/s
N_i.append(0)
for i in range(1, N+1):
    print "i =", i
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
S_x_flag = True
S_y_flag = True
# S 的坐标比较讲究，直接放在功耗最大的节点旁边
S_x = N_x_new[0][P_i_Max_Node]
S_y = N_y_new[0][P_i_Max_Node]
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
NodeToOtherNodeLink(x, y, 'g', obstacle_coordinate)
print "N_x_new =", N_x_new
print "N_y_new =", N_y_new
print "len(x)= ", len(x)
print "obstacle_coordinate =", obstacle_coordinate
print "obstacles_Num =", obstacles_Num
x1 = 2
y1 = 9

first_coordinate = []
first_coordinate.append(x1)
first_coordinate.append(y1)
x2 = 9
y2 = 9

second_coordinate = []
second_coordinate.append(x2)
second_coordinate.append(y2)

print "无障碍"
distance_no_obstacle = A.a_star_algorithm(first_coordinate, second_coordinate, obstacle_coordinate, obstacles_Num, False)
print "distance_no_obstacle =", distance_no_obstacle
print "有障碍"
x1 = 2
y1 = 9

first_coordinate = []
first_coordinate.append(x1)
first_coordinate.append(y1)
x2 = 9
y2 = 9

second_coordinate = []
second_coordinate.append(x2)
second_coordinate.append(y2)
distance_obstacle = A.a_star_algorithm(first_coordinate, second_coordinate, obstacle_coordinate, obstacles_Num, True)
print "distance_obstacle =", distance_obstacle

# distance = T.CreateDistanceMatrix(N_x_new, N_y_new, len(x), N_i, obstacle_coordinate, obstacles_Num)
# T.PrintNewMatrix(distance, len(x))
R_list0 = [0,1,2]
R_list1 = [0,3,4]
R_list2 = [0, 5]
# 用来保存充电回来的子集
R = np.empty([N + 1, 1], list)
for i in range(0, N + 1):
    R[i][0] = ' '
R[0][0] = R_list0
R[1][0] = R_list1
R[2][0] = R_list2
for i in range(0, N + 1):
    if  len(R[i][0]) != 1:
        list_new = R[i][0]
        print "type(list_new) is", type(list_new)
        print "len(list_new) =", len(list_new)
        print "list_new =", list_new
        x = []
        y = []
        for j in range(0, len(list_new)):
            x.append(N_x_new[0][list_new[i]])
            y.append(N_x_new[0][list_new[i]])
        print "x =", x
        print "y =", y
        print "obstacle_coordinate =", obstacle_coordinate
        NodeToOtherNodeLink(x, y, 'g', obstacle_coordinate)

