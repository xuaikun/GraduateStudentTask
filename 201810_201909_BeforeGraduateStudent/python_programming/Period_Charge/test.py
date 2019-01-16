# encoding: utf-8


import numpy as np
import random
import time
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator
import TourConstructionProblem as T
import A_Star_Algorithm as A
import os
my_color =['b', 'g', 'r', 'c', 'm', 'y', 'k']
my_style = ['-', '--', '-.', ':']
my_logo = ['.', 'o', 'v', '^', '>', '<', '1', '2', '3', '4', 's', 'p', '*']
png_path = "E:\\00000000000graduate-study\\GraduateStudentTask\\201810_201909_BeforeGraduateStudent\\png"
# 数据初始化
N = 2   # 假设我有N辆电单车
Max_Node = 1
edge_n = 10000 # 假设定义的二维空间范围是 edge_n * edge_n
obstacles_Num = 5 # 障碍 数量
kedu = 40 # 坐标刻度
p = 1   # 表示障碍的边长 为pm
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
    
    ax.text(x_list[0], y_list[0], S_Flag, fontsize=20)
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
    # 设置生成图片保存路径
    partPath = [str(int(time.time()))]
    origin_path = partPath[0] + 'origin.png'  
    All_path = os.path.join(png_path, origin_path)
    plt.savefig(All_path)
    plt.xlim([0 - 1,edge_n + 1]) #设置绘图X边界                                                                                                   
    plt.ylim([0 - 1,edge_n + 1]) #设置绘图Y边界
    plt.show()
    return

# 将节点前后连接起来
def NodeToOtherNodeLink(x, y, obstacle_coordinate_new):
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
    
    ax.text(x_list[0], y_list[0],  S_Flag, fontsize=20)
    for i in range(1, len(y_list)):
        ax.text(x_list[i], y_list[i], i, fontsize = 10)
    # 将节点连接构成回路
    # 首先判断 R 中有几个子集，
    # 一个一个子集相连，然后再显示
    # 这个画图的 函数 应该重新设计 
    q = len(x_list) - 1
    CSL_string = my_color[random.randint(0, len(my_color)-1)]
    CSL_string = CSL_string + my_style[random.randint(0, len(my_style)-1)]
    CSL_string = CSL_string + my_logo[random.randint(0, len(my_logo)-1)]
    for i in range(0, q):
        # 前后连接
        x_temp = [x_list[i], x_list[i+1]]
        y_temp = [y_list[i], y_list[i+1]]
        plt.plot(x_temp, y_temp, CSL_string,linewidth=1)
        if i == q - 1:
            print 'i =', i
            x_temp = [x_list[i + 1], x_list[0]]
            y_temp = [y_list[i + 1], y_list[0]]
            plt.plot(x_temp, y_temp, CSL_string,linewidth=1)
    ax.legend(loc='best', edgecolor='black')
    plt.xlim([0 - 1,edge_n + 1]) #设置绘图X边界                                                                                                   
    plt.ylim([0 - 1,edge_n + 1]) #设置绘图Y边界
    plt.show()
    return 

# 将子回路首尾连接起来
def ChildrenTourConstruction(x_new, y_new, obstacle_coordinate_new, R_new):
    
    print "R_new =", R_new
    print "x_new =", x_new
    print "y_new =", y_new
    # Num = 0
    MC_Num = 0
    # for i in range(0, len(x_new)):
    # MC_Num.append(str(i)
    # 图例显示的标志
    plt.figure('Scatter fig')
    plt.title('S & Node')
    ax = plt.gca()
    # 每个节点用红圈圈表示出来
    ax.scatter(x_new, y_new,color = 'r',label = 'Node', marker = 'o')
    # 给每个节点标号
    for k in range(1, N + 1):
        ax.text(x_new[0][k], y_new[0][k], k, fontsize = 10)
    legend_flag = True
    for i in range(0, N + 1):
        if  len(R[i][0]) != 1:
            list_new = R[i][0]
            x = []
            y = []
            for j in range(0, len(list_new)):
                x.append(x_new[0][list_new[j]])
                y.append(y_new[0][list_new[j]])
        
            print "x =", x
            print "y =", y
            #分别存放所有点的横坐标和纵坐标，一一对应
            x_list = x
            y_list = y
            
             # 障碍坐标区域使用绿色表示
            # 障碍横坐标x的范围
            if legend_flag is True:
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
                # ax.scatter(x_list, y_list,color = 'r',label = 'Node', marker = 'o')
                # ax.scatter(x_obstacle_list, y_obstacle_list,color = 'g', marker = '8')
                # S点为红色正方形，并且大一点
                ax.scatter(x_list[0], y_list[0], s = 100, color = 'k',label = 'S', marker = 's')
                
                ax.text(x_list[0], y_list[0], S_Flag, fontsize=20)
                legend_flag = False
            # 给每个节点标号
            # for k in range(1, len(y_list)):
            #     ax.text(x_list[k], y_list[k], list_new[k], fontsize = 10)
            # 将节点连接构成回路
            q = len(x_list) - 1
            
            CSL_string = my_color[random.randint(0, len(my_color)-1)]
            CSL_string = CSL_string + my_style[random.randint(0, len(my_style)-1)]
            CSL_string = CSL_string + my_logo[random.randint(0, len(my_logo)-1)]
            print "CSL_string =", CSL_string
            MC_Num_temp = MC_Num 
            label_value = 'MCV' + str(MC_Num_temp)
            for i in range(0, q):
                # 前后连接
                print "CSL_string =", CSL_string
                x_temp = [x_list[i], x_list[i+1]]
                y_temp = [y_list[i], y_list[i+1]]
                # label 表示线的颜色
                plt.plot(x_temp, y_temp, CSL_string, linewidth=1)
                if i == q - 1:
                    print 'i =', i
                    print "CSL_string =", CSL_string
                    x_temp = [x_list[i + 1], x_list[0]]
                    y_temp = [y_list[i + 1], y_list[0]]
                    plt.plot(x_temp, y_temp, CSL_string, label = label_value, linewidth=1)
            MC_Num = MC_Num + 1
    plt.grid()
    plt.legend(loc='best', edgecolor='black')
    ax.legend(loc='best', edgecolor='black')
    plt.xlim([0 - 1,edge_n + 1]) #设置绘图X边界                                                                                                   
    plt.ylim([0 - 1,edge_n + 1]) #设置绘图Y边界
    plt.show()
    return 

# 随机生成障碍区域
'''
for i in range(0, obstacles_Num):
    x_down[0][i] = (random.randint(1, edge_n))
    y_down[0][i] = (random.randint(1, edge_n))

print "x_down =", x_down
print "y_down =", y_down
# 保证障碍在要操作的二维区域内
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
'''
x_down[0] = [1,1,1,4,9]
x_up[0] = [2,2,2,5,10]
y_down[0] = [3,6,7,8,9]
y_up[0] = [4,7,8,9,10]
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
N_x[0] = [0, 2.72,2.87]
N_y[0] = [0, 9.03, 1.25]
alpha[0]= [0, 107.0,146.0]
Es_i[0]=[0, 840000,840000]
N_i = [0,1,2]
'''
for i in range(1, N+1):
    print "i =", i
    N_x[0][i] = round(random.uniform(1, edge_n), 2)
    N_y[0][i] = round(random.uniform(1, edge_n), 2)

    # 每辆点电单车的电量，初始化为840Kj
    Es_i[0][i] = 840000
    # 电车运行方向的初始化
    alpha[0][i] = random.randint(0, 360)
    N_i.append(i)
'''   
# 程序开始运行，计时开始
start_time = time.time()

print "N_i =", N_i

 # 将节点坐标复制
N_x_new = N_x
N_y_new = N_y
# 随机生成每辆电单车的功率
'''
for i in range(1, N + 1):
        # 单位为 W
        P_i[0][i] = round(random.uniform(95, 115), 2)
        # P_i[0][i] = round(random.uniform(150, 180), 2)
        # P_i_temp[0][i] = P_i[0][i]
'''
P_i[0] = [0, 1, 5]
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
Max_Node = P_i_Max_Node
S_Flag = 'S' + '&' + str(Max_Node)
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
NodeToOtherNodeLink(x, y, obstacle_coordinate)
print "N_x_new =", N_x_new
print "N_y_new =", N_y_new
print "len(x)= ", len(x)
print "obstacle_coordinate =", obstacle_coordinate
print "obstacles_Num =", obstacles_Num

print "开始构造子回路"
print "\n#################"
print "******************"
print "$$$$$$$$$$$$$$$$$$$\n"
# distance = T.CreateDistanceMatrix(N, N_x_new, N_y_new, len(x), N_i, obstacle_coordinate, obstacles_Num)
# T.PrintNewMatrix(distance, len(x))

R_list0 = []
R_list1 = []
R_list0.append(0)
R_list1.append(0)
for i in range(1, N):
    R_list0.append(N_i[i])
    R_list1.append(N_i[i+N/2])

# 用来保存充电回来的子集
R = np.empty([N + 1, 1], list)
for i in range(0, N + 1):
    R[i][0] = ' '
R[0][0] = R_list0
R[1][0] = R_list1
print "R =", R
ChildrenTourConstruction(N_x_new, N_y_new, obstacle_coordinate, R)

first_coordinate = []
second_coordinate = []

first_coordinate.append(1)
first_coordinate.append(10000)

second_coordinate.append(10000)
second_coordinate.append(1)

print "first_coordinate =", first_coordinate
print "second_coordinate =", second_coordinate
print "测试A*算法用时："
a_star_time_start = time.time()
result = A.a_star_algorithm(edge_n, first_coordinate, second_coordinate, obstacle_coordinate, obstacles_Num, True)
a_star_time_end = time.time()

print "A*算法用时：", (a_star_time_end - a_star_time_start), 's'
print "result =", result
