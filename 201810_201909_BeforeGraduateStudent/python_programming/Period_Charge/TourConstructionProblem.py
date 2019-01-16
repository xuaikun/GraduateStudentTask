# encoding: utf-8
# 本程序主要实现，充电回路的构造

# 算法功能：充电回路的构造
# 算法输入：电单车在二维空间中的坐标以及功率
# 算法输出： 充电回路子回路结果

import numpy as np
import random
import A_Star_Algorithm as A
import JudgingWhetherScheduled as B
import matplotlib.pyplot as plt
import time
import math
from matplotlib.ticker import MultipleLocator
import os
# 图片保存路径
png_path = "E:\\00000000000graduate-study\\GraduateStudentTask\\201810_201909_BeforeGraduateStudent\\png"


# 自己的调色板
my_color =['b', 'g', 'r', 'c', 'm', 'y', 'k']
my_style = ['-', '--', '-.', ':']
# my_logo = ['.', 'o', 'v', '^', '>', '<', '1', '2', '3', '4', 's', 'p', '*']
# 数据初始化
N = 5   # 假设我有N辆电单车  会影响程序运行的时间
edge_n = 200 # 假设定义的二维空间范围是 edge_n * edge_n 影响构造充电子回路
obstacles_Num =20  # 障碍个数
kedu = 10  # 表示坐标间隔
p = 10   # 表示障碍的边长 为10m
S_Flag = 'S'
alpha_value = 30 # 转向的度数
# Em = 72000  # MC的总能量为72000 kj
# 1J=1s*1W
# 1KJ=1000J
Em = 150000
qm = 55  # Mc移动功耗为qm = 10 J/m
qc = 40  # qc*n 为能量传输率，qc= 4.45 W
nl = 0.5  # 类似于效率一样，占比多少 n = 0.5
T = 7200  # 充电周期需要知道10s
vm = 5  # MC的移动速度0.3m/s
v = 3   # 初始化电单车的运行速度为 5m/s
# 随机生成每辆电单车的功率
    # 定义 每个节点消耗的功率
P_i = np.empty([1, N + 1], float)
# 将从小到大排序的节点消耗功率，保存到P_i_temp临时数组中，以便后期查询知道其时那个节点的功率
P_i_temp = np.empty([1, N + 1], float)
# 重新标号后的电单车的消耗功率
P_i_new = np.empty([1, N + 1], float)
# 每辆电单车的能量
Es_i = np.empty([1, N + 1], float)
# 重新编号后电单车的能量
Es_i_new = np.empty([1, N + 1], float)

# 每辆电单车的所要走的方向
alpha = np.empty([1, N + 1], float)
for i in range(1, N + 1):
    # 单位为 W
    P_i[0][i] = round(random.uniform(0.25, 0.8), 2)
    # P_i[0][i] = round(random.uniform(150, 180), 2)
    # 每辆点电单车的电量，初始化为840Kj
    # Es_i[0][i] = 840000
    Es_i[0][i] = 5600
    # 电车运行方向的初始化
    alpha[0][i] = random.randint(0, 360)
# 初始化电单车在二维空间中的坐标
N_x = np.empty([1, N + 1], float)
N_y = np.empty([1, N + 1], float)
# 重新编号后的节点的标号改变，对应的坐标的标号获得修改，相关值的大小未被修改
N_x_new = np.empty([1, N + 1], float)
N_y_new = np.empty([1, N + 1], float)

N_x_final = np.empty([1, N + 1], float)
N_y_final = np.empty([1, N + 1], float)





# 用来保存充电回来的子集
R = np.empty([N + 1, 1], list)

# 障碍坐标范围
obstacle_coordinate = []
x_down = np.empty([1, obstacles_Num], int)
x_up = np.empty([1, obstacles_Num], int)
y_down = np.empty([1, obstacles_Num], int)
y_up = np.empty([1, obstacles_Num], int)

# 定义选择排序 A
def select_sort(array):
    length = array.shape[1]
    for position_i in range(1, length - 1):
        min_position = position_i
        for position_j in range(position_i + 1, length):
            if array[0][min_position] > array[0][position_j]:
                min_position = position_j
        tmp = array[0][min_position]
        array[0][min_position] = array[0][position_i]
        array[0][position_i] = tmp
    return

# 分别存放所有点的横坐标和纵坐标，一一对应
# 将图中离散的点一一连接

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
        # 障碍填充绿色
        plt.fill_between(range(x_down_value, x_up_value+1), y_down_value , y_up_value, facecolor='green')
    # 障碍图例显示
    ax.scatter((x_down_list[0][1]+x_up_list[0][1])/2, (
             y_down_list[0][1]+y_up_list[0][1])/2, color = 'green',label = 'Obstacle', marker = 's')
    # 图片坐标刻度设置 
    ax.xaxis.set_major_locator(MultipleLocator(kedu))
    ax.yaxis.set_major_locator(MultipleLocator(kedu))
    #设置x轴、y轴名称
    ax.set_xlabel('X(m)')
    ax.set_ylabel('Y(m)')
    
    # 每个节点用红圈圈表示出来
    ax.scatter(x_list, y_list,color = 'r',label = 'Node', marker = 'o')
    # ax.scatter(x_obstacle_list, y_obstacle_list,color = 'g', marker = '8')
    # S点为红色正方形，并且大一点
    ax.scatter(x_list[0], y_list[0], s = 100, color = 'k',label = 'S', marker = 's')
    # 给S点标号
    ax.text(x_list[0], y_list[0], S_Flag, fontsize=20)
    for i in range(1, len(y_list)):
        ax.text(x_list[i], y_list[i], i, fontsize = 10)
    # 图例位置
    ax.legend(loc='upper rigth', edgecolor='black')
    # 保存生成的图片
    partPath = [str(int(time.time()))]
    origin_path = partPath[0] + 'origin.png'  
    All_path = os.path.join(png_path, origin_path)
    plt.savefig(All_path)
    
    plt.xlim([0 - 1,edge_n + 1]) #设置绘图X边界                                                                                                   
    plt.ylim([0 - 1,edge_n + 1]) #设置绘图Y边界
    plt.show()
    return

# 将节点前后连接起来
def NodeToOtherNodeLink(x, y, x_new,y_new, obstacle_coordinate_new):
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
        # 障碍填充绿色
        plt.fill_between(range(x_down_value, x_up_value+1), y_down_value , y_up_value, facecolor='green')
    # 障碍图例显示
    ax.scatter((x_down_list[0][1]+x_up_list[0][1])/2, (
             y_down_list[0][1]+y_up_list[0][1])/2, color = 'green',label = 'Obstacle', marker = 's')
    # 图片坐标刻度设置
    ax.xaxis.set_major_locator(MultipleLocator(kedu))
    ax.yaxis.set_major_locator(MultipleLocator(kedu))
    #设置x轴、y轴名称
    ax.set_xlabel('X(m)')
    ax.set_ylabel('Y(m)')
    # 每个节点用红圈圈表示出来
    ax.scatter(x_new, y_new,color = 'r',label = 'Node', marker = 'o')
    # ax.scatter(x_obstacle_list, y_obstacle_list,color = 'g', marker = '8')
    # S点为红色正方形，并且大一点
    
    ax.scatter(x_new[0], y_new[0], s = 100, color = 'k',label = 'S', marker = 's')
    # S点图例
    ax.text(x_new[0], y_new[0], 'S', fontsize=20)
    
    for i in range(1, len(y_new)):
        ax.text(x_new[i], y_new[i], i, fontsize = 20)
    # 将节点连接构成回路
    q = len(x_list) - 1
    CSL_string = my_color[2]
    # CSL_string = CSL_string + my_style[random.randint(0, len(my_style)-1)]
    # CSL_string = CSL_string + my_logo[random.randint(0, len(my_logo)-1)]
    for i in range(0, q):
        # 前后连接
        x_temp = [x_list[i], x_list[i+1]]
        y_temp = [y_list[i], y_list[i+1]]
        plt.plot(x_temp, y_temp, CSL_string,linewidth=1)
        # plt.plot(x_temp, y_temp,linewidth=1)
        if i == q - 1:
            # print 'i =', i
            x_temp = [x_list[i + 1], x_list[0]]
            y_temp = [y_list[i + 1], y_list[0]]
            plt.plot(x_temp, y_temp, CSL_string,linewidth=1)
            # plt.plot(x_temp, y_temp,linewidth=1)
    # 每个节点用红圈圈表示出来
    # ax.scatter(x_list, y_list,color = 'r',label = 'Node', marker = 'o')
    # 图例定位
    ax.legend(loc='upper right', edgecolor='black')
    # 保存生成的图片
    partPath = [str(int(time.time()))]
    origin_path = partPath[0] + 'Alllink.png'  
    All_path = os.path.join(png_path, origin_path)
    plt.savefig(All_path)
    plt.xlim([0 - 1,edge_n + 1]) #设置绘图X边界                                                                                                   
    plt.ylim([0 - 1,edge_n + 1]) #设置绘图Y边界
    plt.show()
    # plt.plot()
    return 

# 将子回路首尾连接起来
def ChildrenTourConstruction(x_new, y_new, obstacle_coordinate_new, R_result, S_Flag_new):
    result = R_result
    print "result = ", result
    print "len(result) = ", len(result)
    MC_Num = 0
    # 图例显示的标志
    plt.figure('Scatter fig')
    plt.title('S & Node')
    ax = plt.gca()
    # 每个节点用红圈圈表示出来
    ax.scatter(x_new, y_new,color = 'r',label = 'Node', marker = 'o')
    # 给每个节点标号MC_Num,N表示所有的节点都要标号
    for k in range(1, N + 1):
        ax.text(x_new[0][k], y_new[0][k], k, fontsize = 10)
    legend_flag = True
    for k in range(0, len(result)):
        R_list = result[k]
        z = R_list[1]
        # print "R_list =", R_list
        # print "z =", z
        # 打印检查产生的充电子回路有多少条，每条子回路由那些节点组成
        for p in range(0, z):
            # print "R_list[0][p] =", R_list[0][p]
            # print "R_list[0][p] =", len(R_list[0][p])
            R_value = R_list[0][p]
            # print "x =", R_value[0] 
            # print "len(R_value[0]) =", len(R_value[0])
            goal = R_value[0]
            # print "x_new = ", x_new
            # print "y_new = ", y_new
            # S点为红色正方形，并且大一点
            # print "p =", p
            if p == 0:
                if k == 0:
                    S_Flag = S_Flag_new
                else:
                    print "goal[0] =", goal[0]
                    S_Flag = 'S' + '&' + str(goal[0])
                ax.scatter(x_new[0][goal[0]], y_new[0][goal[0]], s = 100, color = 'k',label = S_Flag, marker = 's')
                ax.text(x_new[0][goal[0]], y_new[0][goal[0]], S_Flag, fontsize=20)
            # print "R_new =", R_new
            # print "x_new =", x_new
            # print "y_new =", y_new
            # 用于图例显示MC（）
            
            # for i in range(0, len(R_list[0][p])):
            # 表示存在回路才需要连线
            if  len(R_value[0]) != 1:
                list_new = R_value[0]
                x = []
                y = []
                for j in range(0, len(list_new)):
                    x.append(x_new[0][list_new[j]])
                    y.append(y_new[0][list_new[j]])
                #分别存放所有点的横坐标和纵坐标，一一对应
                x_list = x
                y_list = y
                print "x_list =", x_list
                print "y_list =", y_list
                
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
                        # 给障碍填充绿色
                        plt.fill_between(range(x_down_value, x_up_value+1), y_down_value , y_up_value, facecolor='green')
                    # 障碍图例显示
                    ax.scatter((x_down_list[0][1]+x_up_list[0][1])/2, (
                             y_down_list[0][1]+y_up_list[0][1])/2, color = 'green',label = 'Obstacle', marker = 's')
                    # 图片坐标刻度设置
                    ax.xaxis.set_major_locator(MultipleLocator(kedu))
                    ax.yaxis.set_major_locator(MultipleLocator(kedu))
                    #设置x轴、y轴名称
                    ax.set_xlabel('X(m)')
                    ax.set_ylabel('Y(m)')
                    legend_flag = False
                # 将节点连接构成回路
                # 获取当前回路节点个数
                q = len(x_list) - 1
                # 设置回路线条、格式、颜色等等
                CSL_string = my_color[random.randint(0, len(my_color)-1)]
                CSL_string = CSL_string + my_style[random.randint(0, len(my_style)-1)]
                # CSL_string = CSL_string + my_logo[random.randint(0, len(my_logo)-1)]
                # 计算当前是第几个回路
                MC_Num_temp = MC_Num 
                label_value = 'MCV' + str(MC_Num_temp)
                for i in range(0, q):
                    # 前后连接
                    x_temp = [x_list[i], x_list[i+1]]
                    y_temp = [y_list[i], y_list[i+1]]
                    # label 表示线的颜色
                    plt.plot(x_temp, y_temp, CSL_string, linewidth=1)
                    if i == q - 1:
                        x_temp = [x_list[i + 1], x_list[0]]
                        y_temp = [y_list[i + 1], y_list[0]]
                        plt.plot(x_temp, y_temp, CSL_string, label = label_value, linewidth=1)
                MC_Num = MC_Num + 1
    plt.grid()
    plt.legend(loc='upper right', edgecolor='black')
    ax.legend(loc='upper right', edgecolor='black')
    # 保存生成的图片
    partPath = [str(int(time.time()))]
    origin_path = partPath[0] + 'OneToOtherlink.png'  
    All_path = os.path.join(png_path, origin_path)
    plt.savefig(All_path)
    plt.xlim([0 - 1,edge_n + 1]) #设置绘图X边界                                                                                                   
    plt.ylim([0 - 1,edge_n + 1]) #设置绘图Y边界
    plt.show()
    return 


# 创建邻接矩阵，对于重新排序的节点创建邻接矩阵
def CreateDistanceMatrix(n_1, N_x_x, N_y_y, n, N_i_new, obstacle_coordinate_new, obstacle_num_new):
    # i和j表示第n个节点
    N_distance_temp = np.empty([n, n], float)
    for i in range(0, n):
        for j in range(0, n):
            N_distance_temp[i][j] = 0.0
    for i in range(0, n):
        for j in range(i, n):
            if i == j:
                # 自己到自己的距离为0
                N_distance_temp[i][j] = 0.0
            else:
                # i到j的距离，i<j,因为为无向图，可以使用N_distance[j][i]= N_distance[i][j] 将对称的位置赋相同的值
                # 将当前两个点的坐标提取出来
                
                x1 = N_x_x[0][i]
                y1 = N_y_y[0][i]
                
                first_coordinate = []
                first_coordinate.append(x1)
                first_coordinate.append(y1)
                x2 = N_x_x[0][j]
                y2 = N_y_y[0][j]
                
                second_coordinate = []
                second_coordinate.append(x2)
                second_coordinate.append(y2)
                # print "第一个坐标：", first_coordinate
                # print "第二个坐标：", second_coordinate
                result = A.a_star_algorithm(n_1, first_coordinate, second_coordinate, obstacle_coordinate_new, obstacle_num_new, True)
                # print "构造回路的result = ", result
                # 电单车i到电单车j（或电单车j到电单车i）的实际距离
                # 距离取两位小数，就好
                N_distance_temp[i][j] = round(result[3],2)
                N_distance_temp[j][i] = round((N_distance_temp[i][j]), 2)
    return N_distance_temp

# 创建新邻接矩阵，对于过程中创建的邻接矩阵
def CreateDistanceNewMatrix(n_1, N_x_x, N_y_y, n, N_i_new, obstacle_coordinate_new, obstacle_num_new):
    # i和j表示第n个节点
    N_distance_temp = np.empty([n, n], float)
    for i in range(0, n):
        for j in range(0, n):
            N_distance_temp[i][j] = 0.0
    for i in range(0, n):
        for j in range(i, n):
            if i == j:
                # 自己到自己的距离为0
                N_distance_temp[i][j] = 0.0
            else:
                # i到j的距离，i<j,因为为无向图，可以使用N_distance[j][i]= N_distance[i][j] 将对称的位置赋相同的值
                # 将当前两个点的坐标提取出来
               
                x1 = N_x_x[0][N_i_new[i]]
                y1 = N_y_y[0][N_i_new[i]]
                
                first_coordinate = []
                first_coordinate.append(x1)
                first_coordinate.append(y1)
                x2 = N_x_x[0][N_i_new[j]]
                y2 = N_y_y[0][N_i_new[j]]
                
                second_coordinate = []
                second_coordinate.append(x2)
                second_coordinate.append(y2)
                # print "第一个坐标：", first_coordinate
                # print "第二个坐标：", second_coordinate
                result = A.a_star_algorithm(n_1, first_coordinate, second_coordinate, obstacle_coordinate_new, obstacle_num_new, True)
                # print "构造回路的result = ", result
                # 电单车i到电单车j（或电单车j到电单车i）的实际距离
                # 距离取两位小数，就好
                N_distance_temp[i][j] = round(result[3],2)
                N_distance_temp[j][i] = round((N_distance_temp[i][j]), 2)
    return N_distance_temp

# 重新打印邻接矩阵
def PrintNewMatrix(N_distance_parm, n):
    print "新的无向图："
    for i in range(0, n):
        N_distance_list = []
        for j in range(0, n):
            N_distance_list.append(N_distance_parm[i][j])
        print N_distance_list
    return

# 构建充电子回路
def ChangeChildrenTour(Es_sort, Node_sort, P_temp, N_distance_temp, first_flag):
    result_new = []
    # 初始化充电子回路的子集
    # 用来保存充电回来的子集
    # print "进入while之前Node_sort =", len(Node_sort)
    # 只要Node_sort里面处理S以外还有节点
    while(len(Node_sort) > 1):
        # print "在while里面Node_sort =", len(Node_sort)
        # 每次Node_sort的值都会变，所以每次顺序和个数都会修改
        Node_sort_new = []
        # 咱们把S点放进去
        Node_sort_new.append(Node_sort[0])
        # 节点数比N的数量少一个，因为服务站S不算在其中
        N = len(Node_sort) - 1
        R_temp = np.empty([N + 1, 1], list)
        for i in range(0, N + 1):
            R_temp[i][0] = ' '
        i = 1
        z = 0
        count = 0
        # 用作记录是否重新创建回路的标志
        R_list_flag = True
        R_list = []
        # 可以添加回路成功的标志
        success_flag = True
        # 还没执行到最后一个节点
        while i <= N:
            if len(Node_sort_new) == 0:
                Node_sort_new.append(Node_sort[0])
            for k in range(i, N + 1):
                if len(Node_sort_new) == 0:
                    Node_sort_new.append(Node_sort[0])
                # print "返回开始时i =", i
                # 记录是否重新创建回路的标志
                if R_list_flag is True:
                    R_list = []
                    R_list.append(Node_sort[0])
                # 将电单车Nk加入Rz里面
                # print "每次k都会+1 k =", k
                # 只要你将Node_sort[k]节点添加到R_list中，则我们需要将Node_sort[k]从Node_sort中移除
                R_list.append(Node_sort[k])
                Node_sort_len = len(Node_sort_new)
                # 保证最后的位置有相同的值，先取出来，如果后面没有调度则下一次操作
                # print "len(Node_sort_new) =", len(Node_sort_new)
                # print "Node_sort_new = ", Node_sort_new
                # print "Node_sort =", Node_sort 
                # 保证 当前的值相等，且 长度不能为0
                if Node_sort_len != 0:
                    if Node_sort[k] == Node_sort_new[Node_sort_len - 1]:
                        Node_sort_new.remove(Node_sort_new[Node_sort_len - 1])
                        if len(Node_sort_new) == 0:
                            Node_sort_new.append(Node_sort[0])
                # print "添加节点进入子回路后 Node_Flag =", Node_Flag 
                # Node_sort.remove(Node_sort[k])
                R_list_flag = False
                # 应用可调度性判定算法判定再加入Nk后Rz的可调度性
                # 如果Rz不可以可调度，则将Nk从回路中取出
                # 可调度性判断
               
                # print "加入了Nk节点的R_list =", R_list
                # 返回可调度性
                # result = True 可调度
                # result = False 不可调度
                result = B.judging_whether_scheduled(P_temp, Em, qc, qm, nl, R_list, vm, N_distance_temp, T)
                # 最后一组值的操作
                if success_flag is False and result is True and k == N:
                    # print "最后一组结果可以在这保存"
                    R_temp[z][count] = R_list
                    i = k + 1
                    break
                if result is False:
                    # 说明当前这个点不可调度，从子回路里面取出，并将它添加到另一个列表中用作下一次操作
                    # 因为考虑到问题：S的标号和节点标号相同，会被一起删除，先将它标号替换，然后再修改回来
                    Node_first_value = Node_sort[0]
                    if Node_sort[0] != 0:
                        Node_sort[0] = 0
                    R_list.remove(Node_sort[k])
                    Node_sort[0] = Node_first_value
                    # if first_flag is False:
                    #    Node_sort[0] = Node_sort[k]
                    # print "不可调度 Node_Flag =", Node_Flag 
                    # 说明当前的点并没有加入充电子回路
                    Node_sort_new.append(Node_sort[k])
                    # print "len(Node_sort_new) =", len(Node_sort_new)
                    # print "Node_sort_new = ", Node_sort_new
                    # print "Node_sort =", Node_sort
                    # 说明只有S在里面
                    # 下一次从k + 1遍历
                    # print "不可以被调度时，i应该被修改了i = ", i, "并且k =", k
                    # print "删除了Nk节点的R_list = ", R_list
                    # print "z =", z
                    # print "count =", count
                    # print "R[", z, "][0]=", R[z][count]
                    # print "对于不可调度有其他情况，每次先判断当前R_list的长度 len(R_list)=", len(R_list)
                    if len(R_list) == 1:
                        i = k + 1
                        z = z
                    # 说明R_list里面不仅仅包含S还有节点
                    # 下一次从k开始遍历
                    else:
                        # 先保存到列表类型的数组后再修改Z的值
                        # 如果len(R_list) == 1 表示只有充电桩，不需要放入回路
                        R_temp[z][count] = R_list
                        i = k
                        z = z + 1
                    # 表明只能有部分节点能加入同一个回路
                    success_flag = False
                    # print "退出下一轮遍历,并且k = k + 1"
                    R_list_flag = True
                    break
                i = k + 1
                
            # 表示所有的节点都可以放入同一个充电回路
            if success_flag is True:
                R_temp[z][count] = R_list
                # print "表示所有的节点都可以加入，R里面"
                # print "R[", z, "][", count, "]=", R_temp[z][count]
                i = N + 1
                break
        # print "跳出第二个死循环while"
        # print "Node_sort =", Node_sort
        # print "Node_sort_new =", Node_sort_new
        # print "len(Node_sort_new) =", len(Node_sort_new)
        # 我们要看看那个节点那个节点剩余的能量最少 反过来说就是功耗最大的
        # print "Node_sort_new =", Node_sort_new
        # print "len(Node_sort_new) =",len(Node_sort_new)
        if len(Node_sort_new) == 0 or len(Node_sort_new) == 1:
            # print "说明操作完了"
            Node_sort = Node_sort_new
            # print "R_temp = ", R_temp
            z = z + 1
            result = []
            result.append(R_temp)
            result.append(z)
            # print "在循环内的result =", result
            result_new.append(result)
        else:
            # print "没有操作玩，继续"
            # print "R_temp = ", R_temp
            result = []
            result.append(R_temp)
            result.append(z)
            # print "在循环内的result =", result
            result_new.append(result)
            Es_least = 840000   # 为了找出剩余能量最少的额点
            for i in range(1, len(Node_sort_new)):
                if Es_least >  Es_sort[0][Node_sort_new[i]] :
                    # 更新当前最大的功率的点
                    Es_least = Es_sort[0][Node_sort_new[i]]
                    # 保存功率最大的节点
                    Es_least_Node = Node_sort_new[i]
            # print "剩余能量最少的节点是：", Es_least_Node
            # least_Node = Es_least_Node
            # S_Flag = 'S' + '&' + str(Max_Node)
            Node_sort_new[0] = 0
            Node_sort = Node_sort_new
            # print "Node_sort =", Node_sort
            # print "Node_sort_new =", Node_sort_new
            Node_optimal_sort = []
            Node_optimal_sort.append(Node_sort[0])
            while len(Node_sort) != 1:
                # a 为剩余能量所占比重
                # b 为S到各电单车距离所占比重
                # P 为综合考虑剩余能量和S到各电单车距离所占比重考虑的最终值
                a = 0.2
                b = 1.0 - a
                # 只考虑了当前一次，所以得看看如何把所有的点考虑进去，每个点都要考虑到
                # 只考虑当前所剩下的节点，不包括S点（N_i[0]）
                # 假设开始时的P非常非常的大
                P = 10000000.0
                for i in range(1 , len(Node_sort)):
                    # print "Es_i[0][", N_i[i], "] =", Es_i[0][N_i[i]] 
                    # print "N_distance[0][", i, "] =", N_distance[0][i]
                    print "N_sorti[", i, "]=", Node_sort[i],"a*Es_i[0][Node_sort[i]] + b*N_distance_temp[Es_least_Node][Node_sort[i]]=", round((a*Es_i[0][Node_sort[i]] + b*N_distance_temp[Es_least_Node][Node_sort[i]]), 2)
                    if P > a*Es_i[0][Node_sort[i]] + b*N_distance_temp[Es_least_Node][Node_sort[i]]:
                        P = a*Es_i[0][Node_sort[i]] + b*N_distance_temp[Es_least_Node][Node_sort[i]]
                        optimal_N_i = Node_sort[i]
                # 将当前最优的节点从原来的列表中删除
                print "当前最优的节点是optimal_N_i =", optimal_N_i 
                Node_optimal_sort.append(optimal_N_i)
                Node_sort.remove(optimal_N_i)
                # 更新S的位置，S位置为上次充电的电单车
                Node_sort[0] = optimal_N_i
            
            print "最优操作顺序Node_optimal_sort =", Node_optimal_sort 
            Node_optimal_sort[0] = Es_least_Node
            Node_sort = Node_optimal_sort
            print "新的操作顺序 Node_sort= ", Node_sort
    return result_new

# 设置障碍
def Set_obstacles():    
    # 随机生成障碍区域
    for i in range(0, obstacles_Num):
        x_down[0][i] = (random.randint(1, edge_n - 30))
        y_down[0][i] = (random.randint(1, edge_n - 20))

    # print "x_down =", x_down
    # print "y_down =", y_down
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
    
    print "x_down =", x_down
    print "x_up =", x_up
    print "y_down =", y_down
    print "y_up =", y_up
    obstacle_coordinate.append(x_down)
    obstacle_coordinate.append(x_up)
    obstacle_coordinate.append(y_down)
    obstacle_coordinate.append(y_up)
    # print "obstacle_coordinate =", obstacle_coordinate
    return obstacle_coordinate



# 程序从这里开始执行
if __name__ == "__main__":
    programing_start_time = time.time()
    print "设置障碍"
    obstacle_coordinate = Set_obstacles()
    # 随机在1000*1000的空间内生成N辆电单车的坐标
    # 将生成的电单车坐标分别保存到x,y列表中
    # 保存节点序号的
    N_i = []
    N_i.append(0)
    
    # 保证电单车不在障碍区域内
    
    for i in range(1, N+1):
        print "准备生成第", i, "个节点坐标~~~~"
        N_x[0][i] = round(random.uniform(1, edge_n - 30), 2)
        N_y[0][i] = round(random.uniform(1, edge_n - 20), 2)
        # print "第一层for循环"
        # print "N_x[0][", i, "] =", N_x[0][i]
        # print "N_y[0][", i, "] =", N_y[0][i] 
        # j为终止条件
        j = 0
        # 要判断当前生成节点是否存在某一个障碍里面，所以要与每一个障碍区域进行比较
        while(j != obstacles_Num):
            # print "初始化j =", j
            j_temp = j
            equal_flag = True
            # 电单车坐标的设置，扩大到障碍外围至少+1m处
            if((N_x[0][i] > (x_down[0][j_temp]-1)) and (N_x[0][i] < (x_up[0][j_temp]+1))) and(
                (N_y[0][i] > (y_down[0][j_temp]-1)) and (N_y[0][i] < (y_up[0][j_temp]+1))):
                # print "初始j_temp =", j_temp
                j_temp = j_temp + 1
                if equal_flag is True:
                    # print "表示有电单车在障碍区域里面了，不符合,更新当前电单车坐标"
                    N_x[0][i] = round(random.uniform(1, edge_n), 2)
                    N_y[0][i] = round(random.uniform(1, edge_n), 2)
                    j_temp = 0
                    j = 0
                equal_flag = False
                if j_temp >= obstacles_Num:
                    j = j_temp 
                    # print "退出while（）"
                    break
            else:
                j = j + 1
                if j >= obstacles_Num:
                    # print "退出while（）"
                    break
       
        N_i.append(i)
    print "N_i =", N_i
    
     # 将节点坐标复制
    N_x_new = N_x
    N_y_new = N_y
    
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
    # 将电单车和服务站的分布呈现出来
    AllNodeLink(x, y,obstacle_coordinate)
    # 打印 初始化的节点啊的总能量
    Es_temp = []
    for i in range(1 , N + 1):
        Es_temp.append(Es_i[0][i])    
    print "开始时Es_i =", Es_temp 
     # 程序开始运行，计时开始
    start_time = time.time()
    time.sleep(1.5)
    # 备份N_i
    N_i_temp = N_i
    # 最优的操作节点排序
    Node_optimal_sort = []
    # 先把S点放进去
    Node_optimal_sort.append(0)
    # 每个节点都需要操作
    # 直到 len(N_i) = 1 表明里面只剩下当前充电桩S的位置节点
    # 就退出while
    
    # 打印生成的N辆电单车的坐标
    for i in range(0, N + 1):
        print "坐标为：", (N_x_new[0][i], N_y_new[0][i])
    alpha_temp = []
    for i in range(1 , len(N_i)):
        alpha_temp.append(alpha[0][i])
    x = []
    y = []
    x.append(N_x_new[0][0])
    y.append(N_y_new[0][0])
    # print "alpha =", alpha_temp 
    while 1 != len(N_i):
        
        # print "len(N_i) =", len(N_i)
        print "N_i =", N_i
        end_time = time.time()
        # print "while() start_time = ", start_time, "s"
        # print "while() end_time =", end_time, "s"
        t = (end_time - start_time)
        print "电单车运行的时间为：t =", t, "s"
        
        # 更新节点剩余的能量
        # 更新电单车的坐标，毕竟运行了好久，早已经离开了之前的初始的位置
        # 后期优化的时候我们应该考虑边界问题，如果当前的位置已经超过边界，则应该改变运动方向
        # 不仅仅是考虑边界，还得考虑运动过程中，遇到障碍应该避开
        for i in range(1, len(N_i)):
            Es_i[0][N_i[i]] = round((Es_i[0][N_i[i]] - P_i[0][N_i[i]]*t), 2)
            # 考虑到边界问题，先将改变之前的坐标记录下来
            N_x_new_temp = N_x_new[0][N_i[i]]
            N_y_new_temp = N_y_new[0][N_i[i]] 
            N_x_new_temp_new = N_x_new[0][N_i[i]]
            N_y_new_temp_new = N_y_new[0][N_i[i]]
            # print "备份的坐标为：", (N_x_new_temp, N_y_new_temp)
            N_x_new[0][N_i[i]] = round((N_x_new[0][N_i[i]] + v*t*math.cos(alpha[0][N_i[i]]/180.0*math.pi)), 2)
            N_y_new[0][N_i[i]] = round((N_y_new[0][N_i[i]] + v*t*math.sin(alpha[0][N_i[i]]/180.0*math.pi)), 2)
            # print "更新坐标为：", (N_x_new[0][N_i[i]], N_y_new[0][N_i[i]])
           
            # 检查坐标有没有超过边界, 表示超界了，表示当前需要改变运行方向
            # 为什么用while（）一定保证，坐标合理才能进行下一步操作
            # 为了在运动时避开障碍，设置两种状态
            # （1）在无障碍情况下计算距离
            # （2）在有障碍的情况下再计算一次距离
            # 如果两次距离相等，则中途没有障碍，可以直接到达目的位置
            # 反之，不相等，则中途出现障碍，不能直接到大目的地，需要改变运动方向
            # obstacle_flag = True 表明空间存在障碍
            while True:
                # 判断坐标是否越界
                while((N_x_new[0][N_i[i]] < 1 or N_x_new[0][N_i[i]] > edge_n) or (
                        N_y_new[0][N_i[i]] < 1 or N_y_new[0][N_i[i]] > edge_n )): 
                    # 更新电单车运行方向
                    # print "当前更新的坐标越界了：", (N_x_new[0][N_i[i]], N_y_new[0][N_i[i]])
                    # print "之前备份的坐标为：", (N_x_new_temp, N_y_new_temp)
                    # print "更新电单车运动方向"
                    # 改变方向幅度不能太大
                    # alpha[0][N_i[i]] = random.randint(0, 360)
                    # print "alpha[0][", N_i[i], "]=", alpha[0][N_i[i]]
                    alpha[0][N_i[i]] = (alpha[0][N_i[i]] + alpha_value)%360
                    # print "alpha[0][", N_i[i], "]=", alpha[0][N_i[i]]
                    
                    # 用上面备份的当前的坐标，重新往重新生成的方向前进
                    N_x_new[0][N_i[i]] = round((N_x_new_temp + v*t*math.cos(alpha[0][N_i[i]]/180.0*math.pi)), 2)
                    N_y_new[0][N_i[i]] = round((N_y_new_temp + v*t*math.sin(alpha[0][N_i[i]]/180.0*math.pi)), 2)
                    # N_x_new[0][N_i[i]] = 1.0
                    #N_y_new[0][N_i[i]] = 2.0
                    # print "越界重新更新坐标为：", (N_x_new[0][N_i[i]], N_y_new[0][N_i[i]])
                # 越界处理完成的标志，表示已经不越界
                crossing_flag = True
                # 还得保证当前终点不属于任何一个障碍区域内
                first_coordinate = []
                second_coordinate = []
                first_coordinate.append(N_x_new_temp)
                first_coordinate.append(N_y_new_temp)
                second_coordinate.append(N_x_new[0][N_i[i]])
                second_coordinate.append(N_y_new[0][N_i[i]])
                # print "备份的坐标（起始）first_coordinate =", first_coordinate
                # print "更新的坐标（终点）second_coordinate =", second_coordinate
                # print "有障碍"
                distance_obstacle =A.a_star_algorithm(edge_n, first_coordinate, second_coordinate, obstacle_coordinate, obstacles_Num, True)
               
                # 判断是否需要变向
                change_direction_flag = True
                # 如果终点坐标在障碍区域内，也需要改变运动方向
                if distance_obstacle[0] is False:
                    change_direction_flag = False
                    # print "这个坐标时在障碍区域内：", (N_x_new[0][N_i[i]], N_y_new[0][N_i[i]])
                    # print "之前备份的坐标为：", (N_x_new_temp, N_y_new_temp)
                    # print "更新电单车运动方向"
                    # 改变方向幅度不能太大
                    # alpha[0][N_i[i]] = random.randint(0, 360)
                    # print "alpha[0][", N_i[i], "]=", alpha[0][N_i[i]]
                    alpha[0][N_i[i]] = (alpha[0][N_i[i]] + alpha_value)%360
                    # print "alpha[0][", N_i[i], "]=", alpha[0][N_i[i]]
                    # 用上面备份的当前的坐标，重新往重新生成的方向前进
                    N_x_new[0][N_i[i]] = round((N_x_new_temp + v*t*math.cos(alpha[0][N_i[i]]/180.0*math.pi)), 2)
                    N_y_new[0][N_i[i]] = round((N_y_new_temp + v*t*math.sin(alpha[0][N_i[i]]/180.0*math.pi)), 2)
                    # print "障碍区域内的坐标更新为：", (N_x_new[0][N_i[i]], N_y_new[0][N_i[i]])
                
                else:
                    first_coordinate = []
                    second_coordinate = []
                    first_coordinate.append(N_x_new_temp_new)
                    first_coordinate.append(N_y_new_temp_new)
                    second_coordinate.append(N_x_new[0][N_i[i]])
                    second_coordinate.append(N_y_new[0][N_i[i]])
                    # print "备份的坐标（起始）first_coordinate =", first_coordinate
                    # print "更新的坐标（终点）second_coordinate =", second_coordinate
                    
                    # print "无障碍"
                    distance_no_obstacle =A.a_star_algorithm(edge_n, first_coordinate, second_coordinate, obstacle_coordinate, obstacles_Num, False)
                    # N_x_new[0][N_i[i]] = 1.0
                    # N_y_new[0][N_i[i]] = 2.0
                    print "distance_obstacle =", distance_obstacle
                    print "distance_no_obstacle =", distance_no_obstacle
                    # 表明两次同一点同一终点，所得距离不相等，说明行驶过程中遇到障碍，需要变向
                    # 终点在障碍区域内或者运动过程中遇到障碍都将改变即将到达的位置的坐标
                    if distance_no_obstacle[3] != distance_obstacle[3]:
                        change_direction_flag = False
                        # print "运行到这个坐标时遇到障碍：", (N_x_new[0][N_i[i]], N_y_new[0][N_i[i]])
                        # print "之前备份的坐标为：", (N_x_new_temp, N_y_new_temp)
                        # print "更新电单车运动方向"
                        # 改变方向幅度不能太大
                        # alpha[0][N_i[i]] = random.randint(0, 360)
                        # print "alpha[0][", N_i[i], "]=", alpha[0][N_i[i]]
                        alpha[0][N_i[i]] = (alpha[0][N_i[i]] + alpha_value)%360
                        # print "alpha[0][", N_i[i], "]=", alpha[0][N_i[i]]
                        
                        # 用上面备份的当前的坐标，重新往重新生成的方向前进
                        N_x_new[0][N_i[i]] = round((N_x_new_temp + v*t*math.cos(alpha[0][N_i[i]]/180.0*math.pi)), 2)
                        N_y_new[0][N_i[i]] = round((N_y_new_temp + v*t*math.sin(alpha[0][N_i[i]]/180.0*math.pi)), 2)
                        # print "遇到障碍重新更新坐标为：", (N_x_new[0][N_i[i]], N_y_new[0][N_i[i]])
                    if crossing_flag is True and change_direction_flag is True:
                        # print "起点到终点途中不存在障碍或终点不存在与障碍区域"
                        # print "退出while()"
                        break
        for i in range(0, len(N_i)):
            print "更新坐标为：", (N_x_new[0][N_i[i]], N_y_new[0][N_i[i]])
        print "################################\n"
        # print "延时10s，查看结果"
        # time.sleep(10)
        # 创建一个N*N的空降，生成邻接矩阵，构造无向图G(V,E)
        # N_distance[Ni][Nj] = x 表示Ni到Nj（或Nj到Ni）的距离为x
        N_distance = np.empty([len(N_i), len(N_i)], float)
        for i in range(0, len(N_i)):
            for j in range(0, len(N_i)):
                N_distance[i][j] = 0.0
       
        # 加入S点后，邻接矩阵每次都会更新，之后每次都会减少节点
        N_distance = CreateDistanceNewMatrix(edge_n, N_x_new, N_y_new, len(N_i), N_i, obstacle_coordinate, obstacles_Num)
        # 重新打印邻接矩阵
        PrintNewMatrix(N_distance, len(N_i))
        # 重新开始计算时间，为什么从这开始呢？主要是构建邻接矩阵比较花时间，而且期间所花不属于电单车运行时间
        print "################################\n"
        # print "延时10s，构造邻接矩阵"
        # time.sleep(10)
        start_time = time.time()
        
        time.sleep(1.8)
        # 只考虑当前所剩下的节点，不包括S点（N_i[0]）
        # 准备充电之前都在运行
        
        # 电单车能量到此开始计算减少了多少
        # 好好考虑 这个时间如何处理
        
        # S统计到各个节点的距离
        # 统计电单车剩余的能量
        N_distance_temp = []
        Es_temp = []
        for i in range(1 , len(N_i)):
            N_distance_temp.append(N_distance[0][i])
            Es_temp.append(Es_i[0][N_i[i]])  
        print "S到每辆电单车的距离 =", N_distance_temp
        print "结束时Es_i =", Es_temp 
        
        # a 为剩余能量所占比重
        # b 为S到各电单车距离所占比重
        # P 为综合考虑剩余能量和S到各电单车距离所占比重考虑的最终值
        a = 0.2
        b = 1.0 - a
        # 只考虑了当前一次，所以得看看如何把所有的点考虑进去，每个点都要考虑到
        # 只考虑当前所剩下的节点，不包括S点（N_i[0]）
        # 假设开始时的P非常非常的大
        P = 10000000.0
        for i in range(1 , len(N_i)):
            # print "Es_i[0][", N_i[i], "] =", Es_i[0][N_i[i]] 
            # print "N_distance[0][", i, "] =", N_distance[0][i]
            print "N_i[", i, "]=", N_i[i],"a*Es_i[0][N_i[i]] + b*N_distance[0][i]=", round((a*Es_i[0][N_i[i]] + b*N_distance[0][i]), 2)
            if P > a*Es_i[0][N_i[i]] + b*N_distance[0][i]:
                P = a*Es_i[0][N_i[i]] + b*N_distance[0][i]
                optimal_N_i = N_i[i]
        # 将当前最优的节点从原来的列表中删除
        print "当前最优的节点是optimal_N_i =", optimal_N_i 
        Node_optimal_sort.append(optimal_N_i)
        N_i.remove(optimal_N_i)
        # 更新S的位置，S位置为上次充电的电单车
        N_i[0] = optimal_N_i
        # print "延时5s看效果"
        # time.sleep(5)
        x.append(N_x_new[0][optimal_N_i])
        y.append(N_y_new[0][optimal_N_i])
        x_new = N_x_new[0]
        y_new = N_y_new[0]
        NodeToOtherNodeLink(x, y,x_new,y_new, obstacle_coordinate)
        # plt.show()
        # print "延时5s看效果"
        # time.sleep(5)
    
    #  最优的节点序列已经找出
    print "最优的节点序列Node_optimal_sort =", Node_optimal_sort
    # Node_optimal_sort = [0, 1, 3, 2]
    Node_New_sort = []
    # 对原来的排序进行改变     
    for i in range(0, len(Node_optimal_sort)):
        # 对节点重新排号
        Node_New_sort.append(i)
        N_x_final[0][i] = N_x_new[0][Node_optimal_sort[i]]
        N_y_final[0][i] = N_y_new[0][Node_optimal_sort[i]]
        # 将功率和剩余能量按排序后排列
        P_i_new[0][i] = round(P_i[0][Node_optimal_sort[i]], 2)
        Es_i_new[0][i] = round(Es_i[0][Node_optimal_sort[i]], 2)
    # 排序完成 
    x = N_x_final[0]
    y = N_y_final[0]
    
    
    print "N_x_new = ",N_x_new
    print "N_y_new = ",N_y_new
    print "重新编号的节点序列Node_New_sort =", Node_New_sort
    S_Flag = 'S' + '&' + str(Node_New_sort[1])
    print "N_x_final = ",N_x_final
    print "N_y_final = ",N_y_final
    
    print "P_i =", P_i
    print "P_i_new =", P_i_new
    print "Es_i =", Es_i
    print "Es_i_new =", Es_i_new    
    # print "len(Node_optimal_sort) =", len(Node_optimal_sort)
    N_distance = np.empty([len(Node_optimal_sort), len(Node_optimal_sort)], float)
    for i in range(0, len(Node_optimal_sort)):
        for j in range(0, len(Node_optimal_sort)):
            N_distance[i][j] = 0.0
    # print "邻接矩阵矩阵初始化"
    # PrintNewMatrix(N_distance, len(N_i))
    # 加入S点后，邻接矩阵每次都会更新，之后每次都会减少节点
    
    # 与上面的索引生成邻接矩阵不一样，这里是直接产生邻接矩阵
    # 这里是已经排好序的列表
    N_distance = CreateDistanceMatrix(edge_n, N_x_final, N_y_final, len(Node_optimal_sort), Node_optimal_sort, obstacle_coordinate, obstacles_Num)
    # 重新打印邻接矩阵
    PrintNewMatrix(N_distance, len(Node_optimal_sort))
    
    # 加入S点后，将电单车前后连接起来
    NodeToOtherNodeLink(x, y, x, y, obstacle_coordinate)
   
    print "开始构造子回路"
    
    # 构造充电子回路
    # P_i_new, N_distance 可以代表重新节点排序
    # result里面包含N个S中的回路，以及个数
    result = ChangeChildrenTour(Es_i_new, Node_New_sort, P_i_new, N_distance, True)
    
    ChildrenTourConstruction(N_x_final, N_y_final, obstacle_coordinate, result, S_Flag)
    programing_end_time = time.time()
    print "程序运行总共耗时：", (programing_end_time - programing_start_time), "s"