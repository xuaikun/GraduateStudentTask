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

# 如果使用备份数据，则将 Backup_flag = True
# 第一次运行，需要设置Backup_flag = False, 若之后想利用上一次数据，则设置Backup_flag = True
Backup_flag = True

# 数据结果保存路径 (需要使用自己电脑的路径),自己建立一个result文件
result_path = "E:\\00000000000graduate-study\\GraduateStudentTask\\201810_201909_BeforeGraduateStudent\\python_programming\\Period_Charge\\result"

# 需要修改的变量

# 节点数目 从 50 到 200 变化，将100节点的实验先 做全
N = 50   # 假设我有N辆电单车  会影响程序运行的时间

# 修改MCV的变量 每次 只修改一个
# Em  数值从150kj 到400kj变化，间隔50kj变化
Em = 150000.0  # MC的总能量 j 
# 测试过程中从4m/s 到10m/s 变化
vm = 4.0  # MC的移动速度 m/s
# 师姐看看从多少变到多少吧，哈哈哈
qm = 55.0  # Mc移动功耗为 J/m
qc = 40.0  # MCV充电传输速率 W


# 确保数据结果保存路径一定存在
isExist = os.path.exists(result_path)
if not isExist:
    print "不存在该路径，创建对应路径"
    os.makedirs(result_path)


# 自主修改数据结果子的根目录
result_name = 'Node_' + str(int(N))
# print "result_name =", result_name

result_name = os.path.join(result_path, result_name)
isExist = os.path.exists(result_name)
if not isExist:
    print "不存在该路径，创建对应路径"
    os.makedirs(result_name)

# 自主修改数据结果子的子目录
childern_result_name = 'Em_' + str(int(Em)) + '_vm_' +str(int(vm)) + '_qm_' + str(int(qm)) + '_qc_' + str(int(qc))
# print "childern_result_name =", childern_result_name

childern_result_name = os.path.join(result_name, childern_result_name)
isExist = os.path.exists(childern_result_name)
if not isExist:
    print "不存在该路径，创建对应路径"
    os.makedirs(childern_result_name)

MCV_Tour_txt = os.path.join(childern_result_name, 'MCV_Tour_Set.txt')
result_txt = os.path.join(childern_result_name, 'MCV_Tour_Information.txt')

# print "MCV_Tour_txt =", MCV_Tour_txt
# print "result_txt =", result_txt

Obstacle_information_data_txt = os.path.join(result_name, 'Obstacle_information_data.txt')
Node_information_data_txt = os.path.join(result_name, 'Node_information_data.txt')
        
# 如果想获取过程图解，则设置 Process_Graph_flag = True
Process_Graph_flag = False

# 需要获取MCV真实运动路径时，将Real_Graph_flag置为True
Real_Graph_flag = False

# 自己的调色板
my_color =['b', 'g', 'r', 'c', 'm', 'y', 'k']

my_style = ['-', '--', '-.', ':']

# my_logo = ['.', 'o', 'v', '^', '>', '<', '1', '2', '3', '4', 's', 'p', '*']
# 数据初始化

edge_n = 1000 # 假设定义的二维空间范围是 edge_n * edge_n 影响构造充电子回路
obstacles_Num = 20  # 障碍个数
kedu = 50  # 表示坐标间隔
p = 10   # 表示障碍的边长 为10m
# 实际轨迹线路大小
linewidthvalue = 2
# 系统运行时间结束的标志
Time_Over_Flag = True
# 死亡节点个数
N_i_dead_num = 0
S_Flag = 'S'
alpha_value = 30 # 转向的度数

# 1J=1s*1W
# 1KJ=1000J
nl = 0.5  # 类似于效率一样，占比多少 n = 0.5
T = 7200.0  # 充电周期需要知道10s
v = 3.0   # 初始化电单车的运行速度为 5m/s
# 每个节点的能量
Es_i_value = 5600
# Es_i_value  = 560
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

# 每辆车充电所耗时长
Change_Time = np.zeros((1, N + 1), dtype = np.float)

# 每辆电单车的Pab值的计算
Pab_i = np.empty([1, N + 1], float)

# 每辆电单车的所要走的方向
alpha = np.empty([1, N + 1], float)

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

# 初始化邻接矩阵
N_distance = np.empty([N + 1, N + 1], float)
for i in range(0, N + 1):
    for j in range(0, N + 1):
        N_distance[i][j] = 0.0
# 两个节点之间的路径信息,用list来保存
Road_information = np.empty([N + 1, N + 1], list)
# 障碍坐标范围
obstacle_coordinate = []
x_down = np.empty([1, obstacles_Num], int)
x_up = np.empty([1, obstacles_Num], int)
y_down = np.empty([1, obstacles_Num], int)
y_up = np.empty([1, obstacles_Num], int)

# 定义选择排序 A 对二维数组进行排序
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
# 定义选择排序 A 对一维列表排序
def select_sort_new(array):
    length = len(array)
    for position_i in range(1, length - 1):
        min_position = position_i
        for position_j in range(position_i + 1, length):
            if array[min_position] > array[position_j]:
                min_position = position_j
        tmp = array[min_position]
        array[min_position] = array[position_i]
        array[position_i] = tmp
    return
# 定义选择排序 A 对二维列表排序，从第1个开始
def select_sort_Numsort(array):
    length = len(array)
    for position_i in range(1, length - 1):
        min_position = position_i
        for position_j in range(position_i + 1, length):
            if array[min_position][1] > array[position_j][1]:
                min_position = position_j
        tmp = array[min_position][1]
        temp1 = array[min_position][0]
        array[min_position][1] = array[position_i][1]
        array[min_position][0] = array[position_i][0]
        array[position_i][1] = tmp
        array[position_i][0] = temp1
    return

# 定义选择排序 A 对一组数据中，第二个数据进行排序 从第0个开始
def select_sort_Numsort_P(array):
    length = len(array)
    for position_i in range(0, length - 1):
        min_position = position_i
        for position_j in range(position_i + 1, length):
            if array[min_position][1] > array[position_j][1]:
                min_position = position_j
        tmp = array[min_position][1]
        temp1 = array[min_position][0]
        array[min_position][1] = array[position_i][1]
        array[min_position][0] = array[position_i][0]
        array[position_i][1] = tmp
        array[position_i][0] = temp1
    return
# 分别存放所有点的横坐标和纵坐标，一一对应
# 将图中离散的点一一连接， 将Node和S显示在图片中
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
        x_down_value= x_down_list[j]
        x_up_value= x_up_list[j]
        y_down_value= y_down_list[j]
        y_up_value= y_up_list[j]
        # 障碍填充绿色
        plt.fill_between(range(x_down_value, x_up_value+1), y_down_value , y_up_value, facecolor='green')
    # 障碍图例显示
    ax.scatter((x_down_list[1]+x_up_list[1])/2, (
             y_down_list[1]+y_up_list[1])/2, color = 'green',label = 'Obstacle', marker = 's')
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
    ax.legend(loc='upper right', edgecolor='black')
    # 保存生成的图片
    partPath = [str(int(time.time()))]
    origin_path = partPath[0] + 'origin.png'  
    
    All_path = os.path.join(childern_result_name, origin_path)
    plt.savefig(All_path)
    
    plt.xlim([0 - 1,edge_n + 1]) #设置绘图X边界                                                                                                   
    plt.ylim([0 - 1,edge_n + 1]) #设置绘图Y边界
    plt.show()
    return

# 将节点前后连接起来，构建子回路
def NodeToOtherNodeLink(R_list_temp, x, y, x_new,y_new, obstacle_coordinate_new, label_flag, Road_information_temp):
    print "模拟图###########\n"
    print "x =", x
    print "y =", y
    #分别存放所有点的横坐标和纵坐标，一一对应
    print "R_list_temp =", R_list_temp
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
        x_down_value= x_down_list[j]
        x_up_value= x_up_list[j]
        y_down_value= y_down_list[j]
        y_up_value= y_up_list[j]
        # 障碍填充绿色
        plt.fill_between(range(x_down_value, x_up_value+1), y_down_value , y_up_value, facecolor='green')
    # 障碍图例显示
    ax.scatter((x_down_list[1]+x_up_list[1])/2, (
             y_down_list[1]+y_up_list[1])/2, color = 'green',label = 'Obstacle', marker = 's')
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
    S_Flag = 'S' + '&' + str(label_flag)
    ax.text(x_new[0], y_new[0], S_Flag, fontsize=20)
    
    for i in range(1, len(y_new)):
        ax.text(x_new[i], y_new[i], i, fontsize = 20)
    # 将节点连接构成回路
    q = len(x_list) - 1
    CSL_string = my_color[2]
    # CSL_string = CSL_string + my_style[random.randint(0, len(my_style)-1)]
    # CSL_string = CSL_string + my_logo[random.randint(0, len(my_logo)-1)]
    print "q =", q
    for i in range(0, q):
        x_temp = [x_list[i], x_list[i+1]]
        y_temp = [y_list[i], y_list[i+1]]
        plt.plot(x_temp, y_temp, CSL_string,linewidth=1)
        if i == q - 1:
            x_temp = [x_list[i + 1], x_list[0]]
            y_temp = [y_list[i + 1], y_list[0]]
            plt.plot(x_temp, y_temp, CSL_string,linewidth=1)
    # 每个节点用红圈圈表示出来
    # ax.scatter(x_list, y_list,color = 'r',label = 'Node', marker = 'o')
    # 图例定位
    ax.legend(loc='upper right', edgecolor='black')
    # 保存生成的图片
    partPath = [str(int(time.time()))]
    origin_path = partPath[0] + 'Alllink.png'  
    All_path = os.path.join(childern_result_name, origin_path)
    plt.savefig(All_path)
    plt.xlim([0 - 1, edge_n + 1]) #设置绘图X边界                                                                                                   
    plt.ylim([0 - 1, edge_n + 1]) #设置绘图Y边界
    plt.show()
    if Real_Graph_flag is True:
        print "实际轨迹图@@@@@@@@\n"
        
        print "x =", x
        print "y =", y
        #分别存放所有点的横坐标和纵坐标，一一对应
        print "R_list_temp =", R_list_temp
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
        S_Flag = 'S' + '&' + str(label_flag)
        ax.text(x_new[0], y_new[0], S_Flag, fontsize=20)
        
        for i in range(1, len(y_new)):
            ax.text(x_new[i], y_new[i], i, fontsize = 20)
        # 将节点连接构成回路
        q = len(x_list) - 1
        CSL_string = my_color[2]
        # plt.plot()
        for i in range(0, q):
            # 前后连接
            
            # R_list_temp[i] 第i个节点
            # R_list_temp[i+1]第i+个节点
            # Road_information[R_list_temp[i]][R_list_temp[i+1]] 表示第i个节点 到 第i+1个节点的所有路径
            # len(Road_information[R_list_temp[i]][R_list_temp[i+1]]) 表示两点之间的路径的长度即为数量
            # print "Road_information[", R_list_temp[i], "][", R_list_temp[i+1], "] =", Road_information[R_list_temp[i]][R_list_temp[i+1]]
            # print "len(Road_information[R_list_temp[i]][R_list_temp[i+1]] ) =", len(Road_information[R_list_temp[i]][R_list_temp[i+1]])
            if len(Road_information[R_list_temp[i]][R_list_temp[i+1]]) == 1:
                Road_information[R_list_temp[i]][R_list_temp[i+1]] = [[R_list_temp[i], R_list_temp[i]], [R_list_temp[i+1], R_list_temp[i+1]]]
            
            Road_Node_i_To_Node_j_list = Road_information[R_list_temp[i]][R_list_temp[i+1]] 
            # print "Road_Node_i_To_Node_j_list =", Road_Node_i_To_Node_j_list 
            # print "len(Road_Node_i_To_Node_j_list ) =", len(Road_Node_i_To_Node_j_list)
            
                
            for m in range(0, len(Road_Node_i_To_Node_j_list) - 1):
                x_temp = [Road_Node_i_To_Node_j_list[m][0], Road_Node_i_To_Node_j_list[m + 1][0]]
                y_temp = [Road_Node_i_To_Node_j_list[m][1], Road_Node_i_To_Node_j_list[m + 1][1]]
            
                plt.plot(x_temp, y_temp, CSL_string,linewidth= linewidthvalue)
                # plt.plot(x_temp, y_temp,linewidth=1)
            if i == q - 1:
                # print "Road_information[", R_list_temp[i + 1], "][", R_list_temp[0], "] =", Road_information[R_list_temp[i + 1]][R_list_temp[0]]
                # print "len(Road_information[R_list_temp[i + 1]][R_list_temp[0]] ) =", len(Road_information[R_list_temp[i + 1]][R_list_temp[0]])
                if len(Road_information[R_list_temp[i + 1]][R_list_temp[0]]) == 1:
                    Road_information[R_list_temp[i + 1]][R_list_temp[0]] = [[R_list_temp[i + 1], R_list_temp[i + 1]], [R_list_temp[0], R_list_temp[0]]]
                
                Road_Node_i_To_Node_j_list = Road_information[R_list_temp[i + 1]][R_list_temp[0]] 
                # print "Road_Node_i_To_Node_j_list =", Road_Node_i_To_Node_j_list 
                # print "len(Road_Node_i_To_Node_j_list ) =", len(Road_Node_i_To_Node_j_list)
                for m in range(0, len(Road_Node_i_To_Node_j_list) - 1):
                    # print 'i =', i
                    # x_temp = [x_list[i + 1], x_list[0]]
                    # y_temp = [y_list[i + 1], y_list[0]]
                    x_temp = [Road_Node_i_To_Node_j_list[m][0], Road_Node_i_To_Node_j_list[m + 1][0]]
                    y_temp = [Road_Node_i_To_Node_j_list[m][1], Road_Node_i_To_Node_j_list[m + 1][1]]
                    plt.plot(x_temp, y_temp, CSL_string,linewidth= linewidthvalue)
                    # plt.plot(x_temp, y_temp,linewidth=1)
        # 每个节点用红圈圈表示出来
        # ax.scatter(x_list, y_list,color = 'r',label = 'Node', marker = 'o')
        # 图例定位
        ax.legend(loc='upper right', edgecolor='black')
        # 保存生成的图片
        partPath = [str(int(time.time()))]
        origin_path = partPath[0] + 'Alllink.png'  
        All_path = os.path.join(childern_result_name, origin_path)
        plt.savefig(All_path)
        plt.xlim([0 - 1, edge_n + 1]) #设置绘图X边界                                                                                                   
        plt.ylim([0 - 1, edge_n + 1]) #设置绘图Y边界
        plt.show()
        # plt.plot()
    return 

# 将子回路首尾连接起来
def ChildrenTourConstruction(x_new, y_new, obstacle_coordinate_new, R_result, S_Flag_new):
    print "模拟轨迹##########\n"
    Style_num = 0
    Color_num = 0
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
    ChildernTour_list_sum = []
    ChildernTour_list_tour = []
    for k in range(0, len(result)):
        R_list = result[k]
        print "R_list =", R_list
        print "len(R_list) = ", len(R_list)
        if R_list[len(R_list) - 1] == 0:
            z = len(R_list) - 1
        else:
            z = len(R_list)
        print "z =", z
        # print "R_list =", R_list
        # print "z =", z
        # 打印检查产生的充电子回路有多少条，每条子回路由那些节点组成
        for p in range(0, z):
            # print "R_list[0][p] =", R_list[0][p]
            # print "R_list[0][p] =", len(R_list[0][p])
            R_value = R_list[p]
            # print "x =", R_value[0] 
            # print "len(R_value[0]) =", len(R_value[0])
            goal = R_value
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
            if  len(R_value) != 1:
                list_new = R_value
                Es_ChildernTour = 0.0
                Time_ChildernTour = 0.0
                ChildernTour_Name = MC_Num
                ChildernTour_list = []
                
                ChildernTour_list.append(ChildernTour_Name)
                
                for i in range(1, len(list_new)):
                    # print "节点", list_new[i], "剩余能量 Es =", Es_i[0][list_new[i]]
                    Es_ChildernTour = Es_ChildernTour + (Es_i_value - Es_i[0][list_new[i]])
                    Time_ChildernTour = round((Time_ChildernTour + Change_Time[0][list_new[i]]),2)
                # 求距离D
                D = 0.0
                Euclid_Distance = 0.0
                # 计算一个充电回路的距离
                for i in range(0, (len(list_new) - 1)):
                    # 实际距离
                    D = D + N_distance[list_new[i]][list_new[i + 1]]
                    # 欧几里得距离
                    Euclid_Distance = Euclid_Distance + np.sqrt(np.power((x_new[0][list_new[i]] - x_new[0][list_new[i + 1]]), 2) +np.power((
                            y_new[0][list_new[i]] - y_new[0][list_new[i + 1]]), 2))
                # 将最后一个节点连接会对应的服务站S 
                D = D + N_distance[list_new[len(list_new) - 1]][list_new[0]]
                Euclid_Distance = Euclid_Distance + np.sqrt(np.power((x_new[0][list_new[len(list_new) - 1]] - x_new[0][list_new[0]]), 2) +np.power((
                            y_new[0][list_new[len(list_new) - 1]] - y_new[0][list_new[0]]), 2))
                # 求MCV走完整个回路需要多少时间
                changetour_time = D/vm
                # 走完整个回路所需能量（移动耗能）
                changetour_Es = changetour_time*qm
                
                ChildernTour_list.append(round(Es_ChildernTour, 2))
                ChildernTour_list.append(round(changetour_Es, 2))
                ChildernTour_list.append(Em)
                ChildernTour_list.append(Time_ChildernTour)
                ChildernTour_list.append(changetour_time)
                ChildernTour_list.append(T)
                ChildernTour_list.append(len(list_new) - 1)
                ChildernTour_list.append(round(D, 2))
                ChildernTour_list.append(round(Euclid_Distance, 2))   
                # for i in range(0, len(list_new)):
               
                # 把回路集合保存下来
                # pathname = 'MCV' + str(MC_Num) +'.txt'
                # print "pathname =", pathname
                # np.savetxt(pathname, ChildernTour_list_tour, fmt='%d')
                
                # print "ChildernTour_list =", ChildernTour_list
                ChildernTour_list_tour.append(list_new)
                ChildernTour_list_sum.append(ChildernTour_list)
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
                        x_down_value= x_down_list[j]
                        x_up_value= x_up_list[j]
                        y_down_value= y_down_list[j]
                        y_up_value= y_up_list[j]
                        # 给障碍填充绿色
                        plt.fill_between(range(x_down_value, x_up_value+1), y_down_value , y_up_value, facecolor='green')
                    # 障碍图例显示
                    ax.scatter((x_down_list[1]+x_up_list[1])/2, (
                             y_down_list[1]+y_up_list[1])/2, color = 'green',label = 'Obstacle', marker = 's')
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
                
                CSL_string = my_color[Color_num]
                CSL_string = CSL_string + my_style[Style_num]
                if Color_num < (len(my_color) - 1):
                    Color_num = Color_num + 1 
                if Color_num == (len(my_color) - 1):
                    if Style_num < (len(my_style) - 1):
                        Style_num = Style_num + 1
                        Color_num = 0
                    else:
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
    All_path = os.path.join(childern_result_name, origin_path)
    plt.savefig(All_path)
    plt.xlim([0 - 1,edge_n + 1]) #设置绘图X边界                                                                                                   
    plt.ylim([0 - 1,edge_n + 1]) #设置绘图Y边界
    plt.show()
    
    if Real_Graph_flag is True:
        print "实际轨迹@@@@@@@@@@@@\n"
        
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
            print "R_list =", R_list
            print "len(R_list) = ", len(R_list)
            if R_list[len(R_list) - 1] == 0:
                z = len(R_list) - 1
            else:
                z = len(R_list)
            print "z =", z
            # print "R_list =", R_list
            # print "z =", z
            # 打印检查产生的充电子回路有多少条，每条子回路由那些节点组成
            for p in range(0, z):
                # print "R_list[0][p] =", R_list[0][p]
                # print "R_list[0][p] =", len(R_list[0][p])
                R_value = R_list[p]
                # print "x =", R_value[0] 
                # print "len(R_value[0]) =", len(R_value[0])
                goal = R_value
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
                if  len(R_value) != 1:
                    list_new = R_value
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
                   
                    R_list_temp = list_new
                    for i in range(0, q):
                        # 前后连接
                        
                        # R_list_temp[i] 第i个节点
                        # R_list_temp[i+1]第i+个节点
                        # Road_information[R_list_temp[i]][R_list_temp[i+1]] 表示第i个节点 到 第i+1个节点的所有路径
                        # len(Road_information[R_list_temp[i]][R_list_temp[i+1]]) 表示两点之间的路径的长度即为数量
                        # print "Road_information[", R_list_temp[i], "][", R_list_temp[i+1], "] =", Road_information[R_list_temp[i]][R_list_temp[i+1]]
                        # print "len(Road_information[R_list_temp[i]][R_list_temp[i+1]] ) =", len(Road_information[R_list_temp[i]][R_list_temp[i+1]])
                        if len(Road_information[R_list_temp[i]][R_list_temp[i+1]]) == 1:
                            Road_information[R_list_temp[i]][R_list_temp[i+1]] = [[R_list_temp[i], R_list_temp[i]], [R_list_temp[i+1], R_list_temp[i+1]]]
                        
                        Road_Node_i_To_Node_j_list = Road_information[R_list_temp[i]][R_list_temp[i+1]] 
                        # print "Road_Node_i_To_Node_j_list =", Road_Node_i_To_Node_j_list 
                        # print "len(Road_Node_i_To_Node_j_list ) =", len(Road_Node_i_To_Node_j_list)
                        
                            
                        for m in range(0, len(Road_Node_i_To_Node_j_list) - 1):
                            x_temp = [Road_Node_i_To_Node_j_list[m][0], Road_Node_i_To_Node_j_list[m + 1][0]]
                            y_temp = [Road_Node_i_To_Node_j_list[m][1], Road_Node_i_To_Node_j_list[m + 1][1]]
                        
                            plt.plot(x_temp, y_temp, CSL_string,linewidth= linewidthvalue)
                            
                            # plt.plot(x_temp, y_temp,linewidth=1)
                        if i == q - 1:
                            # print "Road_information[", R_list_temp[i + 1], "][", R_list_temp[0], "] =", Road_information[R_list_temp[i + 1]][R_list_temp[0]]
                            # print "len(Road_information[R_list_temp[i + 1]][R_list_temp[0]] ) =", len(Road_information[R_list_temp[i + 1]][R_list_temp[0]])
                            if len(Road_information[R_list_temp[i + 1]][R_list_temp[0]]) == 1:
                                Road_information[R_list_temp[i + 1]][R_list_temp[0]] = [[R_list_temp[i + 1], R_list_temp[i + 1]], [R_list_temp[0], R_list_temp[0]]]
                            
                            Road_Node_i_To_Node_j_list = Road_information[R_list_temp[i + 1]][R_list_temp[0]] 
                            # print "Road_Node_i_To_Node_j_list =", Road_Node_i_To_Node_j_list 
                            # print "len(Road_Node_i_To_Node_j_list ) =", len(Road_Node_i_To_Node_j_list)
                            for m in range(0, len(Road_Node_i_To_Node_j_list) - 1):
                                # print 'i =', i
                                # x_temp = [x_list[i + 1], x_list[0]]
                                # y_temp = [y_list[i + 1], y_list[0]]
                                x_temp = [Road_Node_i_To_Node_j_list[m][0], Road_Node_i_To_Node_j_list[m + 1][0]]
                                y_temp = [Road_Node_i_To_Node_j_list[m][1], Road_Node_i_To_Node_j_list[m + 1][1]]
                                # plt.plot(x_temp, y_temp, CSL_string,linewidth=5)
                                plt.plot(x_temp, y_temp, CSL_string, linewidth= linewidthvalue)
                            plt.plot(x_temp, y_temp, CSL_string, label = label_value, linewidth= linewidthvalue)
                    MC_Num = MC_Num + 1
        plt.grid()
        plt.legend(loc='upper right', edgecolor='black')
        ax.legend(loc='upper right', edgecolor='black')
        # 保存生成的图片
        partPath = [str(int(time.time()))]
        origin_path = partPath[0] + 'OneToOtherlink.png'  
        All_path = os.path.join(childern_result_name, origin_path)
        plt.savefig(All_path)
        plt.xlim([0 - 1,edge_n + 1]) #设置绘图X边界                                                                                                   
        plt.ylim([0 - 1,edge_n + 1]) #设置绘图Y边界
        plt.show()
    result = []
    result.append(ChildernTour_list_tour)
    result.append(ChildernTour_list_sum)
    return result

# 创建新邻接矩阵，对于过程中创建的邻接矩阵
def CreateDistanceNewMatrix(Road_information_temp, N_distance_temp, n_1, N_x_x, N_y_y, n, N_i_new, m, N_i_change, obstacle_coordinate_new, obstacle_num_new):
    # i和j表示第n个节点
    # N_distance_temp = np.empty([n, n], float)
    # for i in range(0, n):
    #     for j in range(0, n):
    #         N_distance_temp[i][j] = 0.0
    for i in range(0, m):
        for j in range(0, n):
            if N_i_change[i] == j:
                # 自己到自己的距离为0
                N_distance_temp[N_i_change[i]][j] = 0.0
                Road_information_temp[N_i_change[i]][j] = [[j, j],[j, j]]
            else:
                # i到j的距离，i<j,因为为无向图，可以使用N_distance[j][i]= N_distance[i][j] 将对称的位置赋相同的值
                # 将当前两个点的坐标提取出来
               
                x1 = N_x_x[0][N_i_new[N_i_change[i]]]
                y1 = N_y_y[0][N_i_new[N_i_change[i]]]
                
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
                N_distance_temp[N_i_change[i]][j] = round(result[3],2)
                N_distance_temp[j][N_i_change[i]] = round((N_distance_temp[N_i_change[i]][j]), 2)
                
                # result[5] 表示当前两个点的运动路径
                # 把路径保存到列表数组中不错
                Road_information_temp[N_i_change[i]][j] = result[5]
                Road_information_temp[j][N_i_change[i]] = Road_information_temp[N_i_change[i]][j]
                # print "Road_information =\n", Road_information
    result_new = []
    result_new.append(N_distance_temp)
    result_new.append(Road_information_temp)         
    return result_new

# 设置障碍
def Set_obstacles():    
    # 随机生成障碍区域
    for i in range(0, obstacles_Num):
        x_down[0][i] = (random.randint(1, edge_n))
        y_down[0][i] = (random.randint(1, edge_n))

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
    
    # print "x_down =", x_down
    # print "x_up =", x_up
    # print "y_down =", y_down
    # print "y_up =", y_up
    obstacle_coordinate.append(x_down[0])
    obstacle_coordinate.append(x_up[0])
    obstacle_coordinate.append(y_down[0])
    obstacle_coordinate.append(y_up[0])
    return obstacle_coordinate

# 程序从这里开始执行
if __name__ == "__main__":
    
    programing_start_time = time.time()
    
    if Backup_flag is False:
        print "设置障碍"
        obstacle_coordinate = Set_obstacles()
        
        # 为了让服务站S处于中心，设置节点1的功耗最大
        for i in range(1, N + 1):
            # 单位为 W
            if i == 1:
                P_i[0][1] =0.80
            else:
                P_i[0][i] = round(random.uniform(0.25, 0.8), 2)
            # P_i[0][i] = round(random.uniform(150, 180), 2)
            # 每辆点电单车的电量，初始化为840Kj
            # Es_i[0][i] = 840000
            Es_i[0][i] = Es_i_value
            # 电车运行方向的初始化
            alpha[0][i] = random.randint(0, 360)
        # 随机在1000*1000的空间内生成N辆电单车的坐标
        # 将生成的电单车坐标分别保存到x,y列表中
    
        # 保证电单车不在障碍区域内
        
        for i in range(1, N+1):
            # print "准备生成第", i, "个节点坐标~~~~"
            # 第一个节点放中间，第一个服务站S就在中间
            if i == 1:
                N_x[0][i] = round(random.uniform(edge_n/2 - 50, edge_n/2 + 50), 2)
                N_y[0][i] = round(random.uniform(edge_n/2 - 50, edge_n/2 + 50), 2)
            else:
                N_x[0][i] = round(random.uniform(1, edge_n), 2)
                N_y[0][i] = round(random.uniform(1, edge_n), 2)
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
                        if i == 1:
                            N_x[0][i] = round(random.uniform(edge_n/2 - 50, edge_n/2 + 50), 2)
                            N_y[0][i] = round(random.uniform(edge_n/2 - 50, edge_n/2 + 50), 2)
                        else:
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
                        
        # 添加障碍
        # print "obstacle_coordinate =", obstacle_coordinate
        # 以整形保存
        # np.savetxt("Obstacle_information_data.txt",obstacle_coordinate, fmt='%d')
        np.savetxt(Obstacle_information_data_txt, obstacle_coordinate, fmt='%d')
        # 添加节点相关信息
        data = []
        # 添加节点功率
        data.append(P_i[0])
        # print "P_i[0] =", P_i[0] 
        # 添加节点能量
        data.append(Es_i[0])
        # print "Es_i[0] =", Es_i[0]
        # 添加初始运动方向
        data.append(alpha[0])
        # print "alpha[0] =", alpha[0]
        # 添加节点坐标
        data.append(N_x[0])
        # print "N_x[0] =", N_x[0]
        data.append(N_y[0])
        # print "N_y[0] =", N_y[0]
        # 保存节点相关数据
        # np.savetxt("Node_information_data.txt", data)
        np.savetxt(Node_information_data_txt, data)

   
    if Backup_flag is True:
        obstacle_coordinate = np.loadtxt(Obstacle_information_data_txt, dtype = np.int)
        # print "obstacle_coordinate =", obstacle_coordinate
        # print "len(obstacle_coordinate) =", len(obstacle_coordinate )
        
        # 导入节点相关数据
        data = np.loadtxt(Node_information_data_txt)
        # print "data =", data
        # print "len(data) =", len(data)
        P_i[0] = data[0]
        # print "P_i[0] =", P_i[0] 
        Es_i[0] = data[1]
        # print "Es_i[0] =", Es_i[0]
        alpha[0] = data[2]
        # print "alpha[0] =", alpha[0]
        N_x[0] = data[3]
        # print "N_x[0] =", N_x[0]
        N_y[0] = data[4]
        # print "N_y[0] =", N_y[0]
    
    # 保存节点序号的
    N_i = []
    for l in range(0, N + 1):     
        N_i.append(l)
    
    print "N_i =", N_i
    # 将节点坐标复制
    N_x_new = N_x
    N_y_new = N_y
    
    print "P_i[0]=", P_i[0]
    
    # 我们要看看那个节点那个节点剩余的能量最少 反过来说就是功耗最大的
    P_i_Max = 0  # 初始化功率最大为P_i_Max = 0
    for i in range(1, N + 1):
        if P_i_Max <  P_i[0][N_i[i]] :
            # 更新当前最大的功率的点
            P_i_Max = P_i[0][N_i[i]]
            # 保存功率最大的节点
            P_i_Max_Node = N_i[i]
    print "功率最大的节点是：", P_i_Max_Node
    Max_Node = P_i_Max_Node
    N_i[0] = Max_Node
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
    # print "x =", x
    # print "y =", y
    # 将电单车和服务站的分布呈现出来
    AllNodeLink(x, y,obstacle_coordinate)
    # 打印 初始化的节点啊的总能量
    Es_temp = []
    for i in range(1 , N + 1):
        Es_temp.append(Es_i[0][i])    
    # print "开始时Es_i =", Es_temp 
     # 程序开始运行，计时开始
    start_time = time.time()
    time.sleep(1.5)
    # 备份N_i
    N_i_temp = []
    for i in range(0, len(N_i)):
        N_i_temp.append(N_i[i])
    # print "N_i =", N_i
    # print "N_i_temp =", N_i_temp
    # 最优的操作节点排序
    Node_optimal_sort = []
    # 先把S点放进去
    Node_optimal_sort.append(0)
    # 每个节点都需要操作
    # 直到 len(N_i) = 1 表明里面只剩下当前充电桩S的位置节点
    # 就退出while
    
    # 打印生成的N辆电单车的坐标
    # for i in range(0, N + 1):
    #     print "坐标为：", (N_x_new[0][i], N_y_new[0][i])
    alpha_temp = []
    for i in range(1 , len(N_i)):
        alpha_temp.append(alpha[0][i])
    time.sleep(1.5)
    end_time = time.time()
    t_sum = (end_time - start_time) 
    # 所有回路的结果
    R_list_result = []
    # 当前S的所有回路的结果，包括回路数量
    R_list_result_temp = []
    # 当前S的所有回路的结果，不包括数量
    R_list_result_tour_list = []
    # 充电子回路数量初始化为
    Tour_num = 1
    # 重新创建服务站 S
    S_new_flag = False
    # 备份N_distance_flag的标志
    backup_N_distance_flag = True
    # 计算系统运行所需时间
    Time_Sum = 0.0

    while 1 != len(N_i) and Time_Over_Flag is True:
        R_list = []
        R_list.append(N_i[0])
        x = []
        y = []
        x.append(N_x_new[0][0])
        y.append(N_y_new[0][0])
        print "创建新的充电子回路"
        print "N_i =", N_i
        first_while_num = 1
        Node_num = len(N_i)
        
        while first_while_num != Node_num and Time_Over_Flag is True:
            print "已经操作到N_i中第", first_while_num, "个数"
            print "N_i中之前共有", Node_num,"个数"
            t = t_sum
            print "电单车运行的时间为：t =", t, "s"
            
            # 更新节点剩余的能量
            # 更新电单车的坐标，毕竟运行了好久，早已经离开了之前的初始的位置
            # 后期优化的时候我们应该考虑边界问题，如果当前的位置已经超过边界，则应该改变运动方向
            # 不仅仅是考虑边界，还得考虑运动过程中，遇到障碍应该避开
            for i in range(1, len(N_i)):
                # print "原始Es_i =", Es_i
                Es_i[0][N_i[i]] = round((Es_i[0][N_i[i]] - P_i[0][N_i[i]]*t), 2)
                # print "原始修改Es_i =", Es_i
                # 如果当前 节点剩余能量低于540j，则将该节点视为死亡节点，不再操作
                if Es_i[0][N_i[i]] < 540.0:
                    N_i_valuse = N_i[0]
                    N_i[0] = 0
                    # print "原序列N_i =", N_i
                    # print N_i[i],"点已经不能运作，需要从节点序列中删除"
                    N_i.remove(N_i[i])
                    N_i_dead_num = N_i_dead_num + 1
                    N_i[0] = N_i_valuse
                    # print "更新节点序列 N_i", N_i
                    # print "N_i长度len(N_i) =", len(N_i)
                    # print "此时 i =", i
                    for j in range(1, i):
                        # print "恢复前几个被修改的节点的剩余能量"
                        Es_i[0][N_i[i]] = round((Es_i[0][N_i[i]] + P_i[0][N_i[i]]*t), 2)
                    # print "恢复Es_i =", Es_i
                    break
                # 保证当前节点剩余能量大于540j
                else:
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
                    # 反之，不相等，则中途出现障碍，不能直接到达目的地，需要改变运动方向
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
                            distance_obstacle_result = []
                            distance_obstacle_result.append(distance_obstacle[0])
                            distance_obstacle_result.append(distance_obstacle[1])
                            distance_obstacle_result.append(distance_obstacle[2])
                            distance_obstacle_result.append(distance_obstacle[3])
                            distance_obstacle_result.append(distance_obstacle[4])
    
                            distance_no_obstacle_result = []
                            distance_no_obstacle_result.append(distance_no_obstacle[0])
                            distance_no_obstacle_result.append(distance_no_obstacle[1])
                            distance_no_obstacle_result.append(distance_no_obstacle[2])
                            distance_no_obstacle_result.append(distance_no_obstacle[3])
                            distance_no_obstacle_result.append(distance_no_obstacle[4])
                            # print "distance_obstacle_result =", distance_obstacle_result
                            # print "distance_no_obstacle_result =", distance_no_obstacle_result
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
            # for i in range(0, len(N_i)):
                # print "更新坐标为：", (N_x_new[0][N_i[i]], N_y_new[0][N_i[i]])
            if len(N_i) == 1:
                # print "表明有些节点剩余能量不足540j，已经死亡"
                break
            print "################################\n"
            # print "延时10s，查看结果"
            # time.sleep(10)
            # 创建一个N*N的空降，生成邻接矩阵，构造无向图G(V,E)
            # N_distance[Ni][Nj] = x 表示Ni到Nj（或Nj到Ni）的距离为x
            # 加入S点后，邻接矩阵每次都会更新，之后每次都会减少节点
            print "剩余节点N_i =", N_i
            print "剩余节点N_i_temp =", N_i_temp
            # 邻接矩阵实时都在变化，比较耗时
            # print "修改之前的N_distance =\n", N_distance
            print "更新distance ~"
            N_distance_Road_result = CreateDistanceNewMatrix(Road_information, N_distance, edge_n, N_x_new, N_y_new, len(N_i_temp), N_i_temp,len(N_i), N_i, obstacle_coordinate, obstacles_Num)
            N_distance = N_distance_Road_result[0]
            Road_information = N_distance_Road_result[1]
            # print "Road_information =\n", Road_information 
            # 重新打印邻接矩阵
            # print "修改之后的N_distance =\n", N_distance
            # 重新开始计算时间，为什么从这开始呢？主要是构建邻接矩阵比较花时间，而且期间所花不属于电单车运行时间
            print "################################\n"
            # print "延时10s，构造邻接矩阵"
            # time.sleep(10)
            start_time = time.time()
            
            # time.sleep(1.8)
            # 只考虑当前所剩下的节点，不包括S点（N_i[0]）
            # 准备充电之前都在运行
            
            # 电单车能量到此开始计算减少了多少
            # 好好考虑 这个时间如何处理
            
            # S统计到各个节点的距离
            # 统计电单车剩余的能量
            N_distance_temp = []
            Es_temp = []
            print "R_list =", R_list
            for i in range(1 , len(N_i)):
                N_distance_temp.append(N_distance[R_list[len(R_list) - 1]][N_i[i]])
                Es_temp.append(Es_i[0][N_i[i]])  
            # print "MCV到每辆电单车的距离 =", N_distance_temp
            # print "结束时Es_i =", Es_temp 
            
            # a 为剩余能量所占比重
            # b 为S到各电单车距离所占比重
            # P 为综合考虑剩余能量和S到各电单车距离所占比重考虑的最终值
            print "参与排序的剩余能量的节点有：", N_i
            Es_sort = []
            N_distance_sort = []
            for i in range(0, len(N_i)):
                # 只获取节点神域能量
                Node_number_Es = []
                Node_number_Es.append(N_i[i])
                Node_number_Es.append(round(Es_i[0][N_i[i]],2))
                Es_sort.append(Node_number_Es)
           
                Node_number_distance= []
                Node_number_distance.append(N_i[i])
                Node_number_distance.append(round(N_distance[R_list[len(R_list) - 1]][N_i[i]], 2))
                N_distance_sort.append(Node_number_distance)
            
            # print "Es_i[0] =", Es_i[0]
            # print "N_distance =", N_distance[0]
            # print "Es_sort =", Es_sort
            # print "N_distance_sort =", N_distance_sort
            select_sort_Numsort(Es_sort)
            select_sort_Numsort(N_distance_sort)
            # print "Es_sort =", Es_sort
            # print "N_distance_sort =", N_distance_sort
            
            a = 0.2
            b = 1.0 - a
            # 只考虑了当前一次，所以得看看如何把所有的点考虑进去，每个点都要考虑到
            # 只考虑当前所剩下的节点，不包括S点（N_i[0]）
            # 假设开始时的P非常非常的大
            P = []
            # 这里应该计算剩余所有节点的P值
            for i in range(1 , len(N_i)):
                # Es_sort
                # N_distance_sort
                P_Node_number = []
                P_Node_number.append(N_i[i])
                Es_Rank = 0
                for l in range(1, len(N_i)):
                    if N_i[i] == Es_sort[l][0]:
                        # 当前剩余能量节点的排名
                        Es_Rank = l
                        break
                Distance_Rank = 0
                for m in range(1, len(N_i)):
                    if N_i[i] == N_distance_sort[m][0]:
                        # 当前 节点距离S的排名
                        Distance_Rank = m
                        break
                P_value = a*Es_Rank + b*Distance_Rank
                
                P_Node_number.append(round(P_value, 2))
                P.append(P_Node_number)
    
            # print "获得的P =", P
            select_sort_Numsort_P(P)
            # print "更新的P =", P
            
            # 找到下一个服务点的标志
            Node_Find_flag = False
            # 最优点的选择
            optimal_N_i = 0
            # 是否选择到最后一个Pab最优值的标志
            Pab_final_falg = False
            # 返回当前是否可调度的标志，主要用于设置新建充电子回路的标志
            result_falg = False
            #  新建充电子回路的标志
            New_ChangeTour = False
            
            for k in range(0, len(P)):
                # 表明当前找到的点就是下一个需要服务的点
                if Node_Find_flag is True:
                    # print "已经找到服务节点"
                    # print "寻找下一需要服务的节点"
                    break
                # 如果当前在Pab排序数值中，选到最后一个数值了
                if k == len(P) - 1:
                    # print "选中Pab最优值的最后一个元素"
                    Pab_final_falg = True
                # 判断是否选中N_i中最后一个元素
                N_i_final_flag = False
                # P[k][0]即为当前所有操作的节点
                # 此时选择的节点为 N_i[q]
                # 已经解决选择节点问题
                # P比较巧，里面的值为两个数值一组保存，一组数值包括：节点标号，和对应节点的P值
                # print "节点 P[k][0] =", P[k][0]
                # print "此时选择的N_i[q] =", P[k][0]
                R_list.append(P[k][0])
                # print "R_list =", R_list 
                result = B.judging_whether_scheduled(P_i, Em, qc, qm, nl, R_list, vm, N_distance, T)
                # print "可调度性结果 result =", result
                result_falg = result
                if result is False:
                    # print "添加这个节点不可调度"
                    R_list.remove(P[k][0])
                    # print "R_list =", R_list 
                else:
                    # print "添加这个点后仍然可以调度"
                    # print "当前节点即为可服务节点"
                    Node_Find_flag = True
                    optimal_N_i = P[k][0]
                    # print "节点", optimal_N_i, "剩余能量 Es=", Es_i[0][optimal_N_i]
                    break
                if Pab_final_falg is True and result_falg is False:
                    # print "最有Pab找到最后一个值，节点也查找到最后一个值，又不再可调度"
                    # print "则当前为一个回路，准备下一个充电子回路的创建"
                    New_ChangeTour = True
                    # print "新建充电子回路的标志 New_ChangeToru =", New_ChangeTour
            # print "遍历所有节点之后，观察R_list = ", R_list
            # print "判断R_list的长度len(R_list) =", len(R_list)
            # 如果len(R_list) == 1 说明当前的服务站S根本适合当前的剩余节点使用
            # 必须依附剩余能量最少的节点进行新疆服务站S
            # 如何len（R_ist）> 1 只要有2个值以上说明当前的服务站S可以为它提供服务
                            
            # print " 两层for循环已经操作完毕！！！！"
            # print "原来的N_i =", N_i
            # 表示当前还不需要创建新的服务站S
            if len(R_list) > 1:
                # 表示不需要新建充电子回路
                if New_ChangeTour is False:
                    # if len(R_list) == 2:
                    # 当R_list_result_tour_list中已经包含回路，则每次进入都会更新回路数据
                    if len(R_list_result_temp) != 0:
                        R_list_result_temp[len(R_list_result_temp) - 1] = R_list 
                    else:# 说明里面还没有回路，直接添加回路
                        R_list_result_temp.append(R_list)
                    # print "说明还可以在当前充电子回路中添加符合可调度性决策的点"
                    # z这个位置容易出现问题，先置为0，之后再恢复
                    N_i_first_value = N_i[0]
                    if optimal_N_i != 0:
                        N_i[0] = 0
                    N_i.remove(optimal_N_i)     
                    
                    N_i[0] = N_i_first_value
                    # print "删除", optimal_N_i,  "点后的N_i =", N_i
                    x.append(N_x_new[0][optimal_N_i])
                    y.append(N_y_new[0][optimal_N_i])
                    x_new = N_x_new[0]
                    y_new = N_y_new[0]
                    # 暂时不打印过程图
                    if Process_Graph_flag is True:
                        NodeToOtherNodeLink(R_list, x, y,x_new,y_new, obstacle_coordinate, R_list[0], Road_information)
                    #  选中的点和上一个点的距离，以及为当前点充电的额时间
                    # 最后进R_list的两个点的距离
                    print "R_list =", R_list
                    print "len(R_list) =", len(R_list)
                    
                    D = N_distance[R_list[len(R_list) - 2]][R_list[len(R_list) - 1]]
                    # print "最后两个点的距离为 D =", D
                    # MCV运动用的时间
                    t1 = D/vm
                    # 消耗的能量
                    last_Es = Es_i_value - Es_i[0][optimal_N_i]
                    # 充电需要的时间
                    t2 = last_Es/qc
                    Change_Time[0][optimal_N_i] = round(t2, 2)
                    print "每辆车的充电市场Change_Time =", Change_Time[0]
                    print "MCV移动消耗时间 t1 =", t1, 's'
                    print "MCV充电消耗时间 t2 =", t2, 's'
                    # 最后一个点到服务站的距离
                    D1 = N_distance[R_list[0]][R_list[len(R_list) - 1]]
                    # 最后一个点带服务站的时间
                    t3 = D1/vm
                    
                    t_sum = t1+t2
                    
                    print "MCV 总共消耗时间 t_sum =", t_sum, 's'
                    first_while_num = first_while_num + 1 
                    # Time_Sum = 0.0
                    # 倒数第二个会重复加了时间，得减掉
                    t4 = 0.0
                    # 回路节点冲过3个时会多计算了一段距离的时间，得减去
                    if len(R_list) >= 3:
                        D2 = N_distance[R_list[0]][R_list[len(R_list) - 2]]
                        t4 = D2/vm
                        
                    Time_Sum = round((Time_Sum + t1 + t3 - t4 + t2), 2)
                    # print "系统运行时间为：Time_Sum =", Time_Sum, "s"
                    # 系统时间结束
                    if Time_Sum > T:
                        print "系统停止运行\n"
                        print "Time_Sum =", Time_Sum, 's'
                        # for k in range(1, len(N_i)):
                            # 如果整个周期内，小车得不到电量补充，也会死亡
                            # N_i_dead_num = N_i_dead_num + 1
                        Time_Over_Flag = False
                    
                else:
                    D = N_distance[R_list[0]][R_list[len(R_list) - 1]]
                    # print "从当前点回到S的距离 D =", D
                    t1 = D/vm
                    print "MCV移动消耗的时间为 t1 =", t1, 's'
                    t_sum = t1
                    # print "准备新建一个充电子回路"
                    # 将同一个服务站的S的充电子回路添加到一个列表中
                    if len(R_list_result_temp) != 0:
                        R_list_result_temp[len(R_list_result_temp) - 1] = R_list
                    # 上一个充电子回路已经确定，接下来新建一个回路（初始化）
                    R_list_result_temp.append(0)
                    # Tour_num = Tour_num + 1
                    break
            # 表示当前的服务站S不能再为剩余的节点提供服务
            # 要重新创建服务站S
            else:
                # 添加上一个服务站S的回路总和
                # print "R_list_result_temp =", R_list_result_temp
                R_list_result.append(R_list_result_temp)
                # print "之前那个回路的情况R_list_result=\n", R_list_result
                # 重新创建服务站S,服务站S的充电子回路重新创建
                R_list_result_tour_list = []
                R_list_result_temp = []
                # 重新创建服务站S后，充电子回路计数重新开始
                Tour_num = 1
                # print "表示当前服务站S不能为剩余的节点提供服务"
                # print "需要重新依附剩余能量最少的节点进行重新创建服务站S"
                # print "重新创建S，它会依附在最优的节点附近，运动几乎不花时间"
                t_sum = 1.5
                # print "原来的N_i =", N_i
                # 这是剩下的界定，我们需要找到节点中剩余能量最少的，
                # 创建服务站S时，将S依附在它附近
                Min = 840000
                for i in range(1, len(N_i)):
                    if Min > Es_i[0][N_i[i]]:
                        Min = Es_i[0][N_i[i]]
                        S_position_Num = N_i[i]
                # print "当前选择节点S_position_Num =", S_position_Num, "为新建服务站依附的节点"
                N_i[0] = S_position_Num
                # print "更新的N_i =", N_i
                # S 的坐标比较讲究，直接放在功耗最大的节点旁边
                S = []
                S_x = N_x_new[0][S_position_Num]
                S_y = N_y_new[0][S_position_Num]
                S.append(S_x)
                S.append(S_y)
                print "充电桩坐标为：", (S[0], S[1])
                # 表示充电桩S的坐标，加入了N中
                # 更新S的坐标
                N_x_new[0][0] = S[0]
                N_y_new[0][0] = S[1]
                break
            
    print "\n"
    print "已经执行完后，补充添相关的数据"
    # 添加最后一个回路总和
    # print "R_list_result_temp", R_list_result_temp
    R_list_result.append(R_list_result_temp)
    # print "所有回路的情况R_list_result=\n", R_list_result
            
    N_x_final = N_x_new
    N_y_final = N_y_new
    # print "N_x_final =", N_x_final
    # print "N_y_final =", N_y_final
    
    result = ChildrenTourConstruction(N_x_final, N_y_final, obstacle_coordinate, R_list_result, S_Flag)
    # 回路集合保存路径

    f1 = open(MCV_Tour_txt, 'w')
    f1.write(str(result[0]))
    f1.close()
    
    # 回路消耗等相关数据的保存路径
    np.savetxt(result_txt, result[1], fmt='%d')
    
    system_time = Time_Sum
    print "回路集合结果 result[0] =\n", result[0]
    print "回路消耗的相关结果 result[1] =\n", result[1]
    print "\n系统运行时间 system_time =", system_time, 's'
    print "\n死亡节点的个数为 N_i_dead_num =", N_i_dead_num
    programing_end_time = time.time()
    print "程序运行总共耗时：", (programing_end_time - programing_start_time), "s"
    ProgrammingTime = (programing_end_time - programing_start_time)
    SystemTime_NodeDeadNum_ProgrammingTime_txt = os.path.join(childern_result_name, 'SystemTime_NodeDeadNum_ProgrammingTime.txt')
    # 回路消耗等相关数据的保存路径
    SystemTime_NodeDeadNum_ProgrammingTime_list = []
    SystemTime_NodeDeadNum_ProgrammingTime_list.append(system_time)
    SystemTime_NodeDeadNum_ProgrammingTime_list.append(N_i_dead_num)
    SystemTime_NodeDeadNum_ProgrammingTime_list.append(ProgrammingTime)
    
    np.savetxt(SystemTime_NodeDeadNum_ProgrammingTime_txt, SystemTime_NodeDeadNum_ProgrammingTime_list, fmt='%d')