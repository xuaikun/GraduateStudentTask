# encoding: utf-8
# 本程序主要实现，充电回路的构造

# 算法功能：充电回路的构造
# 算法输入：电单车在二维空间中的坐标以及功率
# 算法输出： 充电回路子回路结果
import os
import time
import math
import random
import numpy as np
import TourFunction as F
import A_Star_Algorithm as A
import matplotlib.pyplot as plt
import JudgingWhetherScheduled as B
from matplotlib.ticker import MultipleLocator
# 调试程序的标志，DebugFlag = True 为调试
# DebugFlag = False 不调试
DebugFlag = False
# 第一种出发机制对比实验中，固定缓冲池的大小
RequestThreshold = 4

# 使用备份数据时，UseBackupDataFlag = True
# 不使用备份数据时，UseBackupDataFlag = False
# 使用备份数据的条件，预先已经产生相关数据文件，否则不能使用备份数据
UseBackupDataFlag = True
# 数据结果保存路径 (需要使用自己电脑的路径),自己建立一个result文件
result_path = "E:\\00000000000graduate-study\\GraduateStudentTask\\201810_201909_BeforeGraduateStudent\\python_programming\\Ondemand_Charge\\result"

# 以下为数据初始化
# 节点数目 从 50 到 200 变化，将100节点的实验先 做全# 假设我有N辆电单车  会影响程序运行的时间
NodeNum = 20

# 选择插入算法角度阈值设定
# cos90 = 0
# cos180 = -1
# cos90 > cosr > cos180
# 90 < r < 180
# 目前设定
MaxAngle = -1  # 对应最大角度为180°
MinAngle = 0   # 对应最小角度为90° 

# 修改MCV的变量 每次 只修改一个
# 1J=1s*1W
# 1KJ=1000J
# Em  数值从150kj 到400kj变化，间隔50kj变化# MC的总能量 j
# Em = 150000.0   
Em = 150000.0
# 测试过程中从4m/s 到10m/s 变化# MC的移动速度 m/s
Vm = 4.0  
# Mc移动功耗为 J/m
Qm = 55.0  
# MCV充电传输速率 W
Qc = 40.0  
# 阈值上限当电车剩余能量小于1000.0j时，电车将发送request给MCV
Et = 500
# 假设阈值下限为0j
El =0
# 每个节点的能量为5600j
NodeEsValue = 900
# 假设定义的二维空间范围是 edge_n * edge_n 影响构造充电子回路# 电单车运动的空间边长 单位为m
EdgeLength = 1000
# 障碍个数
ObstaclesNum = 20  
# 转向变化的度数
AlphaValue = 30 
# 类似于效率一样，占比多少 n = 0.5
nl = 0.5  
 # 充电周期需要知道7200s
T = 7200.0
# 初始化电单车的运行速度为 3m/s 
V = 3.0   
# 障碍边长 ObstacleLength
ObstacleLength = 10
# 坐标刻度
CoordinateScale = 50

# EL非零处理的临界值
ElThresholdValue = 200

# El操作的时候的步长
Elsteplight = 20

# 求w时需要的精确度
WAccuracy = 0.01

# 坐标刻度
kedu = 40

# 电单车随机游走的时间为100s
T_sum = 100

# 定义并随机生成每个节点功率
NodeP = np.empty([1, NodeNum + 1], float)
# 每个节点的能量
NodeEs = np.empty([1, NodeNum + 1], float)
# 每辆电单车的所要走的方向
Alpha = np.empty([1, NodeNum + 1], float)
# 电单车节点运动时间
NodeMoveTime = np.zeros((1, NodeNum + 1), dtype = np.float)
# 用于判断某个点是否向服务站S发出过Request请求
NodeRequest = np.zeros((1, NodeNum + 1), dtype = np.int)
# 用于判断某个点是否向服务站S发出过ALERT
NodeALERT = np.zeros((1, NodeNum + 1), dtype = np.int)
# 电单车充电频率
NodeChangeFrequency = np.empty([1, NodeNum + 1], float)
# 初始化电单车在二维空间中的坐标
NodeXCoordinate = np.empty([1, NodeNum + 1], float)
NodeYCoordinate = np.empty([1, NodeNum + 1], float)
# 电单车在二维空间中的坐标的中间值
NodeXCoordinateNew = np.empty([1, NodeNum + 1], float)
NodeYCoordinateNew = np.empty([1, NodeNum + 1], float)
# 初始化邻接矩阵
N_distance = np.zeros([NodeNum + 1, NodeNum + 1], dtype = np.float)
# 两个节点之间的路径信息,用list来保存,即：从起点到终点过程中记录走过的位置
Road_information = np.empty([NodeNum + 1, NodeNum + 1], list)

# 障碍坐标范围
ObstacleXDown = np.empty([1, ObstaclesNum], int)
ObstacleXUp   = np.empty([1, ObstaclesNum], int)
ObstacleYDown = np.empty([1, ObstaclesNum], int)
ObstacleYUp   = np.empty([1, ObstaclesNum], int)

# 主要用于画图中进行操作，线条的颜色
LineColor =['b', 'g', 'r', 'c', 'm', 'y', 'k']
# 线条的风格
LineStyle = ['-', '--', '-.', ':']
# 线条的标志
LineLogo = ['.', 'o', 'v', '^', '>', '<', '1', '2', '3', '4', 's', 'p', '*']

# 判断路径是否存在
isExist = os.path.exists(result_path)
if not isExist:
    print "不存在该路径，创建对应路径"
    os.makedirs(result_path)
# 自主修改数据结果子的根目录
result_name = 'Node_' + str(int(NodeNum))
result_name = os.path.join(result_path, result_name)
isExist = os.path.exists(result_name)
if not isExist:
    print "不存在该路径，创建对应路径"
    os.makedirs(result_name)
# 自主修改数据结果子的子目录
childern_result_name = 'Em_' + str(int(Em)) + '_vm_' +str(int(Vm)) + '_qm_' + str(int(Qm)) + '_qc_' + str(int(Qc))
childern_result_name = os.path.join(result_name, childern_result_name)
isExist = os.path.exists(childern_result_name)
if not isExist:
    print "不存在该路径，创建对应路径"
    os.makedirs(childern_result_name)
    
First_path = os.path.join(childern_result_name, "First")
isExist = os.path.exists(First_path)
if not isExist:
    print "不存在该路径，创建对应路径"
    os.makedirs(First_path)
    
Second_path = os.path.join(childern_result_name, "Second")
isExist = os.path.exists(Second_path)
if not isExist:
    print "不存在该路径，创建对应路径"
    os.makedirs(Second_path)
# 创建txt文件
Obstacle_information_data_txt = os.path.join(result_name, 'Obstacle_information_data.txt')
Node_information_data_txt = os.path.join(result_name, 'Node_information_data.txt')

# 第一种出发机制的生成的一些txt文档   
CompareFirst_MCV_Tour_Set_txt = os.path.join(First_path, 'CompareFirst_MCV_Tour_Set.txt')
CompareFirst_MCV_Tour_Information_txt = os.path.join(First_path, 'CompareFirst_MCV_Tour_Information.txt')
CompareFirstDeadNodeNum_data_txt  = os.path.join(First_path, 'CompareFirstDeadNodeNum_data.txt')
 

# 第二种出发机制的生成的一些txt文档   
SecondMCV_Tour_Set_txt = os.path.join(Second_path, 'SecondMCV_Tour_Set.txt')
CompareSecond_MCV_Tour_Set_txt = os.path.join(Second_path, 'CompareSecond_MCV_Tour_Set.txt')

SecondMCV_Tour_Information_txt = os.path.join(Second_path, 'SecondMCV_Tour_Information.txt')
CompareSecond_MCV_Tour_Information_txt = os.path.join(Second_path, 'CompareSecond_MCV_Tour_Information.txt')

AECR_Txt = os.path.join(Second_path, 'SecondAECRData.txt')
AFP_Txt = os.path.join(Second_path, 'SecondAFPData.txt')

El_wBest_Txt = os.path.join(Second_path, 'SecondEl_wBestData.txt')

SecondDeadNodeNum_txt = os.path.join(Second_path, 'SecondDeadNodeNum.txt')
CompareSecondDeadNodeNum_txt = os.path.join(Second_path, 'CompareSecondDeadNodeNum.txt')

#  使用备份数据
def UseBackupData():
    ObstacleCoordinate = np.loadtxt(Obstacle_information_data_txt, dtype = np.int)
         
    ObstacleXDown[0] = ObstacleCoordinate[0]
    ObstacleXUp[0]   = ObstacleCoordinate[1]
    ObstacleYDown[0] = ObstacleCoordinate[2]
    ObstacleYUp[0]   = ObstacleCoordinate[3]
    # print "obstacle_coordinate =", obstacle_coordinate
    # print "len(obstacle_coordinate) =", len(obstacle_coordinate )
    
    # 导入节点相关数据
    data = np.loadtxt(Node_information_data_txt)
    # print "data =", data
    # print "len(data) =", len(data)
    NodeP[0] = data[0]
    # print "P_i[0] =", P_i[0] 
    NodeEs[0] = data[1]
    # print "Es_i[0] =", Es_i[0]
    Alpha[0] = data[2]
    # print "alpha[0] =", alpha[0]
    NodeXCoordinate[0] = data[3]
    # print "N_x[0] =", N_x[0]
    NodeYCoordinate[0] = data[4]
    # print "N_y[0] =", N_y[0]
    NodeChangeFrequency[0] = data[5]
    return ObstacleCoordinate

# 添加充电回路相关信息
def TourConstructionInformation(R_Sum, NodeEs, DeadNodeNum_data_txt, N_distance, NodeXCoordinateNew, NodeYCoordinateNew, MCV_Tour_Set_txt, MCV_Tour_Information_txt, ObstacleCoordinate, FlagNum):
    print "充电回路汇总"
    print "R_Sum =", R_Sum
    if DebugFlag is True:
        print "len(R_Sum) =", len(R_Sum)
        print "节点剩余能量 NodeEs[0] =", NodeEs[0]
    # 总结死亡节点数量，通过剩余能量来判断
    print "NodeEs[0] =", NodeEs[0]
    
    # 用输入的节点的数量进行统计
    DeadNodeNum = 0
    for i in range(1, NodeNum):
        # 一个节点剩余能量低于0时，表明该节点已经死亡
        if NodeEs[0][i] <= 0:
            DeadNodeNum = DeadNodeNum + 1
    print "死亡节点个数DeadNodeNum =", DeadNodeNum
    f1 = open(DeadNodeNum_data_txt, 'w')
    f1.write(str(DeadNodeNum))
    f1.close()

    # 充电回路标号
    MCV_Num = 0
    # 充电回路的详细信息集合
    DataStore_list_sum = []
    
    for i in range(0, len(R_Sum)):
        DataStore_list = []
        print "R_Sum[i] =\n", R_Sum[i]
        ChargingTour = R_Sum[i]
        print "len(ChargingTour) =", len(ChargingTour)
        # 单充电子回路充电能量总和初始化
        ChargingTour_Es = 0.0
        # 单充电子回路充电时间总和初始化
        ChargingTour_Time = 0.0
        for j in range(1, len(ChargingTour)):
            Es = NodeEsValue - NodeEs[0][ChargingTour[j]]
            # MCV在整个充电子回路中充电所需能量
            ChargingTour_Es = ChargingTour_Es +  Es
            # MCV在整个充电子回路中充电消耗的时间
            ChargingTour_Time = ChargingTour_Time + Es/Qc 
        # 实际距离初始化
        D = 0.0
        # 欧几里得距离初始化
        Euclid_D = 0.0
        # 计算一个充电回路的距离
        for j in range(0, (len(ChargingTour) - 1)):
            # 实际距离
            D = D + N_distance[ChargingTour[j]][ChargingTour[j + 1]]
            # print "N_distance[", ChargingTour[j], "][", ChargingTour[j + 1], "] =", N_distance[ChargingTour[j]][ChargingTour[j + 1]]
            # 欧几里得距离
            Euclid_D = Euclid_D + np.sqrt(np.power((NodeXCoordinateNew[0][ChargingTour[j]] - NodeXCoordinateNew[0][ChargingTour[j + 1]]), 2) +np.power((
                    NodeYCoordinateNew[0][ChargingTour[j]] - NodeYCoordinateNew[0][ChargingTour[j + 1]]), 2))
        # 将最后一个节点连接会对应的服务站S 
        D = D + N_distance[ChargingTour[len(ChargingTour) - 1]][ChargingTour[0]]
        # print "N_distance[", ChargingTour[len(ChargingTour) - 1], "][", ChargingTour[0], "] =", N_distance[len(ChargingTour) - 1][ChargingTour[0]]
        Euclid_D = Euclid_D + np.sqrt(np.power((NodeXCoordinateNew[0][ChargingTour[len(ChargingTour) - 1]] - NodeXCoordinateNew[0][ChargingTour[0]]), 2) +np.power((
                    NodeYCoordinateNew[0][ChargingTour[len(ChargingTour) - 1]] - NodeYCoordinateNew[0][ChargingTour[0]]), 2))
        # 求MCV走完整个回路需要多少时间
        Move_time = D/Vm
        # 走完整个回路所需能量（移动耗能）
        Move_Es = Move_time*Qm

        # 添加充电回路的名字
        DataStore_list.append(MCV_Num)
        # 添加MCV给这个回路充电的电量
        DataStore_list.append(round(ChargingTour_Es, 2))
        # 添加MCV在这个回路中的移动能耗
        DataStore_list.append(round(Move_Es, 2))
        # 添加MCV的总能量
        DataStore_list.append(Em)
        # 添加MCV给这个回路充电的时间
        DataStore_list.append(round(ChargingTour_Time, 2))
        # 添加MCV在这个回路中移动所花时间
        DataStore_list.append(round(Move_time, 2))
        # 添加周期时间
        DataStore_list.append(T)
        # 添加充电回路的吞吐量
        DataStore_list.append(len(ChargingTour) - 1)
        # 添加回路的路径长度（实际）
        DataStore_list.append(round(D, 2))
        # 添加回路路径长度（欧几里得路径）
        DataStore_list.append(round(Euclid_D, 2))
        # 执行完一个充电回路，回路名字加1
        MCV_Num = MCV_Num + 1
        if DebugFlag is True:
            print "DataStore_list =\n", DataStore_list
        DataStore_list_sum.append(DataStore_list)
    print "DataStore_list_sum =\n", DataStore_list_sum   
    f1 = open(MCV_Tour_Set_txt, 'w')
    f1.write(str(R_Sum))
    f1.close()
    # 回路消耗等相关数据的保存路径
    np.savetxt(MCV_Tour_Information_txt, DataStore_list_sum, fmt='%0.2f')
    ChildrenTourConstruction(NodeXCoordinateNew, NodeYCoordinateNew, ObstacleCoordinate, R_Sum, 'S', NodeNum, FlagNum)
    return
    

# 将子回路首尾连接起来
def ChildrenTourConstruction(x_new, y_new, obstacle_coordinate_new, R_result, S_Flag_new, N, GraphName):
    print "模拟轨迹##########\n"
    Style_num = 0
    Color_num = 0
    result = R_result
    # print "result = ", result
    # print "len(result) = ", len(result)
    MC_Num = 0
    # 图例显示的标志
    plt.figure('Scatter fig')
    plt.title('S & Node')
    ax = plt.gca()
    Sposition = result[0]

    x_new[0][0] = x_new[0][Sposition[0]]
    y_new[0][0] = y_new[0][Sposition[0]]
    # 每个节点用红圈圈表示出来
    ax.scatter(x_new, y_new,color = 'r',label = 'Node', marker = 'o')
    # 给每个节点标号MC_Num,N表示所有的节点都要标号
    
    for k in range(1, N + 1):
        ax.text(x_new[0][k], y_new[0][k], k, fontsize = 10)
    legend_flag = True
    
    R_list = result
    # print "R_list =", R_list
    # print "len(R_list) = ", len(R_list)
    
    z = len(R_list)
    # print "z =", z
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
            # print "x_list =", x_list
            # print "y_list =", y_list
            
             # 障碍坐标区域使用绿色表示
            # 障碍横坐标x的范围
            if legend_flag is True:
                x_down_list = obstacle_coordinate_new[0]
                x_up_list  = obstacle_coordinate_new[1]
            
                # 障碍纵坐标y的范围
                y_down_list = obstacle_coordinate_new[2]
                y_up_list = obstacle_coordinate_new[3]
                for j in range(0, ObstaclesNum):
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
            
            CSL_string = LineColor[Color_num]
            CSL_string = CSL_string + LineStyle[Style_num]
            if Color_num < (len(LineColor) - 1):
                Color_num = Color_num + 1 
            if Color_num == (len(LineColor) - 1):
                if Style_num < (len(LineStyle) - 1):
                    Style_num = Style_num + 1
                    Color_num = 0
                else:
                    CSL_string = LineColor[random.randint(0, len(LineColor)-1)]
                    CSL_string = CSL_string + LineStyle[random.randint(0, len(LineStyle)-1)]
            
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
    if GraphName == 0:
        
        origin_path = 'SecondFinal.png'  
        All_path = os.path.join(Second_path, origin_path)
    if GraphName == 1:
        
        origin_path = 'CompareSecondFinal.png'  
        All_path = os.path.join(Second_path, origin_path)
    if GraphName == 2:
        
        origin_path = 'CompareFirstFinal.png'  
        All_path = os.path.join(First_path, origin_path)
    if GraphName == 3:
        
        origin_path = 'FirstFinal.png'  
        All_path = os.path.join(First_path, origin_path)
    plt.savefig(All_path)
    plt.xlim([0 - 1, EdgeLength + 1]) #设置绘图X边界                                                                                                   
    plt.ylim([0 - 1, EdgeLength + 1]) #设置绘图Y边界
    plt.show()
    return 

# 初始化函数
def Init(NodeNum, NodeXCoordinate, NodeYCoordinate, NodeP, NodeEs, ObstacleCoordinate, ObstaclesNum, EdgeLength):
    # 保存节点序号的
    NodeList = []
    # 备份节点标号
    NodeListBackup = []
    for i in range(0, NodeNum + 1):     
        NodeList.append(i)
        NodeListBackup.append(i)
    # 打印节点生成的节点
    print "节点序号 NodeList =", NodeList
    # 将节点坐标进行备份
    NodeXCoordinateNew = NodeXCoordinate
    NodeYCoordinateNew = NodeYCoordinate
    # 打印节点功率
    print "节点功率 NodeP[0]=", NodeP[0]
    print "节点能量 NodeEs[0]=", NodeEs[0]
    # 将功率大的节点作为首先部署服务站S的位置，因为功率是已知的，故不适用剩余能量进行判断
    # 初始化功率最大为 NodePMax = 0
    NodePMax = 0  
    for i in range(1, NodeNum + 1):
        if NodePMax <  NodeP[0][NodeList[i]] :
            # 更新当前最大的功率的点
            NodePMax = NodeP[0][NodeList[i]]
            # 保存功率最大的节点
            NodePMaxNode = NodeList[i]
    print "功率最大的节点是：", NodePMaxNode
    # 把第一个服务站S依附的节点的序号放在NodeList序列中的第一个位置
    NodeList[0] = NodePMaxNode
    NodeListBackup[0] = NodePMaxNode 
    # S&1表示服务站依附在节点1处,用SLabel表示
    SLabel= 'S' + '&' + str(NodePMaxNode)
    # S表示充电桩的位置，它的开始时坐标的确定，主要是，靠近功率最大的点
    SCoordinate = []
    # S 的坐标比较讲究，直接放在功耗最大的节点旁边
    SNodeXCoordinate = NodeXCoordinateNew[0][NodePMaxNode]
    SNodeYCoordinate = NodeYCoordinateNew[0][NodePMaxNode]
    SCoordinate.append(SNodeXCoordinate)
    SCoordinate.append(SNodeYCoordinate)
    print "充电桩坐标为：", (SCoordinate[0], SCoordinate[1])
    # 表示充电桩S的坐标，加入了N中
    NodeXCoordinateNew[0][0] = SCoordinate[0]
    NodeYCoordinateNew[0][0] = SCoordinate[1]
    x = []
    y = []
    # 将每个节点添加如x和y列表中
    for i in range(0, NodeNum + 1):
        x.append(NodeXCoordinate[0][i])
        y.append(NodeYCoordinate[0][i])
    # 将电单车和服务站的分布用图呈现出来
    F.AllNodeShow(x, y, ObstacleCoordinate, ObstaclesNum, CoordinateScale, SLabel, EdgeLength, childern_result_name)
    result = []
    result.append(NodeList)
    result.append(NodeListBackup)
    result.append(NodeXCoordinateNew)
    result.append(NodeYCoordinateNew)
    return result

# 修改坐标信息
def ChangeCoordinate(i, El, NodeList, NodeEs, NodeP, t_sum, NodeEsBackup, NodeMoveTime, NodeXCoordinateNew, NodeYCoordinateNew, V, Alpha, AlphaValue, EdgeLength, ObstacleCoordinate, ObstaclesNum):
    # 修改节点能量
    NodeEs[0][NodeList[i]] = round((NodeEs[0][NodeList[i]] - NodeP[0][NodeList[i]]*t_sum), 2)
    if DebugFlag is True:
        print "预计消耗的能量 NodeP[0][NodeList[i]]*t =", NodeP[0][NodeList[i]]*t_sum, 'j'
        print "能量阈值下限 El =", El, 'j'
    
    # 能量将低于阈值下限
    if NodeEs[0][NodeList[i]] <= El:
        if NodeEsBackup[i] <= El:
            if DebugFlag is True:
                print "case 1"
                print "之前该节点已经失效，不再参与计算"
            # 剩余能量还是上次备份的能量
            NodeEs[0][NodeList[i]] = NodeEsBackup[i]
        else:
            if DebugFlag is True:
                print "case 2"
                print "NodeEs[0][NodeList[i]]  =", NodeEs[0][NodeList[i]]
                print "节点实际消耗的能量为", NodeEsBackup[i] - El, 'j'
            # 修改节点剩余能量,该点变为静态点了
            NodeEs[0][NodeList[i]] = El
            if DebugFlag is True:
                print "节点实际运动的时间为", (NodeEsBackup[i] - El)/NodeP[0][NodeList[i]], 's'
            NodeMoveTime[0][NodeList[i]] = (NodeEsBackup[i] - El)/NodeP[0][NodeList[i]]
    # 能量不会低于阈值下限
    if NodeEs[0][NodeList[i]] > El:
        if DebugFlag is True:
            print "case 3"
            print "NodeEs[0][NodeList[i]]  =", NodeEs[0][NodeList[i]]
            print "节点实际消耗的能量为", NodeP[0][NodeList[i]]*t_sum, 'j'
            print "节点实际运动的时间为", t_sum, 's'
        NodeMoveTime[0][NodeList[i]] = t_sum
    if DebugFlag is True:
        print "NodeMoveTime[0] =", NodeMoveTime[0]
           
    # 修改节点的运动时间，因为当节点能量小于阈值El时，电单车节点变为静态节点，保留阈值El能量
    t = NodeMoveTime[0][NodeList[i]]
    # 考虑到边界问题，先将改变之前的坐标记录下来
    N_x_new_temp = NodeXCoordinateNew[0][NodeList[i]]
    N_y_new_temp = NodeYCoordinateNew[0][NodeList[i]] 
    N_x_new_temp_new = NodeXCoordinateNew[0][NodeList[i]]
    N_y_new_temp_new = NodeYCoordinateNew[0][NodeList[i]]
    # 更新节点坐标
    NodeXCoordinateNew[0][NodeList[i]] = round((NodeXCoordinateNew[0][NodeList[i]] + V*t*math.cos(Alpha[0][NodeList[i]]/180.0*math.pi)), 2)
    NodeYCoordinateNew[0][NodeList[i]] = round((NodeYCoordinateNew[0][NodeList[i]] + V*t*math.sin(Alpha[0][NodeList[i]]/180.0*math.pi)), 2)
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
        while((NodeXCoordinateNew[0][NodeList[i]] < 1 or NodeXCoordinateNew[0][NodeList[i]] > EdgeLength) or (
                NodeYCoordinateNew[0][NodeList[i]] < 1 or NodeYCoordinateNew[0][NodeList[i]] > EdgeLength)): 
            # 更新电单车运动方向 改变方向幅度不能太大                        
            Alpha[0][NodeList[i]] = (Alpha[0][NodeList[i]] + AlphaValue)%360

            # 用上面备份的当前的坐标，重新往重新生成的方向前进
            NodeXCoordinateNew[0][NodeList[i]] = round((N_x_new_temp + V*t*math.cos(Alpha[0][NodeList[i]]/180.0*math.pi)), 2)
            NodeYCoordinateNew[0][NodeList[i]] = round((N_y_new_temp + V*t*math.sin(Alpha[0][NodeList[i]]/180.0*math.pi)), 2)
            
        # 越界处理完成的标志，表示已经不越界
        crossing_flag = True
        # 还得保证当前终点不属于任何一个障碍区域内
        first_coordinate = []
        second_coordinate = []
        first_coordinate.append(N_x_new_temp)
        first_coordinate.append(N_y_new_temp)
        second_coordinate.append(NodeXCoordinateNew[0][NodeList[i]])
        second_coordinate.append(NodeYCoordinateNew[0][NodeList[i]])
        # 有障碍 的二维空间
        distance_obstacle =A.a_star_algorithm(EdgeLength, first_coordinate, second_coordinate, ObstacleCoordinate, ObstaclesNum, True)
       
        # 判断是否需要变向
        change_direction_flag = True
        # 如果终点坐标在障碍区域内，也需要改变运动方向
        if distance_obstacle[0] is False:
            change_direction_flag = False
            # 更新电单车运动方向 改变方向幅度不能太大
            Alpha[0][NodeList[i]] = (Alpha[0][NodeList[i]] + AlphaValue)%360
            # 用上面备份的当前的坐标，重新往重新生成的方向前进
            NodeXCoordinateNew[0][NodeList[i]] = round((N_x_new_temp + V*t*math.cos(Alpha[0][NodeList[i]]/180.0*math.pi)), 2)
            NodeYCoordinateNew[0][NodeList[i]] = round((N_y_new_temp + V*t*math.sin(Alpha[0][NodeList[i]]/180.0*math.pi)), 2)                    
        # 终点不在障碍区域内，则开始计算无障碍二维空间中的亮点之间的距离
        else:
            first_coordinate = []
            second_coordinate = []
            first_coordinate.append(N_x_new_temp_new)
            first_coordinate.append(N_y_new_temp_new)
            second_coordinate.append(NodeXCoordinateNew[0][NodeList[i]])
            second_coordinate.append(NodeYCoordinateNew[0][NodeList[i]])
            # 无障碍二维空间
            distance_no_obstacle =A.a_star_algorithm(EdgeLength, first_coordinate, second_coordinate, ObstacleCoordinate, ObstaclesNum, False)
            
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
        
            # 表明两次同一点同一终点，所得距离不相等，说明行驶过程中遇到障碍，需要变向
            # 终点在障碍区域内或者运动过程中遇到障碍都将改变即将到达的位置的坐标
            if distance_no_obstacle[3] != distance_obstacle[3]:
                change_direction_flag = False
                # 更新电单车运动方向 改变方向幅度不能太大
                Alpha[0][NodeList[i]] = (Alpha[0][NodeList[i]] + AlphaValue)%360
                # 用上面备份的当前的坐标，重新往重新生成的方向前进
                NodeXCoordinateNew[0][NodeList[i]] = round((N_x_new_temp + V*t*math.cos(Alpha[0][NodeList[i]]/180.0*math.pi)), 2)
                NodeYCoordinateNew[0][NodeList[i]] = round((N_y_new_temp + V*t*math.sin(Alpha[0][NodeList[i]]/180.0*math.pi)), 2)
                # print "遇到障碍重新更新坐标为：", (N_x_new[0][N_i[i]], N_y_new[0][N_i[i]])
            if crossing_flag is True and change_direction_flag is True:
                # 起点到终点途中不存在障碍或终点不存在与障碍区域
                # 退出while()
                break
    return

if __name__ == "__main__":
    print "Programming is Begin……"
    ProgrammingStartTime = time.time()
    # 不使用备份数据时，备份数据标志为False
    
    if UseBackupDataFlag is False:
        # 返回障碍区域即：障碍的坐标
        ObstacleCoordinate = F.SetObstacles(ObstaclesNum, EdgeLength, ObstacleLength)
        ObstacleXDown[0] = ObstacleCoordinate[0]
        ObstacleXUp[0]   = ObstacleCoordinate[1]
        ObstacleYDown[0] = ObstacleCoordinate[2]
        ObstacleYUp[0]   = ObstacleCoordinate[3]
        # 为了让服务站S处于中心，设置节点1的功耗最大
        for i in range(1, NodeNum + 1):
            # 单位为 W
            if i == 1:
                NodeP[0][1] =0.80
                NodeChangeFrequency[0][1] = 0.1
            else:
                NodeP[0][i] = round(random.uniform(0.25, 0.8), 2)
                NodeChangeFrequency[0][i] = round(random.uniform(0.1, 0.9), 2)
            # P_i[0][i] = round(random.uniform(150, 180), 2)
            # 每辆点电单车的电量，初始化为NodeEsValue
            NodeEs[0][i] = NodeEsValue
            # 电车运行方向的初始化
            Alpha[0][i] = random.randint(0, 360)
            # 电单车使用频率的倒数
            
        # 随机在二维空间中生成NodeNum辆电单车的坐标
        # 将生成的电单车坐标分别保存到x,y列表中# 保证电单车不在障碍区域内 
        for i in range(1, NodeNum+1):
            # 第一个节点放在二维空间的中间，第一个服务站S就在中间
            if i == 1:
                NodeXCoordinate[0][i] = round(random.uniform(EdgeLength/2 - 50, EdgeLength/2 + 50), 2)
                NodeYCoordinate[0][i] = round(random.uniform(EdgeLength/2 - 50, EdgeLength/2 + 50), 2)
            # 其他坐标可以随意放置
            else:
                NodeXCoordinate[0][i] = round(random.uniform(1, EdgeLength), 2)
                NodeYCoordinate[0][i] = round(random.uniform(1, EdgeLength), 2)
            # j为终止条件，即判断是否比较到最后一个障碍区域
            j = 0
            # 要判断当前生成节点是否存在某一个障碍里面，所以要与每一个障碍区域进行比较
            while(j != ObstaclesNum):
                # j_temp 为一个中间转换值
                j_temp = j
                # 存在障碍的表示，初始化时假设变动后后的坐标处于障碍里面 EqualFlag = True表示生成的节点在障碍区域内
                EqualFlag = True
                # 电单车坐标的设置，扩大到障碍外围至少+1m处
                if((NodeXCoordinate[0][i] > (ObstacleXDown[0][j_temp]-1)) and (NodeXCoordinate[0][i] < (ObstacleXUp[0][j_temp]+1))) and(
                    (NodeYCoordinate[0][i] > (ObstacleYDown[0][j_temp]-1)) and (NodeYCoordinate[0][i] < (ObstacleYUp[0][j_temp]+1))):
                    # 表示有电单车在障碍区域里面了，不符合,更新当前电单车坐标并与障碍区域意义比较
                    # 中间值j_temp每次加1
                    j_temp = j_temp + 1
                    if EqualFlag is True:
                        # 第一个节点放在二维空间的中间，第一个服务站S就在中间
                        if i == 1:
                            NodeXCoordinate[0][i] = round(random.uniform(EdgeLength/2 - 50, EdgeLength/2 + 50), 2)
                            NodeYCoordinate[0][i] = round(random.uniform(EdgeLength/2 - 50, EdgeLength/2 + 50), 2)
                        # 其他坐标可以随意放置
                        else:
                            NodeXCoordinate[0][i] = round(random.uniform(1, EdgeLength), 2)
                            NodeYCoordinate[0][i] = round(random.uniform(1, EdgeLength), 2)
                        j_temp = 0
                        j = 0
                    EqualFlag  = False
                    # 表示已经比较到最后一个障碍区域
                    if j_temp >= ObstaclesNum:
                        j = j_temp 
                        # 退出while（）
                        break
                else:
                    # 表示坐标不在障碍区域内，进入比较下一个坐标
                    j = j + 1
                    # 当前坐标已经比较到看最后一个障碍区域
                    if j >= ObstaclesNum:
                        # 退出while
                        break
        # 以整形保存
        # np.savetxt("Obstacle_information_data.txt",obstacle_coordinate, fmt='%d')
        np.savetxt(Obstacle_information_data_txt, ObstacleCoordinate, fmt='%d')
        # 添加节点相关信息
        data = []
        # 添加节点功率
        data.append(NodeP[0])
        # print "P_i[0] =", P_i[0] 
        # 添加节点能量
        data.append(NodeEs[0])
        # print "Es_i[0] =", Es_i[0]
        # 添加初始运动方向
        data.append(Alpha[0])
        # print "alpha[0] =", alpha[0]
        # 添加节点坐标
        data.append(NodeXCoordinate[0])
        # print "N_x[0] =", N_x[0]
        data.append(NodeYCoordinate[0])
        # print "N_y[0] =", N_y[0]
        data.append(NodeChangeFrequency[0])
        # 保存节点相关数据
        # np.savetxt("Node_information_data.txt", data)
        np.savetxt(Node_information_data_txt, data, fmt='%0.2f')

    # 使用备份数据的标志为真时，使用上次已经备份的数据
    if UseBackupDataFlag is True:
        ObstacleCoordinate = UseBackupData()

    print "第二种出发机制,接收到ALERT信息，MCV即刻出发进行充电"
    
    # 初始化函数
    # 保存节点序号的
    NodeList = []
    # 备份节点标号
    NodeListBackup = []
    result = Init(NodeNum, NodeXCoordinate, NodeYCoordinate, NodeP, NodeEs, ObstacleCoordinate, ObstaclesNum, EdgeLength)
    NodeList = result[0]
    NodeListBackup = result[1]
    NodeXCoordinateNew = result[2]
    NodeYCoordinateNew = result[3]
    
    # 程序开始运行，计时开始
    # start_time = time.time()
    # time.sleep(1.5)
    # end_time = time.time()
    # 充电回路汇总
    R_Sum = []
    # 需要对每个节点进行操作，直到剩余服务站S的标号
    while 1 != len(NodeList):
        print "创建新的充电子回路"
        NodeALERTFlag = False
        # 备份序列的标志
        R_list_Backup_flag = True
        FirstComingFlag = True
        
        print "NodeList =", NodeList
        print "程序正在执行，请等待……"
        # 表示已经操作了节点列表中的几个节点
        OperateMNodeNum = 1
        # 统计当前节点序号列表中的个数，即未进行操作节点的个数
        NodeListNum = len(NodeList)
        # 每次都要对所有节点一一操作    
        while OperateMNodeNum != NodeListNum:
            NodeListNum = len(NodeList)
            # 保证没有准备构建回路的趋势
            if NodeALERTFlag is False:
                if DebugFlag is True:
                    print "随意游走100s"
                t_sum = T_sum
                if DebugFlag is True:
                    print "已经操作到N_i中第", OperateMNodeNum, "个数"
                    print "N_i中之前共有", NodeListNum,"个数"
                t = t_sum
                if DebugFlag is True:
                    print "电单车运行的时间为：t =", t, "s"
                
                # 有个隐含条件，上一次运动一定不会出现停止运动即：节点能量不会低于阈值El_Final发送ALERT给服务站S
                # 备份每个节点的能量,后期操作需要
                # 表明不止剩余一个节点
                # 每次操作前先把能量进行备份，一边后续操作所需
                NodeEsBackup = []
                for i in range(0, len(NodeList)):
                    NodeEsBackup.append(NodeEs[0][NodeList[i]])
                # 每个El下定义静态节点个数
                ElStaticNodeAllList = []
                # 每个El下每个节点平均能量消耗的集合
                ElNodeOperateEsAllList = []

                if len(NodeList) != 1:
                   
                    # 自适应调节El的大小,首先让El取一定范围的值，以获取当前消耗的能量，以及死亡节点个数
                    for El in range(0, Et, Elsteplight):
                        # 对于每个El都需要操作
                        if DebugFlag is True:
                            print "El =", El
                        # 单位时间内针对每一个El统计静态节点的个数
                        StaticNodeNum = 0.0
                        # 单位时间系统运行消耗总能量
                        NodeOperateEs = 0.0
                        # 改变El时，每次能量都需要恢复到最原始的能量大小
                        for i in range(0, len(NodeList)):
                            NodeEs[0][NodeList[i]] = NodeEsBackup[i]
                        ConsumptionSum = 0.0
                        # 每个节点运动t时间后，统计所消耗的能量，以及成为静态节点的个数
                        for i in range(1, len(NodeList)):
                            # 修改节点能量
                            NodeEs[0][NodeList[i]] = round((NodeEs[0][NodeList[i]] - NodeP[0][NodeList[i]]*t), 2)
                            ConsumptionSum = ConsumptionSum + NodeP[0][NodeList[i]]*t 
                            if DebugFlag is True:
                                print "预计消耗的能量 NodeP[0][NodeList[i]]*t =", NodeP[0][NodeList[i]]*t, 'j'
                                print "能量阈值下限 El =", El, 'j'
                            
                            # 节点剩余能量将低于阈值下限
                            if NodeEs[0][NodeList[i]] <= El:
                                if NodeEsBackup[i] <= El:
                                    if DebugFlag is True:
                                        print "之前该节点已经失效，不再参与计算"
                                    # 单位时间系统运行消耗总能量
                                    NodeOperateEs = NodeOperateEs + 0
                                    # 针对每一个El统计静态节点的个数
                                    StaticNodeNum = StaticNodeNum + 1
                                else:
                                    if DebugFlag is True:
                                        print "NodeEs[0][NodeList[i]]  =", NodeEs[0][NodeList[i]]
                                        print "节点实际消耗的能量为", NodeEsBackup[i] - El, 'j'
                                        print "节点实际运动的时间为", (NodeEsBackup[i] - El)/NodeP[0][NodeList[i]], 's'
                                    # 单位时间系统运行消耗总能量
                                    NodeOperateEs = NodeOperateEs + NodeP[0][NodeList[i]]*t - (El - NodeEs[0][NodeList[i]])
                                    # 针对每一个El统计静态节点的个数
                                    StaticNodeNum = StaticNodeNum + 1 
                            # 能量不会低于阈值下限
                            if NodeEs[0][NodeList[i]] > El:
                                if DebugFlag is True:
                                    print "NodeEs[0][NodeList[i]]  =", NodeEs[0][NodeList[i]]
                                    print "节点实际消耗的能量为", NodeP[0][NodeList[i]]*t, 'j'
                                    print "节点实际运动的时间为", t, 's'
                                # 单位时间系统运行消耗总能量
                                NodeOperateEs = NodeOperateEs + NodeP[0][NodeList[i]]*t
                        # 这是针对一个El的统计结果
                        if DebugFlag is True:
                            print "静态节点个数 StaticNodeNum =", StaticNodeNum, '个' 
                            print "节点运行总能量 NodeOperateEs =", NodeOperateEs, 'j'
                        ElStaticNodeList = []
                        ElNodeOperateEsList = []
                        # 添加（El, 失效率）
                        ElStaticNodeList.append(El)
                        ElStaticNodeList.append(round(StaticNodeNum/(len(NodeList) - 1), 5))
                        # 添加(El, 平均能量)
                        ElNodeOperateEsList.append(El)
                        ElNodeOperateEsList.append(round(NodeOperateEs/ConsumptionSum, 5))
                        # 添加每一组（El, 失效率）
                        ElStaticNodeAllList.append(ElStaticNodeList)
                        # 添加每一组（El, 平均能量消耗率）
                        ElNodeOperateEsAllList.append(ElNodeOperateEsList)
                    if DebugFlag is True:
                        print "AECR =", ElNodeOperateEsAllList 
                        print "AFP =", ElStaticNodeAllList
                    np.savetxt(AECR_Txt, ElNodeOperateEsAllList,fmt='%0.5f')
                    np.savetxt(AFP_Txt, ElStaticNodeAllList, fmt='%0.5f')
                    
                    x1 = []
                    x2 = []
                    for i in range(0, len(ElNodeOperateEsAllList)):
                        x1.append(ElNodeOperateEsAllList[i][0])
                        x2.append(ElStaticNodeAllList[i][0])
                    
                    y1 = []
                    y2 = []
                    for i in range(0, len(ElNodeOperateEsAllList)):
                        y1.append(ElNodeOperateEsAllList[i][1])
                        y2.append(ElStaticNodeAllList[i][1])
                    if DebugFlag is True:
                        print "x1 =", x1
                        print "y1 =", y1
                        print "x2 =", x2
                        print "y2 =", y2
                    f1 = np.polyfit(x1,y1,1)
                    f2 = np.polyfit(x2,y2,1)
                    if DebugFlag is True:
                        print "f1 =", f1
                        print "f2 =", f2
                    A1 = f1[0]
                    B1 = f1[1]
                    A2 = f2[0]
                    B2 = f2[1]
                    if DebugFlag is True:
                        print "A1 =", A1, "B1 =", B1
                        print "A2 =", A2, "B2 =", B2
                    El_Average = (0 + Et - 100)/2
                    p1 = np.poly1d(f1)
                    p2 = np.poly1d(f2)
                    if DebugFlag is True:
                        print "p1 =", p1
                        print "p2 =", p2
                        print "p1(El_Average) =", p1(El_Average)
                        print "p2(El_Average) =", p2(El_Average)
                    
                    
                    ElList = []
                    wList = []
                    w = 1.0
                    wBest = 0.0
                    ElBefore = 0.0
                    # 可以重复很多次的，自己设定就好
                    while wBest == 0.0:
                        ElNew = -((A1*B1+w*A2*B2)/(np.power(A1, 2)+ w*np.power(A2, 2)))
                        wList.append(w)
                        ElList.append(ElNew)
                        # print "ElBefore =", ElBefore 
                        # print "ElNew =", ElNew
                        # print "np.abs(ElBefore - ElNew) =", np.abs(ElBefore - ElNew) 
                        if np.abs(ElBefore - ElNew) < WAccuracy:
                            # 表明前后两个El值非常接近
                            ElFinal = ElNew
                            # 只需要一个最好的就好了
                            wBest = w 
                            # print "wBest =", wBest
                            break
                        # print "wBest =", wBest
                        
                        ElBefore = ElNew
                        # 每迭代一次，w + 0.1
                        w = w + 0.1
                    # print "查看相关数据"
                    # print "wBest =", wBest
                        
                    # time.sleep(5)
                    f = np.polyfit(wList,ElList,2)
                    p = np.poly1d(f)
                    if DebugFlag is True:
                        print "f =", f
                        print "p =", p
                    if DebugFlag is True:
                        print "wBest =", wBest
                        print "阈值上限为 Et =", Et
                        print "在这次运行过程中比较适合的阈值下限 ElFinal =", ElFinal
                        print "p1(ElFinal) =", p1(ElFinal)
                        print "p2(ElFinal) =", p2(ElFinal)
                    # 将El的值用ElFinal代替
                    El = ElFinal
                    # 对El做非零处理
                    if El < 0:
                        # 如果El小于0，则取它绝对值
                        El = np.abs(El)
                        if DebugFlag is True:
                            print "El小于0,修改下限 ElFinal =", El
                    # El不能大于Et
                    if El >= Et:
                        if DebugFlag is True:
                            print "El 大于Et~"
                        El = Et - ElThresholdValue
                        if DebugFlag is True:
                            print "El修改为El =", El
                    Ellist = []
                    Ellist.append(El)
                    Ellist.append(wBest)
                    np.savetxt(El_wBest_Txt, Ellist,fmt='%0.2f')
                   
                # 每次能量都需要恢复
                for i in range(0, len(NodeList)):
                    NodeEs[0][NodeList[i]] = NodeEsBackup[i]
                if DebugFlag is True:
                    print "NodeEs[0] =", NodeEs[0]  
                    # time.sleep(2)
                # 以上部分是为了计算得到能量阈值下限El
                # 延时1s观察数据
                # time.sleep(5)
                # 更新节点剩余的能量  # 不仅仅是考虑边界，还得考虑运动过程中，遇到障碍应该避开
                for i in range(1, len(NodeList)):
                    # 修改节点能量
                    ChangeCoordinate(i, El, NodeList, NodeEs, NodeP, t_sum, NodeEsBackup, NodeMoveTime, NodeXCoordinateNew, NodeYCoordinateNew, V, Alpha, AlphaValue, EdgeLength, ObstacleCoordinate, ObstaclesNum)
                    
                    # 测试数据用的 事后需要将其注释掉
                    # NodeEs[0][1] = El
                    # print "NodeEs[0] =", NodeEs[0]
                    # 当电单车节点剩余的能量小于阈值上限时，节点向服务站S发送充电请求信号Request
                    if NodeEs[0][NodeList[i]] <= Et:
                        if DebugFlag is True:
                            print "发送充电请求Request"
                        NodeRequest[0][NodeList[i]] = 1
                        
                    # 当电单车节点能量等于阈值El的大小，则开始构建子回路，并充充电
                    if NodeEs[0][NodeList[i]] <= El:
                        print "开始构建子回路，并开始为电单车节点进行充电"
                        # 表明某些节点想服务站S发送了ALERT
                        NodeALERT[0][NodeList[i]] = 1
                        R_list_Backup_flag = True
                        NodeALERTFlag = True
                        FirstComingFlag = True
                    if NodeALERTFlag is True and (i == (len(NodeList) - 1)):
                        plot1 = plt.plot(x1, y1, 'b--o',label='AECR')
                        plot2 = plt.plot(x2, y2, 'r-.',label='AFP')
                        plt.xlabel('El')
                        plt.ylabel('y')
                        plt.legend(loc=4) #指定legend的位置右下角
                        plt.grid()
                        plt.title('polyfitting')
                        
                        # 保存生成的图片
                        origin_path =  'SecondAECR&AFP.png'  
                        All_path = os.path.join(Second_path, origin_path)
                        plt.savefig(All_path)
                        
                        plt.show()
                        
                        plt.plot(wList, ElList, 'g', label = 'El')
                        plt.xlabel('w')
                        plt.ylabel('El')
                        plt.legend(loc=2) #指定legend的位置右下角
                        plt.grid()
                        plt.title('polyfitting')
                        # 保存生成的图片
                        origin_path =  'SecondAEl.png'  
                        All_path = os.path.join(Second_path, origin_path)
                        plt.savefig(All_path)
                        plt.show()
                        print "接收到ALERT后，获取各节点之间的距离"
                        N_distance_Road_result = F.CreateDistanceNewMatrix(Road_information, N_distance, EdgeLength, NodeXCoordinateNew, NodeYCoordinateNew, NodeListBackup, NodeList, ObstacleCoordinate, ObstaclesNum)
                        N_distance = N_distance_Road_result[0]
                        Road_information = N_distance_Road_result[1]
                    
                       
            # 就在这个环节，将一部分节点（R_list中的节点）从原NodeList中删掉
            elif NodeALERTFlag is True:
                # if DebugFlag is True:
                print "剩余能量 NodeEs[0] =", NodeEs[0]
                
                # 说明有电单车节点向MCV或服务站S发送了ALERT信息
                print "表明要开始创建回路了，回路的节点在NodeRequest[0][i] 中为1者体现"
                # 将已经发送Request信息的节点进行提取，每一次操作的时候将节点的名字等进行备份，以便后面查询需要
                if R_list_Backup_flag is True:
                    R_list_Backup = []
                   
                    R_list_Backup.append(NodeList[0])
                    for i in range(1, len(NodeList)):
                        # 说明该节点发送了Request的信号给MCV或者服务站S
                        if NodeRequest[0][NodeList[i]] == 1:
                            # 将即将充电的电单车节点提取出来
                            R_list_Backup.append(NodeList[i])
                    if DebugFlag is True:
                        print "R_list_Backup =", R_list_Backup
                    # 表明只有第一次进行节点名字等的备份
                    R_list_Backup_flag = False
                    # 初始化每个充电回路节点的集合
                    R_New = []
                    # 添加服务站S依附的节点的名字
                    R_New.append(R_list_Backup[0])
                
                # 将已经发送Request信息的节点进行提取
                R_list = []
                R_list.append(NodeList[0])
                for i in range(1, len(NodeList)):
                    # 说明该节点发送了Request的信号给MCV或者服务站S
                    if NodeRequest[0][NodeList[i]] == 1:
                        # 将即将充电的电单车节点提取出来
                        R_list.append(NodeList[i])
                # 打印检查发送了request信号的节点的集合
                if DebugFlag is True:
                    print "R_list =", R_list
                # 构造三元组（1/k，d，e）使用频率的倒数1/k，节点与MCV距离d，剩余电量e
                # 其中两个有两个及以上小的优先充电
                # 假设三元组很大
                k = 1.0
                d = 1000000.0
                e  = 5601.0
                # 发送了Request信号的节点按顺序的添加进入充电回路中，若未发送Request的节点暂时不添加，如添加的节点不满足调度条件，则暂时不添加
                if DebugFlag is True:       
                    print "准备选出下一服务最佳点"
                    print "检查R_list_Backup =", R_list_Backup
                
                    for i in range(1, len(R_list_Backup)):
                        print "NodeChangeFrequency[0][", R_list_Backup[i], "] =", NodeChangeFrequency[0][R_list_Backup[i]]
                        print "N_distance[", R_New[len(R_New) - 1], "][", R_list_Backup[i], "] =", N_distance[R_New[len(R_New) - 1]][R_list_Backup[i]] 
                        print "NodeEs[0][", R_list_Backup[i], "] =", NodeEs[0][R_list_Backup[i]]
                for i in range(1, len(R_list_Backup)):
                    if ((NodeChangeFrequency[0][R_list_Backup[i]] < k and N_distance[R_New[len(R_New) - 1]][R_list_Backup[i]] < d)) or(
                            ((N_distance[R_New[len(R_New) - 1]][R_list_Backup[i]] < d and NodeEs[0][R_list_Backup[i]] < e))) or(
                                    (NodeChangeFrequency[0][R_list_Backup[i]] < k and NodeEs[0][R_list_Backup[i]] < e)): 
                        # 三元组重新赋值
                        k = NodeChangeFrequency[0][R_list_Backup[i]] 
                        d = N_distance[R_New[len(R_New) - 1]][R_list_Backup[i]] 
                        e = NodeEs[0][R_list_Backup[i]]
                        # 寻找最优的节点
                        BestNode = R_list_Backup[i]
                # 本次寻找最好的节点为 BestNode
                # 该点不再运动
                # 说明被遍历过的节点存在未加入充电回路的，需要创建新的充电回路
                print "BestNode =", BestNode
                # 将最好的节点加入构建的充电回路中
                R_New.append(BestNode)
                # R_list_Backup中的值每次都得进行删除操作，主要是怕不能依次对每个节点进行查询操作
                R_list_Backup_FirstValue = R_list_Backup[0]
                R_list_Backup[0] = 0
                R_list_Backup.remove(BestNode)
                R_list_Backup[0] = R_list_Backup_FirstValue
                if DebugFlag is True:
                    print "R_list_Backup =", R_list_Backup
                # 其它点，当剩余能力未低于El时继续运动
                JudgeResult = B.judging_whether_scheduled(NodeP,Em, Qc, Qm, nl, R_New, Vm, N_distance)
                # 使用调度性条件判断是否满足决策条件
                print "JudgeResult =", JudgeResult
                if JudgeResult is True:
                    # 将最好点加入充电回路后，可以被调度，直接加入充电回路
                    # 从R_list中将最好点删掉，同时将NodeList中最好点删掉
                    # 备份R_list的第一个值，防止服务站节点被删掉
                    R_list_FisrtValuse = R_list[0]
                    R_list[0] = 0
                    R_list.remove(BestNode)
                    R_list[0] = R_list_FisrtValuse
                    # 备份NodeList的第一个值，防止服务站节点被删掉
                    NodeList_FirstValue = NodeList[0]
                    NodeList[0] = 0
                    NodeList.remove(BestNode)
                    NodeList[0] = NodeList_FirstValue
                    # 将被删除的最好点从缓冲池中删除，即：其在缓冲池中的值为零
                    # 对应在Request缓冲池中的值置为零
                    # 对应在ALERT缓冲池中的值也置为零
                    NodeRequest[0][BestNode] = 0
                    NodeALERT[0][BestNode] = 0
                    #  不考虑只存在服务站S与依附的节点的R_New序列
                    NodeALERTFlag_New = False
                    if R_New[len(R_New) - 2] != R_New[len(R_New) - 1]:
                        # A点
                        R_New[len(R_New) - 2]
                        # A点坐标
                        Ax = NodeXCoordinateNew[0][R_New[len(R_New) - 2]]
                        Ay = NodeYCoordinateNew[0][R_New[len(R_New) - 2]]
                        # B点
                        R_New[len(R_New) - 1]
                        # B点坐标
                        Bx = NodeXCoordinateNew[0][R_New[len(R_New) - 1]]
                        By = NodeYCoordinateNew[0][R_New[len(R_New) - 1]]

                        for i in range(1, len(R_list)):
                            if NodeALERT[0][R_list[i]] == 1:
                                # C点
                                R_list[i]
                                # C点坐标
                                Cx = NodeXCoordinateNew[0][R_list[i]]
                                Cy = NodeYCoordinateNew[0][R_list[i]]
                                dAB = math.sqrt(math.pow((Ax - Bx), 2) + math.pow((Ay - By), 2))
                                dAC = math.sqrt(math.pow((Ax - Cx), 2) + math.pow((Ay - Cy), 2))
                                dBC = math.sqrt(math.pow((Cx - Bx), 2) + math.pow((Cy - By), 2))
                                # 选择插入算法
                                # 异常处理，插入点就在其中一个端点的位置上
                                if dAC == 0 or dBC == 0:
                                    cosr = -1
                                else:
                                    cosr = (math.pow(dAC, 2) + math.pow(dBC, 2) - math.pow(dAB, 2))/(2*dAC*dBC)
                                if DebugFlag is True:
                                    print "cosr =", cosr
                                # 表示夹角从90°到180°
                                if cosr < MinAngle and cosr >= MaxAngle:
                                    # 只插入一个中途节点就好
                                    if DebugFlag is True:
                                        print "进入发送过ALERT信息的序列中~~~~~~~~~"
                                        print "运动途中插入节点为 ", R_list[i]
                                    R_New_FisrtValue = R_New[0]
                                    R_New[0] = 0
                                    R_NewInser = R_New[len(R_New) - 1]
                                    R_New.remove(R_NewInser)
                                    R_New.append(R_list[i])
                                    R_New.append(R_NewInser)
                                    R_New[0] = R_New_FisrtValue

                                    # 备份NodeList的第一个值
                                    NodeList_FirstValue = NodeList[0]
                                    NodeList[0] = 0
                                    NodeList.remove(R_list[i])
                                    NodeList[0] = NodeList_FirstValue
                                    
                                    # 备份NodeList的第一个值
                                    # 因为这个节点的备份有时候会直接将操作过的节点进行删除
                                    # 所以得判断该节点是否存在与界定啊备份序列中
                                    if DebugFlag is True:
                                        print "R_New =", R_New
                                        print "R_list =", R_list
                                        print "NodeList =", NodeList
                                        print "R_list_Backup =", R_list_Backup 
                                        print "R_list[i] =", R_list[i] 
                                    # 检查R_list_Backup中R_list[i]是否已经被删除
                                    deleteFlag = False
                                    for k in range(1, len(R_list_Backup)):
                                        if R_list[i] == R_list_Backup[k]:
                                            deleteFlag = True
                                    if deleteFlag is True:
                                        R_list_Backup_FirstValue = R_list_Backup[0]
                                        R_list_Backup[0] = 0
                                        R_list_Backup.remove(R_list[i])
                                        R_list_Backup[0] = R_list_Backup_FirstValue
                                    if DebugFlag is True:
                                        print "R_New =", R_New
                                        print "R_list =", R_list
                                        print "NodeList =", NodeList
                                        print "R_list_Backup =", R_list_Backup 
                                        print "R_list[i] =", R_list[i] 
                                    # 缓冲池值为零
                                    NodeRequest[0][R_list[i]] = 0
                                    NodeALERT[0][R_list[i]] = 0
                                    
                                    R_list_FisrtValuse = R_list[0]
                                    R_list[0] = 0
                                    R_list.remove(R_list[i])
                                    R_list[0] = R_list_FisrtValuse
                                    # 表明已经插入一个发送了ALERT信息的节点
                                    NodeALERTFlag_New = True
                                    if DebugFlag is True:
                                        print "ALERT信息中插入节点的结果检查"
                                        print "R_New =", R_New
                                        print "R_list =", R_list
                                        print "R_list_Backup =", R_list_Backup
                                        print "NodeList =", NodeList
                                    break
                        # 剩余节点没有发送过ALERT信息
                        if NodeALERTFlag_New is False:
                            
                            for i in range(1, len(R_list)):
                                if NodeRequest[0][R_list[i]] == 1:
                           
                                    # C点
                                    R_list[i]
                                    # C点坐标
                                    Cx = NodeXCoordinateNew[0][R_list[i]]
                                    Cy = NodeYCoordinateNew[0][R_list[i]]
                                    dAB = math.sqrt(math.pow((Ax - Bx), 2) + math.pow((Ay - By), 2))
                                    dAC = math.sqrt(math.pow((Ax - Cx), 2) + math.pow((Ay - Cy), 2))
                                    dBC = math.sqrt(math.pow((Cx - Bx), 2) + math.pow((Cy - By), 2))
                                    # 选择插入算法
                                    # 异常处理，插入点就在其中一个端点的位置上
                                    if dAC == 0 or dBC == 0:
                                        cosr = -1
                                    else:
                                        cosr = (math.pow(dAC, 2) + math.pow(dBC, 2) - math.pow(dAB, 2))/(2*dAC*dBC)
                                    if DebugFlag is True:
                                        print "cosr =", cosr
                                    # 表示夹角从90°到180°
                                    if cosr < MinAngle and cosr >= MaxAngle:
                                        # 直接退出当前if条件
                                        NodeALERTFlag_New = True
                                        # 只插入一个中途节点就好
                                        if DebugFlag is True:
                                            print "进入发送过Request信息的序列中~~~~~~~~~"
                                            print "运动途中插入的节点为 ", R_list[i]
                                        R_New_FisrtValue = R_New[0]
                                        R_New[0] = 0
                                        R_NewInser = R_New[len(R_New) - 1]
                                        R_New.remove(R_NewInser)
                                        R_New.append(R_list[i])
                                        R_New.append(R_NewInser)
                                        R_New[0] = R_New_FisrtValue

                                        # 备份NodeList的第一个值
                                        NodeList_FirstValue = NodeList[0]
                                        NodeList[0] = 0
                                        NodeList.remove(R_list[i])
                                        NodeList[0] = NodeList_FirstValue
                                        
                                        # 备份NodeList的第一个值
                                        # 因为这个节点的备份有时候会直接将操作过的节点进行删除
                                        # 所以得判断该节点是否存在与界定啊备份序列中
                                        if DebugFlag is True:
                                            print "R_New =", R_New
                                            print "R_list =", R_list
                                            print "NodeList =", NodeList
                                            print "R_list_Backup =", R_list_Backup 
                                            print "R_list[i] =", R_list[i] 
                                        # 检查R_list_Backup中R_list[i]是否已经被删除
                                        deleteFlag = False
                                        for k in range(1, len(R_list_Backup)):
                                            if R_list[i] == R_list_Backup[k]:
                                                deleteFlag = True
                                        if deleteFlag is True:
                                            R_list_Backup_FirstValue = R_list_Backup[0]
                                            R_list_Backup[0] = 0
                                            R_list_Backup.remove(R_list[i])
                                            R_list_Backup[0] = R_list_Backup_FirstValue
                                        if DebugFlag is True:
                                            print "R_New =", R_New
                                            print "R_list =", R_list
                                            print "NodeList =", NodeList
                                            print "R_list_Backup =", R_list_Backup 
                                            print "R_list[i] =", R_list[i] 
                                        # 缓冲池值为零
                                        NodeRequest[0][R_list[i]] = 0
                                        NodeALERT[0][R_list[i]] = 0
                                        
                                        R_list_FisrtValuse = R_list[0]
                                        R_list[0] = 0
                                        R_list.remove(R_list[i])
                                        R_list[0] = R_list_FisrtValuse
                                        if DebugFlag is True:
                                            print "ALERT信息中插入节点的结果检查"
                                            print "R_New =", R_New
                                            print "R_list =", R_list
                                            print "R_list_Backup =", R_list_Backup
                                            print "NodeList =", NodeList
                                        break
                    # if DebugFlag is True:
                    print "检查数据~~~~"
                    print "NodeList =", NodeList
                    print "R_list =", R_list
                    print "R_list_Backup =", R_list_Backup
                    
                    D = N_distance[R_New[len(R_New) - 2]][R_New[len(R_New) - 1]]
                    t1 = D/Vm
                     # 消耗的能量
                    last_Es = NodeEsValue - NodeEs[0][R_New[len(R_New) - 1]]
                    # 充电需要的时间
                    t2 = last_Es/Qc
                    # 将R_New[len(R_New) - 1]加入充电子回路需要消耗的总时间为t
                    t_sum = t1 + t2 
                    t = t_sum
                    # if DebugFlag is True:
                    print "将某点加入充电子回路中需要消耗的总时间为t =", t, 's'
                    # 表明不止剩余一个节点
                    # 有个隐含条件，上一次运动一定不会出现停止运动即：节点能量不会低于阈值El_Final发送ALERT给服务站S
                    # 备份每个节点的能量,后期操作需要
                    # 表明不止剩余一个节点
                    # 每次操作前先把能量进行备份，一边后续操作所需
                    if DebugFlag is True:
                        print "El =", El
                        # 每次操作前先把能量进行备份，一边后续操作所需
                        print "NodeEs[0] =", NodeEs[0]
                        print "NodeList =", NodeList
                    NodeEsBackup = []
                    for i in range(0, len(NodeList)):
                        NodeEsBackup.append(NodeEs[0][NodeList[i]])
                    if DebugFlag is True:
                        print "NodeEsBackup =", NodeEsBackup
                    # 更新节点剩余的能量  # 不仅仅是考虑边界，还得考虑运动过程中，遇到障碍应该避开
                    for i in range(1, len(NodeList)):
                        print "修改节点能量中~"
                        ChangeCoordinate(i, El, NodeList, NodeEs, NodeP, t_sum, NodeEsBackup, NodeMoveTime, NodeXCoordinateNew, NodeYCoordinateNew, V, Alpha, AlphaValue, EdgeLength, ObstacleCoordinate, ObstaclesNum)
                        
                        # 测试数据用的 事后需要将其注释掉
                        # NodeEs[0][1] = El
                        # print "NodeEs[0] =", NodeEs[0]
                        # 当电单车节点剩余的能量小于阈值上限时，节点向服务站S发送充电请求信号Request
                        if NodeEs[0][NodeList[i]] <= Et:
                            if DebugFlag is True:
                                print "发送充电请求Request"
                            # 先判断该节点是否已经发送过Request请求
                            if NodeRequest[0][NodeList[i]] == 0:
                                NodeRequest[0][NodeList[i]] = 1
                            
                        # 当电单车节点能量等于阈值El的大小，则开始构建子回路，并充充电
                        if NodeEs[0][NodeList[i]] <= El:
                            if DebugFlag is True:
                                print "当前节点发送了ALERT信息"
                                print "开始构建子回路，并开始为电单车节点进行充电"
                            # 表明某些节点想服务站S发送了ALERT
                            # 说明其之前没有发送过ALERT信息给MCV，现在发了
                            if NodeALERT[0][NodeList[i]] == 0:
                                NodeALERT[0][NodeList[i]] = 1
                                SureR_list_Backup_Flag  = False
                                # 将现在发送ALERT信号的节点，添加到R_list_Back序列中
                                for j in range(0, len(R_list_Backup)):
                                    if NodeList[i] == R_list_Backup[j]:
                                        SureR_list_Backup_Flag = True
                                # 仅当发送ALERT不存在与R_list_Backup中时，才能将其添加到R_list_Backup中
                                if SureR_list_Backup_Flag is False:        
                                    R_list_Backup.append(NodeList[i])
                            NodeALERTFlag = True
                        # 修改最后一个节点的位置，随之改变距离邻接矩阵
                        if i == len(NodeList) - 1:
                            # if DebugFlag is True:
                            print "遍历到最后一个节点，修改节点之间的距离"
                            # 主要是获取各节点之间的距离
                            N_distance_Road_result = F.CreateDistanceNewMatrix(Road_information, N_distance, EdgeLength, NodeXCoordinateNew, NodeYCoordinateNew, NodeListBackup, NodeList, ObstacleCoordinate, ObstaclesNum)
                            N_distance = N_distance_Road_result[0]
                            Road_information = N_distance_Road_result[1] 
                            print "节点距离修改完毕"
                else:
                    R_New_FirstValue = R_New[0]
                    R_New[0] = 0
                    R_New.remove(BestNode)
                    R_New[0] = R_New_FirstValue
                # if DebugFlag is True:
                print "添加完一个点后，检查相关数据"
                print "R_New =", R_New
                print "R_list =", R_list
                print "R_list_Backup =", R_list_Backup
                print "NodeList =", NodeList
                
                if len(R_list_Backup) == 1 and len(R_list) != 1:
                    if DebugFlag is True:
                        print "重新创建充电回路"
                        print "len(R_New) =", len(R_New)
                    # 重新备份
                    # 将所有充电回路进行汇总
                    # 当充电回路中只有服务站S时，不需要添加到充电回路集合中
                    if len(R_New) != 1:
                        R_Sum.append(R_New)
                    R_list_Backup_flag = True
                    # 如果len(R_New) > 2 使用原来的服务站创建新的充电回路
                    # 添加服务站S，并创建新的回路
                    if len(R_New) == 1:
                        New_S = 0
                        for i in range(1, len(R_list)):
                            # 优先选择已经停止的节点
                            if NodeALERT[0][R_list[i]] == 1:
                                # 获取新的服务站S
                                New_S = R_list[i]
                                break
                            if NodeRequest[0][R_list[i]] == 1:
                                 # 获取新的服务站S
                                New_S = R_list[i]
                                break
                        # 更新服务站依附的节点
                        NodeList[0] = New_S
                        
                                
                if len(R_list) == 1:
                    # 统计当前节点序号列表中的个数，即未进行操作节点的个数
                    if DebugFlag is True:
                        print "对发送过Request请求的节点已经操作完毕"
                    NodeListNum = len(NodeList)
                    R_list_Backup_flag = True
                    NodeALERTFlag = False
                    R_Sum.append(R_New)
                    # 发送请求的节点已经全部加入充电回路了,退出while循环，重新获取充电请求
                    break
    # 添加构建回路的相关信息
    TourConstructionInformation(R_Sum, NodeEs, SecondDeadNodeNum_txt, N_distance, NodeXCoordinateNew, NodeYCoordinateNew, SecondMCV_Tour_Set_txt, SecondMCV_Tour_Information_txt, ObstacleCoordinate, 0)
    
    
    '''
    print "第二种出发机制的做对比实验，接收大Request信息时，MCV即刻出发进行充电"

    # 使用备份数据
    ObstacleCoordinate = UseBackupData()
    # 初始化函数
    # 保存节点序号的
    # 初始化函数
    # 保存节点序号的
    NodeList = []
    # 备份节点标号
    NodeListBackup = []
    result = Init(NodeNum, NodeXCoordinate, NodeYCoordinate, NodeP, NodeEs, ObstacleCoordinate, ObstaclesNum, EdgeLength)
    NodeList = result[0]
    NodeListBackup = result[1]
    NodeXCoordinateNew = result[2]
    NodeYCoordinateNew = result[3]
    
    # 程序开始运行，计时开始
    # start_time = time.time()
    # time.sleep(1.5)
    # end_time = time.time()
    # t_sum = (end_time - start_time)
    # 充电回路汇总
    R_Sum = []
    # 充电回路的命名
    MCV_Num = 0
    # 每个充电回路的信息的汇总
    DataStore_list_sum = []
    # 需要对每个节点进行操作，直到剩余服务站S的标号
    while 1 != len(NodeList):
        t_sum = T_sum
        t = t_sum
        if DebugFlag is True:
            print "每次运行花费 t =", t, 's'
        # 每次操作前先把能量进行备份，一边后续操作所需
        NodeEsBackup = []
        for i in range(0, len(NodeList)):
            NodeEsBackup.append(NodeEs[0][NodeList[i]])
        # 更新节点剩余的能量  # 不仅仅是考虑边界，还得考虑运动过程中，遇到障碍应该避开
        # 只取NodeList中第一个节点，直到这个节点发出Request并且被满足可调度性条件
        i = 1
        # 修改节点能量
        ChangeCoordinate(i, Et, NodeList, NodeEs, NodeP, t_sum, NodeEsBackup, NodeMoveTime, NodeXCoordinateNew, NodeYCoordinateNew, V, Alpha, AlphaValue, EdgeLength, ObstacleCoordinate, ObstaclesNum)
        
        # 测试数据用的 事后需要将其注释掉
        # NodeEs[0][1] = El
        # print "NodeEs[0] =", NodeEs[0]
        # 当电单车节点剩余的能量小于阈值上限时，节点向服务站S发送充电请求信号Request
        if NodeEs[0][NodeList[i]] <= Et:
            if DebugFlag is True:
                print "发送充电请求Request"
                print "将该节点添加进入充电回路"
            NodeRequest[0][NodeList[i]] = 1
            
            BestNode = NodeList[i]
            if DebugFlag is True:
                print "剩余能量 NodeEs[0] =", NodeEs[0]
                print "NodeRequest[0] =", NodeRequest[0]
            # 主要是获取各节点之间的距离
            N_distance_Road_result = F.CreateDistanceNewMatrix(Road_information, N_distance, EdgeLength, NodeXCoordinateNew, NodeYCoordinateNew, NodeListBackup, NodeList, ObstacleCoordinate, ObstaclesNum)
            N_distance = N_distance_Road_result[0]
            Road_information = N_distance_Road_result[1]
            
            JudgeResult = False
            while JudgeResult is False:
                R_New = []
                R_New.append(NodeList[0])
                if DebugFlag is True:
                    # 最佳点是要被添加到充电回路中的
                    print "最佳的节点为 ", BestNode
                
                # 将最好的节点加入构建的充电回路中
                R_New.append(BestNode)
                # 接下来要操作最佳节点
                # 其它点，当剩余能量未低于El时继续运动
                JudgeResult = B.judging_whether_scheduled(NodeP,Em, Qc, Qm, nl, R_New, Vm, N_distance)
                # 使用调度性条件判断是否满足决策条件
                print "JudgeResult =", JudgeResult
                if JudgeResult is True:
                    # 用于保存当前回路的相关信息
                    DataStore_list = []
                    # 将最好点加入充电回路后，可以被调度，直接加入充电回路
                    # 备份NodeList的第一个值，防止服务站节点被删掉
                    NodeList_FirstValue = NodeList[0]
                    NodeList[0] = 0
                    NodeList.remove(BestNode)
                    NodeList[0] = NodeList_FirstValue
                    # 将被删除的最好点从缓冲池中删除，即：其在缓冲池中的值为零
                    # 对应在Request缓冲池中的值置为零
                    # 对应在ALERT缓冲池中的值也置为零
                    NodeRequest[0][BestNode] = 0
                    NodeALERT[0][BestNode] = 0
                    if DebugFlag is True:
                        print "检查数据~~~~"
                        print "NodeList =", NodeList
                        print "R_New =", R_New
                    
                    D = N_distance[R_New[len(R_New) - 2]][R_New[len(R_New) - 1]]
                    t1 = D/Vm
                    # 欧几里得距离
                    Euclid_Distance = (np.sqrt(np.power((NodeXCoordinateNew[0][R_New[0]] - NodeXCoordinateNew[0][R_New[1]]), 2) +np.power((
                            NodeYCoordinateNew[0][R_New[0]] - NodeYCoordinateNew[0][R_New[1]]), 2)))*2
                     # 消耗的能量
                    last_Es = NodeEsValue - NodeEs[0][R_New[len(R_New) - 1]]
                    # 充电需要的时间
                    t2 = last_Es/Qc
                    # 将R_New[len(R_New) - 1]加入充电子回路需要消耗的总时间为t
                    t_sum = t1 + t2   
                    t = t_sum
                    # MCV移动耗时
                    MCVMoveTime = D*2/Vm
                    # MCV移动耗能
                    MCVMoveEs = MCVMoveTime*Qm
                    if DebugFlag is True:
                        print "电单车运行的时间为：t =", t, "s"
                        print "NodeListBackup =", NodeListBackup 
                    R_Sum.append(R_New)
                    # 添加充电回路的名字
                    DataStore_list.append(MCV_Num)
                    # 添加MCV给这个回路充电的电量
                    DataStore_list.append(round(last_Es, 2))
                    # 添加MCV在这个回路中的移动能耗
                    DataStore_list.append(round(MCVMoveEs, 2))
                    # 添加MCV的总能量
                    DataStore_list.append(Em)
                    # 添加MCV给这个回路充电的时间
                    DataStore_list.append(round(t2, 2))
                    # 添加MCV在这个回路中移动所花时间
                    DataStore_list.append(round(MCVMoveTime, 2))
                    # 添加周期时间
                    DataStore_list.append(T)
                    # 添加充电回路的吞吐量
                    DataStore_list.append(len(R_New) - 1)
                    # 添加回路的路径长度（实际）
                    DataStore_list.append(round(D*2, 2))
                    # 添加回路路径长度（欧几里得路径）
                    DataStore_list.append(round(Euclid_Distance, 2))
                    MCV_Num = MCV_Num + 1 
                    if DebugFlag is True:
                        print "DataStore_list =", DataStore_list
                    DataStore_list_sum.append(DataStore_list)
                else:
                    if DebugFlag is True:
                        print "不符合调度条件,当前最佳点将被从充电回路中删除"
                    R_New_FirstValue = R_New[0]
                    R_New[0] = 0
                    R_New.remove(BestNode)
                    R_New[0] = R_New_FirstValue
                    # 将充电子回路添加进入充电回路集合
                    # 在以前的服务站下，没有节点可以再被调度，则创建新的服务站
                    # 创建新的服务站S
                    NodeList[0] = BestNode
                    R_New = []
                    R_New.append(NodeList[0])
            if DebugFlag is True:       
                print "R_New =", R_New
                print "NodeList =", NodeList
                print "操作一次Request缓冲池需要的时间为 t =", t, 's'
        if len(NodeList) == 1:
            print "说明NodeList已经删除到剩余服务站节点"
            break
    
    # 总结死亡节点数量，通过剩余能量来判断
    print "NodeEs[0] =", NodeEs[0]
    # 用输入的节点的数量进行统计
    DeadNodeNum = 0
    for i in range(1, NodeNum):
        # 一个节点剩余能量低于0时，表明该节点已经死亡
        if NodeEs[0][i] <= 0:
            DeadNodeNum = DeadNodeNum + 1
    print "死亡节点个数DeadNodeNum =", DeadNodeNum
    f1 = open(CompareSecondDeadNodeNum_txt, 'w')
    f1.write(str(DeadNodeNum))
    f1.close()
    
    print "充电回路汇总"
    print "R_Sum =\n", R_Sum
    f1 = open(CompareSecond_MCV_Tour_Set_txt, 'w')
    f1.write(str(R_Sum))
    f1.close()
    print "DataStore_list_sum =\n", DataStore_list_sum
    # 回路消耗等相关数据的保存路径
    np.savetxt(CompareSecond_MCV_Tour_Information_txt, DataStore_list_sum, fmt='%0.2f')
    ChildrenTourConstruction(NodeXCoordinateNew, NodeYCoordinateNew, ObstacleCoordinate, R_Sum, 'S', NodeNum, 1)
    '''
    
    '''
    print "第一种出发机制的对比实验，固定缓冲池"
    # 使用备份数据
    ObstacleCoordinate = UseBackupData()
    
    # 初始化函数
    # 保存节点序号的
    NodeList = []
    # 备份节点标号
    NodeListBackup = []
    result = Init(NodeNum, NodeXCoordinate, NodeYCoordinate, NodeP, NodeEs, ObstacleCoordinate, ObstaclesNum, EdgeLength)
    NodeList = result[0]
    NodeListBackup = result[1]
    NodeXCoordinateNew = result[2]
    NodeYCoordinateNew = result[3]
    
    # 充电回路的命名
    MCV_Num = 0
    # 每个充电回路的信息的汇总
    DataStore_list_sum = []
    # 程序开始运行，计时开始
    # start_time = time.time()
    # time.sleep(1.5)
    # end_time = time.time()
    # 充电回路汇总
    R_Sum = []
    # 初始化节点发送Request请求的数量
    RequestNum = 0 
    # 将发送Request的节点序列进行保存
    Request_list = []
    # 需要对每个节点进行操作，直到剩余服务站S的标号
    while 1 != len(NodeList):
        print "创建新的充电子回路"
        NodeRequestFlag = False
        # 备份序列的标志
        R_list_Backup_flag = True
        FirstComingFlag = True
        BeginR_list = True

        print "NodeList =", NodeList
        print "程序正在执行，请等待……"
        # 表示已经操作了节点列表中的几个节点
        OperateMNodeNum = 1
        # 统计当前节点序号列表中的个数，即未进行操作节点的个数
        NodeListNum = len(NodeList)
        # 每次都要对所有节点一一操作  
        while OperateMNodeNum != NodeListNum:
            NodeListNum = len(NodeList)
            # 保证没有准备构建回路的趋势
            if NodeRequestFlag is False:
                if DebugFlag is True:
                    print "随意游走100s"
                t_sum = T_sum
                if DebugFlag is True:
                    print "已经操作到N_i中第", OperateMNodeNum, "个数"
                    print "N_i中之前共有", NodeListNum,"个数"
                t = t_sum
                if DebugFlag is True:
                    print "电单车运行的时间为：t =", t, "s"
                
                # 有个隐含条件，上一次运动一定不会出现停止运动即：节点能量不会低于阈值El_Final发送ALERT给服务站S
                # 备份每个节点的能量,后期操作需要
                # 表明不止剩余一个节点
                # 每次操作前先把能量进行备份，一边后续操作所需
                NodeEsBackup = []
                for i in range(0, len(NodeList)):
                    NodeEsBackup.append(NodeEs[0][NodeList[i]])
                # 每次能量都需要恢复
                for i in range(0, len(NodeList)):
                    NodeEs[0][NodeList[i]] = NodeEsBackup[i]
                if DebugFlag is True:
                    print "NodeEs[0] =", NodeEs[0]  
                    # time.sleep(2)
                # 以上部分是为了计算得到能量阈值下限El
                # 延时1s观察数据
                # time.sleep(5)
                # 更新节点剩余的能量  # 不仅仅是考虑边界，还得考虑运动过程中，遇到障碍应该避开
                for i in range(1, len(NodeList)):
                    # 修改节点能量
                    ChangeCoordinate(i, El, NodeList, NodeEs, NodeP, t_sum, NodeEsBackup, NodeMoveTime, NodeXCoordinateNew, NodeYCoordinateNew, V, Alpha, AlphaValue, EdgeLength, ObstacleCoordinate, ObstaclesNum)
                    
                    # 测试数据用的 事后需要将其注释掉
                    # NodeEs[0][1] = El
                    # print "NodeEs[0] =", NodeEs[0]
                    # 当电单车节点剩余的能量小于阈值上限时，节点向服务站S发送充电请求信号Request
                    if NodeEs[0][NodeList[i]] <= Et:
                        if DebugFlag is True:
                            print "发送充电请求Request"
                        if NodeRequest[0][NodeList[i]] == 0:
                            NodeRequest[0][NodeList[i]] = 1 
                            # 将发送了Request的节点进行保存
                            Request_list.append(NodeList[i])

                    if  (len(Request_list) >= RequestThreshold) and (i == len(NodeList) - 1):
                        print "发送的Request数量已经达到上限，可以构建充电回路了"
                        # 充电请求已经达到固定阈值上限，开始创建充电回路
                        NodeRequestFlag = True
                        # 判断是否是第一次进行构造充电回路
                        FirstComingFlag = True 
                        BeginR_list = True
                        # 主要是获取各节点之间的距离
                        N_distance_Road_result = F.CreateDistanceNewMatrix(Road_information, N_distance, EdgeLength, NodeXCoordinateNew, NodeYCoordinateNew, NodeListBackup, NodeList, ObstacleCoordinate, ObstaclesNum)
                        N_distance = N_distance_Road_result[0]
                        Road_information = N_distance_Road_result[1] 
                        
                    # 说明剩余的每个节点都发送了Request请求信息
                    if len(Request_list) >= (len(NodeList) - 1) and (i == len(NodeList) - 1):
                        # 这种情况下，统计的Request请求数量可能达不到到MCV出发的阈值，也照样得出发进行充电
                        NodeRequestFlag = True
                        # 判断是否是第一次进行构造充电回路
                        FirstComingFlag = True 
                        BeginR_list = True
                        # 主要是获取各节点之间的距离
                        N_distance_Road_result = F.CreateDistanceNewMatrix(Road_information, N_distance, EdgeLength, NodeXCoordinateNew, NodeYCoordinateNew, NodeListBackup, NodeList, ObstacleCoordinate, ObstaclesNum)
                        N_distance = N_distance_Road_result[0]
                        Road_information = N_distance_Road_result[1] 
                        

            # 就在这个环节，将一部分节点（R_list中的节点）从原NodeList中删掉
            while NodeRequestFlag is True:
                if DebugFlag is True:
                    print "剩余能量 NodeEs[0] =", NodeEs[0]
                    # 说明有电单车节点向MCV或服务站S发送了ALERT信息
                    print "表明要开始创建回路了，回路的节点在NodeRequest[0][i] 中为1者体现"
                print "发送过Request的节点序列为 Request_list =", Request_list
                print "发送过Request的节点序列长度为len(Request_list) =", len(Request_list)

                if BeginR_list is True:
                    BeginR_list = False
                    if len(Request_list) > RequestThreshold:
                        print "说明接收到的Request数量大于阈值，构建充电回路的节点只要阈值的数量"
                        # 将已经发送Request信息的节点进行提取，每一次操作的时候将节点的名字等进行备份，以便后面查询需要
                        R_list_Backup = []
                        # 将已经发送Request信息的节点进行提取
                        R_list = []
                        R_list.append(NodeList[0])
                        R_list_Backup.append(NodeList[0])
                        deletedata = []
                        NodeList_FirstValue = NodeList[0]
                        NodeList[0] = 0
                        deletedataFlag = False
                        for i in range(0, RequestThreshold):
                            # 说明该节点发送了Request的信号给MCV或者服务站S
                            R_list.append(Request_list[i])
                            R_list_Backup.append(Request_list[i])
                            for j in range(0, len(NodeList)):
                                if NodeList[j] == Request_list[i]:
                                    deletedataFlag = True
                                    deletedata.append(NodeList[j])
                        
                        if deletedataFlag is True:
                            for k in range(0, len(deletedata)):
                                NodeList.remove(deletedata[k])
                        NodeList[0] = NodeList_FirstValue
                        
                        # 初始化每个充电回路节点的集合
                        R_New = []
                        # 添加服务站S依附的节点的名字
                        R_New.append(R_list_Backup[0])
                    else:
                        # 将已经发送Request信息的节点进行提取，每一次操作的时候将节点的名字等进行备份，以便后面查询需要
                        print "说明接收到的Request数量小于等于阈值，构建充电回路的节点可以小于阈值的数量"
                        R_list_Backup = []   
                        R_list = []
                        R_list.append(NodeList[0])
                        R_list_Backup.append(NodeList[0])
                        print "NodeList =", NodeList 
                        deletedata = []
                        deletedateFlag = False
                        NodeList_FirstValue = NodeList[0]
                        NodeList[0] = 0
                        for i in range(0, len(Request_list)):
                            R_list.append(Request_list[i])
                            R_list_Backup.append(Request_list[i])
                            for j in range(0, len(NodeList)):
                                if NodeList[j] == Request_list[i]:
                                    deletedataFlag = True
                                    deletedata.append(NodeList[j])
                        
                        if deletedataFlag is True:
                            
                            for k in range(0, len(deletedata)):
                                NodeList.remove(deletedata[k])
                        NodeList[0] = NodeList_FirstValue
                        
                        print "NodeList =", NodeList
                        
                        # 初始化每个充电回路节点的集合
                        R_New = []
                        # 添加服务站S依附的节点的名字
                        R_New.append(R_list_Backup[0])
                # 打印检查发送了request信号的节点的集合
                if DebugFlag is True:
                    print "R_list =", R_list
                    print "R_list_Backup =", R_list_Backup

                # 选择最佳下一服务点为，R_list_Backup中的第二个节点
                
                BestNode = R_list_Backup[1]
                
                # 本次寻找最好的节点为 BestNode
                # 该点不再运动
                
                print "BestNode =", BestNode
                # 将最好的节点加入构建的充电回路中
                R_New.append(BestNode)
                # R_list_Backup中的值每次都得进行删除操作，主要是怕不能依次对每个节点进行查询操作
                R_list_Backup_FirstValue = R_list_Backup[0]
                R_list_Backup[0] = 0
                R_list_Backup.remove(BestNode)
                R_list_Backup[0] = R_list_Backup_FirstValue
                if DebugFlag is True:
                    print "R_list_Backup =", R_list_Backup
                # 其它点，当剩余能力未低于El时继续运动
                JudgeResult = B.judging_whether_scheduled(NodeP,Em, Qc, Qm, nl, R_New, Vm, N_distance)
                # 使用调度性条件判断是否满足决策条件
                print "JudgeResult =", JudgeResult
                if JudgeResult is True:
                    # 将最好点加入充电回路后，可以被调度，直接加入充电回路
                    # 从R_list中将最好点删掉，同时将NodeList中最好点删掉
                    # 备份R_list的第一个值，防止服务站节点被删掉
                    R_list_FisrtValuse = R_list[0]
                    R_list[0] = 0
                    R_list.remove(BestNode)
                    R_list[0] = R_list_FisrtValuse
                    # 从Request_list中将BestNode删掉
                    Request_list.remove(BestNode)
                    # 将被删除的最好点从缓冲池中删除，即：其在缓冲池中的值为零
                    # 对应在Request缓冲池中的值置为零
                    # 对应在ALERT缓冲池中的值也置为零
                    NodeRequest[0][BestNode] = 0
                    NodeALERT[0][BestNode] = 0
                    #  不考虑只存在服务站S与依附的节点的R_New序列
                    NodeALERTFlag_New = False
                    
                    if DebugFlag is True:
                        print "检查数据~~~~"
                        print "NodeList =", NodeList
                        print "R_list =", R_list
                        print "R_list_Backup =", R_list_Backup
                    
                    D = N_distance[R_New[len(R_New) - 2]][R_New[len(R_New) - 1]]
                    t1 = D/Vm
                     # 消耗的能量
                    last_Es = NodeEsValue - NodeEs[0][R_New[len(R_New) - 1]]
                    # 充电需要的时间
                    t2 = last_Es/Qc
                    # 将R_New[len(R_New) - 1]加入充电子回路需要消耗的总时间为t
                    t_sum = t1 + t2 
                    t = t_sum
                    if DebugFlag is True:
                        print "将某点加入充电子回路中需要消耗的总时间为t =", t, 's'
                    # 表明不止剩余一个节点
                    # 有个隐含条件，上一次运动一定不会出现停止运动即：节点能量不会低于阈值El_Final发送ALERT给服务站S
                    # 备份每个节点的能量,后期操作需要
                    # 表明不止剩余一个节点
                    # 每次操作前先把能量进行备份，一边后续操作所需
                    if DebugFlag is True:
                        print "El =", El
                        # 每次操作前先把能量进行备份，一边后续操作所需
                        print "NodeEs[0] =", NodeEs[0]
                        print "NodeList =", NodeList
                    NodeEsBackup = []
                    for i in range(0, len(NodeList)):
                        NodeEsBackup.append(NodeEs[0][NodeList[i]])
                    if DebugFlag is True:
                        print "NodeEsBackup =", NodeEsBackup
                    # 更新节点剩余的能量  # 不仅仅是考虑边界，还得考虑运动过程中，遇到障碍应该避开
                    for i in range(1, len(NodeList)):
                        # 修改节点能量
                        ChangeCoordinate(i, El, NodeList, NodeEs, NodeP, t_sum, NodeEsBackup, NodeMoveTime, NodeXCoordinateNew, NodeYCoordinateNew, V, Alpha, AlphaValue, EdgeLength, ObstacleCoordinate, ObstaclesNum)
                        
                        # 测试数据用的 事后需要将其注释掉
                        # NodeEs[0][1] = El
                        # print "NodeEs[0] =", NodeEs[0]
                        # 当电单车节点剩余的能量小于阈值上限时，节点向服务站S发送充电请求信号Request
                        if NodeEs[0][NodeList[i]] <= Et:
                            if DebugFlag is True:
                                print "发送充电请求Request"
                            # 先判断该节点是否已经发送过Request请求
                            if NodeRequest[0][NodeList[i]] == 0:
                                NodeRequest[0][NodeList[i]] = 1
                                # 只添加还未发送过Request请求的节点
                                Request_list.append(NodeList[i])
                        # 修改最后一个节点的位置，随之改变距离邻接矩阵
                        if i == len(NodeList) - 1:
                            if DebugFlag is True:
                                print "遍历到最后一个节点，修改节点之间的距离"
                            # 主要是获取各节点之间的距离
                            N_distance_Road_result = F.CreateDistanceNewMatrix(Road_information, N_distance, EdgeLength, NodeXCoordinateNew, NodeYCoordinateNew, NodeListBackup, NodeList, ObstacleCoordinate, ObstaclesNum)
                            N_distance = N_distance_Road_result[0]
                            Road_information = N_distance_Road_result[1] 
                            
                else:
                    R_New_FirstValue = R_New[0]
                    R_New[0] = 0
                    R_New.remove(BestNode)
                    R_New[0] = R_New_FirstValue
                if DebugFlag is True:
                    print "R_New =", R_New
                    print "R_list =", R_list
                    print "R_list_Backup =", R_list_Backup
                    print "NodeList =", NodeList
                    print "Request_list =", Request_list
                
                if len(R_list_Backup) == 1 and len(R_list) != 1:
                    # 重新备份
                    # 将所有充电回路进行汇总
                    # 当充电回路中只有服务站S时，不需要添加到充电回路集合中
                    if len(R_New) != 1:
                        R_Sum.append(R_New)
                    R_list_Backup = []
                    for q in range(0, len(R_list)):
                        R_list_Backup.append(R_list[q])
                    if DebugFlag is True:
                        print "重新创建充电回路"
                        print "len(R_New) =", len(R_New)
                    
                    # 如果len(R_New) > 2 使用原来的服务站创建新的充电回路
                    # 添加服务站S，并创建新的回路
                    if len(R_New) == 1:
                        New_S = 0
                        for i in range(1, len(R_list)):
                            # 优先选择已经停止的节点
                            if NodeALERT[0][R_list[i]] == 1:
                                # 获取新的服务站S
                                New_S = R_list[i]
                                break
                            if NodeRequest[0][R_list[i]] == 1:
                                 # 获取新的服务站S
                                New_S = R_list[i]
                                break
                        # 更新服务站依附的节点
                        NodeList[0] = New_S
                        # 如果服务站S修改，则每个节点序列也得修改服务站
                        R_list[0] = NodeList[0] 
                        R_list_Backup[0] = NodeList[0] 
                    # 重新构建充电回路
                    R_New = []
                    R_New.append(R_list[0])
                    
                # 当前充电回路的节点已经被添加完毕
                if len(R_list) == 1:
                    # 统计当前节点序号列表中的个数，即未进行操作节点的个数
                    if DebugFlag is True:
                        print "对发送过Request请求的节点已经操作完毕"
                    NodeListNum = len(NodeList)
                    R_Sum.append(R_New)
                    # 当前Request_list序列为空
                    if len(Request_list) == 0:
                        print "到目前为止，所有发送过Request信息的节点都已经被操作完了"
                        NodeRequestFlag = False
                        break
                    # 当前Request_list序列不为空
                    if len(Request_list) != 0:
                        if len(Request_list) >= RequestThreshold:
                            print "缓冲池已满，可再次构建回路"
                            print "直接把剩余的一部分节点构成充电回路"
                            BeginR_list = True
                        else:
                            print "当前剩余节点不足以填满缓冲池，需要节点再发送Request信息"
                            if (len(NodeList) - 1) == len(Request_list):
                                print "说明剩余的节点都已经发送了Request信息"
                                BeginR_list = True
                                break
                            else:
                                print "说明还有点未发送Request信息"
                                NodeRequestFlag = False
                                break
    # 添加构建回路的相关信息
    TourConstructionInformation(R_Sum, NodeEs, CompareFirstDeadNodeNum_data_txt, N_distance, NodeXCoordinateNew, NodeYCoordinateNew, CompareFirst_MCV_Tour_Set_txt, CompareFirst_MCV_Tour_Information_txt, ObstacleCoordinate, 2)
    '''
    print "程序即将结束"
    # ProgrammingEndTime = time.time()
    # print "固定缓冲池对比实验最终耗时 ", (ProgrammingEndTime - ProgrammingStartTime), 's'
    ProgrammingEndTime = time.time()    
    print "程序总共耗时 ", (ProgrammingEndTime- ProgrammingStartTime), 's'
          