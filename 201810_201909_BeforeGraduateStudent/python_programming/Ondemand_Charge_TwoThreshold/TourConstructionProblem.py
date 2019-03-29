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
import TourDistribution as TD
import A_Star_Algorithm as A
import matplotlib.pyplot as plt
import JudgingWhetherScheduled as B
from matplotlib.ticker import MultipleLocator

# 调试程序的标志，DebugFlag = True 为调试
# DebugFlag = False 不调试
DebugFlag = False
# 第一种出发机制缓冲池最小值
RequestThresholdMin = 4
# 第一种出发机制缓冲池最大值
RequestThresholdMax = 6
# 第一种出的机制以及其对比实验【固定缓冲池】初始化缓冲池大小
RequestThreshold = 10

# 几种出发机制运行的标志  True表示运行， False表示不运行
# 第二种出发机制
SecondFlag = True
# NJNP方法实验
NJNPFlag = True
# TADP算法实验
TADPFlag = True
# RCSS 算法实验
RCSSFlag = True



# 使用备份数据时，UseBackupDataFlag = True
# 不使用备份数据时，UseBackupDataFlag = False
# 使用备份数据的条件，预先已经产生相关数据文件，否则不能使用备份数据
UseBackupDataFlag = True
# 数据结果保存路径 (需要使用自己电脑的路径),自己建立一个result文件
# result_path = "E:\\00000000000graduate-study\\GraduateStudentTask\\201810_201909_BeforeGraduateStudent\\python_programming\\Ondemand_Charge\\result"
result_path = "E:\\00000000000graduate-study\\GraduateStudentTask\\201810_201909_BeforeGraduateStudent\\python_programming\\Ondemand_Charge_TwoThreshold\\result"
# 以下为数据初始化
# 节点数目 从 50 到 200 变化，将100节点的实验先 做全# 假设我有N辆电单车  会影响程序运行的时间

NodeNum = 30
# NodeNum = 40
# NodeNum = 20
# 选择插入算法角度阈值设定
# cos90 = 0
# cos180 = -1
# cos90 > cosr > cos180
# 90 < r < 180
# 目前设定
MaxAngle = -1  # 对应最大角度为180°
MinAngle = 0   # 对应最小角度为90°或者270°，如果要修改角度阈值，最好选择修改MinAngle

# 修改MCV的变量 每次 只修改一个
# 1J=1s*1W
# 1KJ=1000J
# Em  数值从150kj 到400kj变化，间隔50kj变化# MC的总能量 j
# Em = 150000.0   
Em = 50000.0
# 测试过程中从4m/s 到10m/s 变化# MC的移动速度 m/s
# Vm = 4.0  
Vm = 5.0
# 初始化电单车的运行速度为 3m/s 
V = 3.0 
# Mc移动功耗为 J/m
# Qm = 55.0  
Qm = 30
# MCV充电传输速率 W
# Qc = 40.0  
Qc = 5
# 类似于效率一样，占比多少 n = 0.5
nl = 0.5  
# 阈值上限当电车剩余能量小于1000.0j时，电车将发送request给MCV
# Et = 500
Et = 225
# 假设阈值下限为0j
El =0
# 每个节点的能量为5600j
# NodeEsValue = 5600
NodeEsValue = 500
# 假设定义的二维空间范围是 edge_n * edge_n 影响构造充电子回路# 电单车运动的空间边长 单位为m
# EdgeLength = 1000
EdgeLength = 500
# 障碍个数
ObstaclesNum = 20  
# 转向变化的度数
AlphaValue = 30

 # 充电周期需要知道7200s
T = 7200.0
  
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

# 实验仿真时间初始化为0.0s
Simulation_time = 0.0

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
# LineColor =['b']
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
childern_result_name = 'Em_' + str(int(Em)) + '_vm_' +str(int(Vm)) + '_v_' + str(int(V)) +'_qm_' + str(int(Qm)) + '_qc_' + str(int(Qc)) + '_ObstaclesNum_' + str(int(ObstaclesNum))
childern_result_name = os.path.join(result_name, childern_result_name)
isExist = os.path.exists(childern_result_name)
if not isExist:
    print "不存在该路径，创建对应路径"
    os.makedirs(childern_result_name)

# 第二种出发机制实验【双阈值】
Second_path = os.path.join(childern_result_name, "Second")
isExist = os.path.exists(Second_path)
if not isExist:
    print "不存在该路径，创建对应路径"
    os.makedirs(Second_path)

# NJNP方法实验路径
NJNP_path = os.path.join(childern_result_name, "NJNP")
isExist = os.path.exists(NJNP_path)
if not isExist:
    print "不存在该路径，创建对应路径"
    os.makedirs(NJNP_path)
# TADP方法实验路径
TADP_path = os.path.join(childern_result_name, "TADP")
isExist = os.path.exists(TADP_path)
if not isExist:
    print "不存在该路径，创建对应路径"
    os.makedirs(TADP_path)
# RCSS方法实验路径
RCSS_path = os.path.join(childern_result_name, "RCSS")
isExist = os.path.exists(RCSS_path)
if not isExist:
    print "不存在该路径，创建对应路径"
    os.makedirs(RCSS_path)

# 创建txt文件
Obstacle_information_data_txt = os.path.join(result_name, 'Obstacle_information_data.txt')
Node_information_data_txt = os.path.join(result_name, 'Node_information_data.txt')

# 第二种出发机制的生成的一些txt文档   
Second_MCV_Tour_Set_txt = os.path.join(Second_path, 'Second_MCV_Tour_Set.txt')

Second_MCV_Tour_Information_txt = os.path.join(Second_path, 'Second_MCV_Tour_Information.txt')

Second_AECR_Txt = os.path.join(Second_path, 'SecondAECRData.txt')
Second_AFP_Txt = os.path.join(Second_path, 'SecondAFPData.txt')

Second_El_wBest_Txt = os.path.join(Second_path, 'SecondEl_wBestData.txt')

Second_Dead_NodeNum_txt = os.path.join(Second_path, 'Second_Dead_NodeNum.txt')

Second_PerformanceSimulation_list_txt  = os.path.join(Second_path, 'Second_PerformanceSimulation_list.txt')

Second_ResponseTimeAndServiceTimeSimulation_list_txt = os.path.join(Second_path, 'Second_ResponseTimeAndServiceTimeSimulation_list.txt')

# 初始化第二种出发机制

# 仿真时间统计
Second_PerformanceSimulationTime_list = []
# 死亡节点和吞吐量的统计
Second_DeadNodeNum_list = []
Second_Throughput_Num_list = []
# 充电能量和移动能量的统计
Second_MCVChargeEs_list = []
Second_MCVMoveEs_list = []
# 充电时间和移动时间的统计
Second_MCVChargeTime_list = []
Second_MCVMoveTime_list = []
# 实际距离和欧几里得距离的统计
Second_MCVRealDistance_list = []
Second_MCVEuclidDistance_list = []


# 平均响应时间：从发送Request信息到被确认为下一服务点时，中间间隔时间
# t = t2(被确认为下一服务节点时) - t1(发送Request信息)
# 定义数组用于保存当节点发送Request时的时间
Second_RequestTime = np.zeros((1, NodeNum + 1), dtype = np.float)
# 定义数组用于保存当前节点被选为下一服务节点(最佳节点)的时间
Second_BestNodeTime = np.zeros((1, NodeNum + 1), dtype = np.float)
# 平均响应时间序列
Second_AverageResponseTime_list = []

# 平均服务时间：从被确认为下一服务点时，到被充电完成，中间间隔时间
# t = MCV移动时间(从当前服务节点出发) + MCV充电时间(下一服务节点)
# 定义数组，用于保存服务时间
Second_ServiceTime = np.zeros((1, NodeNum + 1), dtype = np.float)
# 平均服务时间序列
Second_AverageServiceTime_list = []

# 平均响应时间和平均服务时间的仿真时间 
Second_ResponseTimeAndServiceTimeSimulationTime_list = []


# NJNP对比实验
# NJNP出发机制的生成的一些txt文档   
NJNP_MCV_Tour_Set_txt = os.path.join(NJNP_path, 'NJNP_MCV_Tour_Set.txt')
NJNP_MCV_Tour_Information_txt = os.path.join(NJNP_path, 'NJNP_MCV_Tour_Information.txt')
NJNP_DeadNodeNum_data_txt  = os.path.join(NJNP_path, 'NJNP_DeadNodeNum_data.txt')
NJNP_PerformanceSimulation_list_txt  = os.path.join(NJNP_path, 'NJNP_PerformanceSimulation_list.txt')
NJNP_ResponseTimeAndServiceTimeSimulation_list_txt = os.path.join(NJNP_path, 'NJNP_ResponseTimeAndServiceTimeSimulation_list.txt')

NJNP_AECR_Txt = os.path.join(NJNP_path, 'NJNPAECRData.txt')
NJNP_AFP_Txt = os.path.join(NJNP_path, 'NJNPAFPData.txt')

NJNP_El_wBest_Txt = os.path.join(NJNP_path, 'NJNPEl_wBestData.txt')

# NJNP出发机制对比实验
# 仿真时间统计

NJNP_PerformanceSimulationTime_list = []
# 死亡节点和吞吐量的统计
NJNP_DeadNodeNum_list = []
NJNP_Throughput_Num_list = []
# 充电能量和移动能量的统计
NJNP_MCVChargeEs_list = []
NJNP_MCVMoveEs_list = []
# 充电时间和移动时间的统计
NJNP_MCVChargeTime_list = []
NJNP_MCVMoveTime_list = []
# 实际距离和欧几里得距离的统计
NJNP_MCVRealDistance_list = []
NJNP_MCVEuclidDistance_list = []

# 平均响应时间：从发送Request信息到被确认为下一服务点时，中间间隔时间
# t = t2(被确认为下一服务节点时) - t1(发送Request信息)
# 定义数组用于保存当节点发送Request时的时间
NJNP_RequestTime = np.zeros((1, NodeNum + 1), dtype = np.float)
# 定义数组用于保存当前节点被选为下一服务节点(最佳节点)的时间
NJNP_BestNodeTime = np.zeros((1, NodeNum + 1), dtype = np.float)
# 平均响应时间序列
NJNP_AverageResponseTime_list = []

# 平均服务时间：从被确认为下一服务点时，到被充电完成，中间间隔时间
# t = MCV移动时间(从当前服务节点出发) + MCV充电时间(下一服务节点)
# 定义数组，用于保存服务时间
NJNP_ServiceTime = np.zeros((1, NodeNum + 1), dtype = np.float)
# 平均服务时间序列
NJNP_AverageServiceTime_list = []

# 平均响应时间和平均服务时间的仿真时间 
NJNP_ResponseTimeAndServiceTimeSimulationTime_list = []


# TADP对比实验
# TADP出发机制的生成的一些txt文档   
TADP_MCV_Tour_Set_txt = os.path.join(TADP_path, 'TADP_MCV_Tour_Set.txt')
TADP_MCV_Tour_Information_txt = os.path.join(TADP_path, 'TADP_MCV_Tour_Information.txt')
TADP_DeadNodeNum_data_txt  = os.path.join(TADP_path, 'TADP_DeadNodeNum_data.txt')
TADP_PerformanceSimulation_list_txt  = os.path.join(TADP_path, 'TADP_PerformanceSimulation_list.txt')
TADP_ResponseTimeAndServiceTimeSimulation_list_txt = os.path.join(TADP_path, 'TADP_ResponseTimeAndServiceTimeSimulation_list.txt')

TADP_AECR_Txt = os.path.join(TADP_path, 'TADPAECRData.txt')
TADP_AFP_Txt = os.path.join(TADP_path, 'TADPAFPData.txt')
TADP_El_wBest_Txt = os.path.join(TADP_path, 'TADPEl_wBestData.txt')

# TADP出发机制对比实验
# 仿真时间统计

TADP_PerformanceSimulationTime_list = []
# 死亡节点和吞吐量的统计
TADP_DeadNodeNum_list = []
TADP_Throughput_Num_list = []
# 充电能量和移动能量的统计
TADP_MCVChargeEs_list = []
TADP_MCVMoveEs_list = []
# 充电时间和移动时间的统计
TADP_MCVChargeTime_list = []
TADP_MCVMoveTime_list = []
# 实际距离和欧几里得距离的统计
TADP_MCVRealDistance_list = []
TADP_MCVEuclidDistance_list = []

# 平均响应时间：从发送Request信息到被确认为下一服务点时，中间间隔时间
# t = t2(被确认为下一服务节点时) - t1(发送Request信息)
# 定义数组用于保存当节点发送Request时的时间
TADP_RequestTime = np.zeros((1, NodeNum + 1), dtype = np.float)
# 定义数组用于保存当前节点被选为下一服务节点(最佳节点)的时间
TADP_BestNodeTime = np.zeros((1, NodeNum + 1), dtype = np.float)
# 平均响应时间序列
TADP_AverageResponseTime_list = []

# 平均服务时间：从被确认为下一服务点时，到被充电完成，中间间隔时间
# t = MCV移动时间(从当前服务节点出发) + MCV充电时间(下一服务节点)
# 定义数组，用于保存服务时间
TADP_ServiceTime = np.zeros((1, NodeNum + 1), dtype = np.float)
# 平均服务时间序列
TADP_AverageServiceTime_list = []

# 平均响应时间和平均服务时间的仿真时间 
TADP_ResponseTimeAndServiceTimeSimulationTime_list = []


# RCSS对比实验
# RCSS出发机制的生成的一些txt文档   
RCSS_MCV_Tour_Set_txt = os.path.join(RCSS_path, 'RCSS_MCV_Tour_Set.txt')
RCSS_MCV_Tour_Information_txt = os.path.join(RCSS_path, 'RCSS_MCV_Tour_Information.txt')
RCSS_DeadNodeNum_data_txt  = os.path.join(RCSS_path, 'RCSS_DeadNodeNum_data.txt')
RCSS_PerformanceSimulation_list_txt  = os.path.join(RCSS_path, 'RCSS_PerformanceSimulation_list.txt')
RCSS_ResponseTimeAndServiceTimeSimulation_list_txt = os.path.join(RCSS_path, 'RCSS_ResponseTimeAndServiceTimeSimulation_list.txt')

RCSS_AECR_Txt = os.path.join(RCSS_path, 'RCSSAECRData.txt')
RCSS_AFP_Txt = os.path.join(RCSS_path, 'RCSSAFPData.txt')
RCSS_El_wBest_Txt = os.path.join(RCSS_path, 'RCSSEl_wBestData.txt')


# RCSS出发机制对比实验
# 仿真时间统计

RCSS_PerformanceSimulationTime_list = []
# 死亡节点和吞吐量的统计
RCSS_DeadNodeNum_list = []
RCSS_Throughput_Num_list = []
# 充电能量和移动能量的统计
RCSS_MCVChargeEs_list = []
RCSS_MCVMoveEs_list = []
# 充电时间和移动时间的统计
RCSS_MCVChargeTime_list = []
RCSS_MCVMoveTime_list = []
# 实际距离和欧几里得距离的统计
RCSS_MCVRealDistance_list = []
RCSS_MCVEuclidDistance_list = []

# 平均响应时间：从发送Request信息到被确认为下一服务点时，中间间隔时间
# t = t2(被确认为下一服务节点时) - t1(发送Request信息)
# 定义数组用于保存当节点发送Request时的时间
RCSS_RequestTime = np.zeros((1, NodeNum + 1), dtype = np.float)
# 定义数组用于保存当前节点被选为下一服务节点(最佳节点)的时间
RCSS_BestNodeTime = np.zeros((1, NodeNum + 1), dtype = np.float)
# 平均响应时间序列
RCSS_AverageResponseTime_list = []

# 平均服务时间：从被确认为下一服务点时，到被充电完成，中间间隔时间
# t = MCV移动时间(从当前服务节点出发) + MCV充电时间(下一服务节点)
# 定义数组，用于保存服务时间
RCSS_ServiceTime = np.zeros((1, NodeNum + 1), dtype = np.float)
# 平均服务时间序列
RCSS_AverageServiceTime_list = []

# 平均响应时间和平均服务时间的仿真时间 
RCSS_ResponseTimeAndServiceTimeSimulationTime_list = []

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
def TourConstructionInformation(El, R_Sum, NodeEs, DeadNodeNum_data_txt, N_distance, NodeXCoordinateNew, NodeYCoordinateNew, MCV_Tour_Set_txt, MCV_Tour_Information_txt, ObstacleCoordinate, mechanism):
    if DebugFlag is True:
        print "充电回路汇总"
        print "R_Sum =", R_Sum
    if DebugFlag is True:
        print "len(R_Sum) =", len(R_Sum)
        print "节点剩余能量 NodeEs[0] =", NodeEs[0]
        # 总结死亡节点数量，通过剩余能量来判断
        print "NodeEs[0] =", NodeEs[0]
    
    # 用输入的节点的数量进行统计
    DeadNodeNum = 0
    for i in range(1, NodeNum + 1):
        # 一个节点剩余能量低于0时，表明该节点已经死亡
        if NodeEs[0][i] <= 0:
            DeadNodeNum = DeadNodeNum + 1
    if DebugFlag is True:
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
        if DebugFlag is True:
            print "R_Sum[i] =\n", R_Sum[i]
        ChargingTour = R_Sum[i]
        if DebugFlag is True:
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
    if DebugFlag is True:
        print "DataStore_list_sum =\n", DataStore_list_sum   
    f1 = open(MCV_Tour_Set_txt, 'w')
    f1.write(str(R_Sum))
    f1.close()
    # 回路消耗等相关数据的保存路径
    np.savetxt(MCV_Tour_Information_txt, DataStore_list_sum, fmt='%0.2f')
    ChildrenTourConstruction(NodeXCoordinateNew, NodeYCoordinateNew, ObstacleCoordinate, R_Sum, 'S', NodeNum, mechanism)
    return

# 根据时间间隔统计相关数据
def SummaryByTime(El, mechanism, R_Sum, Simulation_time, NodeEs, N_distance, NodeXCoordinateNew, NodeYCoordinateNew, ObstacleCoordinate):
    # print "当前充电回路汇总"
    # print "R_Sum =", R_Sum
    if DebugFlag is True:
        print "len(R_Sum) =", len(R_Sum)
        print "节点剩余能量 NodeEs[0] =", NodeEs[0]
    # 总结死亡节点数量，通过剩余能量来判断
    # print "NodeEs[0] =", NodeEs[0]
    
    # 用剩余能量=0的节点的数量进行统计
    DeadNodeNum = 0
    for i in range(1, NodeNum + 1):
        # 一个节点剩余能量低于0时，表明该节点已经死亡
        if NodeEs[0][i] <= 0:
            DeadNodeNum = DeadNodeNum + 1
    # print "死亡节点个数DeadNodeNum =", DeadNodeNum
    
    # 吞吐量数目
    Throughput_Num = 0
    
    # MCV充电能量统计
    MCVChargeEs = 0.0
    # MCV移动能量统计
    MCVMoveEs = 0.0
    
    # MCV充电时间统计
    MCVChargeTime = 0.0
    # MCV移动时间统计
    MCVMoveTime = 0.0
    
    # MCV移动实际距离统计
    MCVRealDistance = 0.0
    # MCV移动欧几里得距离统计
    MCVEuclidDistance = 0.0
    
    for i in range(0, len(R_Sum)):
        # print "R_Sum[i] =\n", R_Sum[i]
        ChargingTour = R_Sum[i]
        # print "len(ChargingTour) =", len(ChargingTour)
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
        
        # 添加充电回路的吞吐量
        # DataStore_list.append(len(ChargingTour) - 1)
        # 添加吞吐量
        Throughput_Num = Throughput_Num + (len(ChargingTour) - 1)
        # 添加MCV给这个回路充电的电量
        # DataStore_list.append(round(ChargingTour_Es, 2))
        MCVChargeEs = MCVChargeEs +  round(ChargingTour_Es, 2)
    
        # 添加MCV在这个回路中的移动能耗
        # DataStore_list.append(round(Move_Es, 2))
        # MCV移动能量统计
        MCVMoveEs = MCVMoveEs + round(Move_Es, 2)
        
        # 添加MCV给这个回路充电的时间
        # DataStore_list.append(round(ChargingTour_Time, 2))
        # MCV充电时间统计
        MCVChargeTime = MCVChargeTime + round(ChargingTour_Time, 2)
    
        # 添加MCV在这个回路中移动所花时间
        # DataStore_list.append(round(Move_time, 2))
        # MCV移动时间统计
        MCVMoveTime = MCVMoveTime + round(Move_time, 2)
        
        # 添加回路的路径长度（实际）
        # DataStore_list.append(round(D, 2))
        # MCV移动实际距离统计
        MCVRealDistance = MCVRealDistance + round(D, 2) 
    
        # 添加回路路径长度（欧几里得路径）
        # DataStore_list.append(round(Euclid_D, 2))
        
        # MCV移动欧几里得距离统计
        MCVEuclidDistance = MCVEuclidDistance + round(Euclid_D, 2)
 
    if mechanism == 'Second':
        # 仿真时间统计
        Second_PerformanceSimulationTime_list.append(Simulation_time)
        # 死亡节点和吞吐量的统计
        Second_DeadNodeNum_list.append(DeadNodeNum)
        Second_Throughput_Num_list.append(Throughput_Num)
        # 充电能量和移动能量的统计
        Second_MCVChargeEs_list.append(MCVChargeEs)
        Second_MCVMoveEs_list.append(MCVMoveEs)
        # 充电时间和移动时间的统计
        Second_MCVChargeTime_list.append(MCVChargeTime)
        Second_MCVMoveTime_list.append(MCVMoveTime)
        # 实际距离和欧几里得距离的统计
        Second_MCVRealDistance_list.append(MCVRealDistance)
        Second_MCVEuclidDistance_list.append(MCVEuclidDistance)
    
    elif mechanism == 'NJNP':
        # 仿真时间统计
        NJNP_PerformanceSimulationTime_list.append(Simulation_time)
        # 死亡节点和吞吐量的统计
        NJNP_DeadNodeNum_list.append(DeadNodeNum)
        NJNP_Throughput_Num_list.append(Throughput_Num)
        # 充电能量和移动能量的统计
        NJNP_MCVChargeEs_list.append(MCVChargeEs)
        NJNP_MCVMoveEs_list.append(MCVMoveEs)
        # 充电时间和移动时间的统计
        NJNP_MCVChargeTime_list.append(MCVChargeTime)
        NJNP_MCVMoveTime_list.append(MCVMoveTime)
        # 实际距离和欧几里得距离的统计
        NJNP_MCVRealDistance_list.append(MCVRealDistance)
        NJNP_MCVEuclidDistance_list.append(MCVEuclidDistance)
    elif mechanism == 'TADP':
        # 仿真时间统计
        TADP_PerformanceSimulationTime_list.append(Simulation_time)
        # 死亡节点和吞吐量的统计
        TADP_DeadNodeNum_list.append(DeadNodeNum)
        TADP_Throughput_Num_list.append(Throughput_Num)
        # 充电能量和移动能量的统计
        TADP_MCVChargeEs_list.append(MCVChargeEs)
        TADP_MCVMoveEs_list.append(MCVMoveEs)
        # 充电时间和移动时间的统计
        TADP_MCVChargeTime_list.append(MCVChargeTime)
        TADP_MCVMoveTime_list.append(MCVMoveTime)
        # 实际距离和欧几里得距离的统计
        TADP_MCVRealDistance_list.append(MCVRealDistance)
        TADP_MCVEuclidDistance_list.append(MCVEuclidDistance)
    elif mechanism == 'RCSS':
        # 仿真时间统计
        RCSS_PerformanceSimulationTime_list.append(Simulation_time)
        # 死亡节点和吞吐量的统计
        RCSS_DeadNodeNum_list.append(DeadNodeNum)
        RCSS_Throughput_Num_list.append(Throughput_Num)
        # 充电能量和移动能量的统计
        RCSS_MCVChargeEs_list.append(MCVChargeEs)
        RCSS_MCVMoveEs_list.append(MCVMoveEs)
        # 充电时间和移动时间的统计
        RCSS_MCVChargeTime_list.append(MCVChargeTime)
        RCSS_MCVMoveTime_list.append(MCVMoveTime)
        # 实际距离和欧几里得距离的统计
        RCSS_MCVRealDistance_list.append(MCVRealDistance)
        RCSS_MCVEuclidDistance_list.append(MCVEuclidDistance)
    return
    

# 将子回路首尾连接起来
def ChildrenTourConstruction(x_new, y_new, obstacle_coordinate_new, R_result, S_Flag_new, N, mechanism):
    if DebugFlag is True:
        print "模拟轨迹##########\n"
    Style_num = 0
    Color_num = 0
    result = R_result
    # 充电回路的名字
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
    
    z = len(R_list)

    # 打印检查产生的充电子回路有多少条，每条子回路由那些节点组成
    for p in range(0, z):
        R_value = R_list[p]
        goal = R_value
        # S点为红色正方形，并且大一点
        if p == 0:
            if k == 0:
                S_Flag = S_Flag_new
            else:
                if DebugFlag is True:
                    print "goal[0] =", goal[0]
                S_Flag = 'S' + '&' + str(goal[0])
            ax.scatter(x_new[0][goal[0]], y_new[0][goal[0]], s = 100, color = 'k',label = S_Flag, marker = 's')
            ax.text(x_new[0][goal[0]], y_new[0][goal[0]], S_Flag, fontsize=20)
        
        # 用于图例显示MC（）
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
    
    Graph_Name = mechanism + '_Final.png'
    # print "Graph_Name =", Graph_Name
    origin_path = os.path.join(childern_result_name , mechanism)
    # print "origin_path =", origin_path  
    All_path = os.path.join(origin_path, Graph_Name)
    # print "All_path =", All_path
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
def ChangeCoordinate(i, El, Et, RequestTime, Simulation_time, NodeList, NodeEs, NodeP, t_sum, NodeEsBackup, NodeMoveTime, NodeXCoordinateNew, NodeYCoordinateNew, V, Alpha, AlphaValue, EdgeLength, ObstacleCoordinate, ObstaclesNum):
    # 修改节点能量
    NodeEs[0][NodeList[i]] = round((NodeEs[0][NodeList[i]] - NodeP[0][NodeList[i]]*t_sum), 2)
    if DebugFlag is True:
        print "预计消耗的能量 NodeP[0][NodeList[i]]*t =", NodeP[0][NodeList[i]]*t_sum, 'j'
        print "死亡能量阈值El=", 0, 'j'
        print "能量阈值Et=", Et, 'j'
    # 能量将低于阈值下限
    if NodeEs[0][NodeList[i]] <= 0:
        if NodeEsBackup[i] <= 0:
            if DebugFlag is True:
                print "case 1"
                print "之前该节点已经失效,不再参与计算"
            # 剩余能量还是上次备份的能量
            NodeEs[0][NodeList[i]] = NodeEsBackup[i]
            # 即使节点不动，也要给它的时间赋值，避免之后节点的坐标会发生改变
            NodeMoveTime[0][NodeList[i]] = 0
        else:
            if DebugFlag is True:
                print "case 2"
                print "NodeEs[0][NodeList[i]]=", NodeEs[0][NodeList[i]]
                print "节点实际消耗的能量为", NodeEsBackup[i] - 0, 'j'
            # 修改节点剩余能量,该点变为静态点了
            NodeEs[0][NodeList[i]] = 0
            if DebugFlag is True:
                print "节点实际运动的时间为", (NodeEsBackup[i] - 0)/NodeP[0][NodeList[i]], 's'
            NodeMoveTime[0][NodeList[i]] = (NodeEsBackup[i] - 0)/NodeP[0][NodeList[i]]
    # 能量将低于阈值上限，将发出Request信号
    if NodeEs[0][NodeList[i]] <= Et:
        if NodeEsBackup[i] <= Et:
            # 表明之前就已经发送过Request信号了，对应节点发送Request的时间就是上一次的时间
            RequestTime[0][NodeList[i]] = RequestTime[0][NodeList[i]] 
            # print "RequestTime[0][", NodeList[i], "] =", RequestTime[0][NodeList[i]]
        else:
            # 表明节点剩余能量首次小于能量阈值上限，计算节点发送Request请求时的时间
            RequestTime[0][NodeList[i]] = Simulation_time - t_sum + (NodeEsBackup[i] - Et)/NodeP[0][NodeList[i]]
            # print "RequestTime[0][", NodeList[i], "] =", RequestTime[0][NodeList[i]]
    # 能量不会低于阈值下限
    if NodeEs[0][NodeList[i]] > 0:
        if DebugFlag is True:
            print "case 3"
            print "NodeEs[0][NodeList[i]]=", NodeEs[0][NodeList[i]]
            print "节点实际消耗的能量为", NodeP[0][NodeList[i]]*t_sum, 'j'
            print "节点实际运动的时间为", t_sum, 's'
        NodeMoveTime[0][NodeList[i]] = t_sum
    if DebugFlag is True:
        print "NodeMoveTime[0]=", NodeMoveTime[0]
        print "NodeEs[0]=", NodeEs[0]
    # 修改节点的运动时间，因为当节点能量小于阈值El时，电单车节点变为静态节点，保留阈值El能量
    t = NodeMoveTime[0][NodeList[i]]
    t_new = t
    # if DebugFlag is True:
    print "t_new=", t_new
    while ((EdgeLength/2)/V) < t:
        t = ((EdgeLength/2)/V)
        # if DebugFlag is True:
        print "t=", t, 's'

        # 考虑到边界问题，先将改变之前的坐标记录下来
        N_x_new_temp = NodeXCoordinateNew[0][NodeList[i]]
        N_y_new_temp = NodeYCoordinateNew[0][NodeList[i]] 
        N_x_new_temp_new = NodeXCoordinateNew[0][NodeList[i]]
        N_y_new_temp_new = NodeYCoordinateNew[0][NodeList[i]]
        # 更新节点坐标
        NodeXCoordinateNew[0][NodeList[i]] = round((NodeXCoordinateNew[0][NodeList[i]] + V*t*math.cos((Alpha[0][NodeList[i]]*math.pi)/180.0)), 2)
        NodeYCoordinateNew[0][NodeList[i]] = round((NodeYCoordinateNew[0][NodeList[i]] + V*t*math.sin((Alpha[0][NodeList[i]]*math.pi)/180.0)), 2)
        # 检查坐标有没有超过边界, 表示超界了，表示当前需要改变运行方向
        # 为什么用while（）一定保证，坐标合理才能进行下一步操作
        # 为了在运动时避开障碍，设置两种状态
        # （1）在无障碍情况下计算距离
        # （2）在有障碍的情况下再计算一次距离
        # 如果两次距离相等，则中途没有障碍，可以直接到达目的位置
        # 反之，不相等，则中途出现障碍，不能直接到达目的地，需要改变运动方向
        # obstacle_flag = True 表明空间存在障碍
        beforex = NodeXCoordinateNew[0][NodeList[i]]
        beforey = NodeYCoordinateNew[0][NodeList[i]] 
        print "while True , change coordinate"
        originAlphaValue = []
        originAlphaValue.append(Alpha[0][NodeList[i]])
         
        while True:
            if t == 0:
                # print "表明当前节点能量已经为0，已经不能在运动了，视为死亡节点"
                break
            # 判断坐标是否越界
            # print "begin check coordinate"
            # 当前转向角不适合时，可以对转向角每次都进行微调
            AlphaValueStep = 1
            AlphaValueSum = 0
            while((NodeXCoordinateNew[0][NodeList[i]] < 1 or NodeXCoordinateNew[0][NodeList[i]] > EdgeLength) or (
                    NodeYCoordinateNew[0][NodeList[i]] < 1 or NodeYCoordinateNew[0][NodeList[i]] > EdgeLength)): 
                # 更新电单车运动方向 改变方向幅度不能太大 
                # print "originAlphaValue =", originAlphaValue
                # print "AlphaValue =", AlphaValue
                AlphaValueSum = AlphaValueSum + AlphaValue
                Alpha[0][NodeList[i]] = (Alpha[0][NodeList[i]] + AlphaValue)%360
                # print "really operation time t =", t, 's'
                # print "Alpha[0][", NodeList[i], "] =", Alpha[0][NodeList[i]] 
                # print "N_x_new_temp =", N_x_new_temp
                # print "N_y_new_temp =", N_y_new_temp
                # 用上面备份的当前的坐标，重新往重新生成的方向前进
                NodeXCoordinateNew[0][NodeList[i]] = round((N_x_new_temp + V*t*math.cos((Alpha[0][NodeList[i]]*math.pi)/180.0)), 2)
                NodeYCoordinateNew[0][NodeList[i]] = round((N_y_new_temp + V*t*math.sin((Alpha[0][NodeList[i]]*math.pi)/180.0)), 2)
                # print "beforex =", beforex
                # print "beforey =", beforey
                # print "NodeXCoordinateNew[0][", NodeList[i], "] =", NodeXCoordinateNew[0][NodeList[i]]
                # print "NodeYCoordinateNew[0][", NodeList[i], "] =", NodeYCoordinateNew[0][NodeList[i]]
                # print "AlphaValueSum =", AlphaValueSum
                # 如果修改了电单车偏向角后，还会出现陷入死循环的情况，则方向角增加
                if ((beforex == NodeXCoordinateNew[0][NodeList[i]]) and (beforey == NodeYCoordinateNew[0][NodeList[i]])) or (AlphaValueSum%360 == 0):
                    AlphaValueSum = 0
                    # print "dead while delay(1)"
                    # time.sleep(1)
                    # 说明AlphaValueStep已经循环了一圈，只能从修改t下手了
                    # print "operation time t =", t, 's'
                    # 变化90°范围就差不多了的
                    if AlphaValueStep == 90:
                        # 因为感觉，修改角度没什么用了
                        # 只好从时间上做修改，每次时间剩余为上次的99%
                        t = t*(99.0/100.0)       
                        # 顺便把AlphaValueStep 初始化为1
                        AlphaValueStep = 1
                    # print "修改后的 t =", t, 's'
                    # AlphaValue不符合当前的那个数值，我们将其局部修改
                    AlphaValue = AlphaValueStep
                    # 步长加1
                    AlphaValueStep = AlphaValueStep + 1
                # print "delay(3)s"
                # time.sleep(3)
            # print "check coordinate over"    
            # 越界处理完成的标志，表示已经不越界
            crossing_flag = True
            # 电单车转向减小一点
            AlphaValue = 1
            # 还得保证当前终点不属于任何一个障碍区域内
            first_coordinate = []
            second_coordinate = []
            first_coordinate.append(N_x_new_temp)
            first_coordinate.append(N_y_new_temp)
            second_coordinate.append(NodeXCoordinateNew[0][NodeList[i]])
            second_coordinate.append(NodeYCoordinateNew[0][NodeList[i]])
            # print "有障碍 的二维空间"
            distance_obstacle =A.a_star_algorithm(EdgeLength, first_coordinate, second_coordinate, ObstacleCoordinate, ObstaclesNum, True)
            # print "退出有障碍空间"
            # 判断是否需要变向
            change_direction_flag = True
            # 如果终点坐标在障碍区域内，也需要改变运动方向
            if distance_obstacle[0] is False:
                change_direction_flag = False
                # 更新电单车运动方向 改变方向幅度不能太大
                Alpha[0][NodeList[i]] = (Alpha[0][NodeList[i]] + AlphaValue)%360
                # 用上面备份的当前的坐标，重新往重新生成的方向前进
                NodeXCoordinateNew[0][NodeList[i]] = round((N_x_new_temp + V*t*math.cos((Alpha[0][NodeList[i]]*math.pi)/180.0)), 2)
                NodeYCoordinateNew[0][NodeList[i]] = round((N_y_new_temp + V*t*math.sin((Alpha[0][NodeList[i]]*math.pi)/180.0)), 2)                    
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
                if np.abs(distance_no_obstacle[3] - distance_obstacle[3]) != 0.0:
                    change_direction_flag = False
                    # 更新电单车运动方向 改变方向幅度不能太大
                    Alpha[0][NodeList[i]] = (Alpha[0][NodeList[i]] + AlphaValue)%360
                    # 用上面备份的当前的坐标，重新往重新生成的方向前进
                    NodeXCoordinateNew[0][NodeList[i]] = round((N_x_new_temp + V*t*math.cos((Alpha[0][NodeList[i]]*math.pi)/180.0)), 2)
                    NodeYCoordinateNew[0][NodeList[i]] = round((N_y_new_temp + V*t*math.sin((Alpha[0][NodeList[i]]*math.pi)/180.0)), 2)
                    # print "遇到障碍重新更新坐标为：", (N_x_new[0][N_i[i]], N_y_new[0][N_i[i]])
                if crossing_flag is True and change_direction_flag is True:
                    # 起点到终点途中不存在障碍或终点不存在与障碍区域
                    # 退出while()
                    break
        t_new = (t_new - ((EdgeLength/2)/V))
        # print "t_new =", t_new, 's'
        t =  t_new
        # print "t =", t, 's'
    # print "over while ((EdgeLength/2)/V) < t"
    print "t =", t, 's'

    # 考虑到边界问题，先将改变之前的坐标记录下来
    N_x_new_temp = NodeXCoordinateNew[0][NodeList[i]]
    N_y_new_temp = NodeYCoordinateNew[0][NodeList[i]] 
    N_x_new_temp_new = NodeXCoordinateNew[0][NodeList[i]]
    N_y_new_temp_new = NodeYCoordinateNew[0][NodeList[i]]
    # 更新节点坐标
    NodeXCoordinateNew[0][NodeList[i]] = round((NodeXCoordinateNew[0][NodeList[i]] + V*t*math.cos((Alpha[0][NodeList[i]]*math.pi)/180.0)), 2)
    NodeYCoordinateNew[0][NodeList[i]] = round((NodeYCoordinateNew[0][NodeList[i]] + V*t*math.sin((Alpha[0][NodeList[i]]*math.pi)/180.0)), 2)
    
    # print "while True , change coordinate"
    beforex = NodeXCoordinateNew[0][NodeList[i]]
    beforey = NodeYCoordinateNew[0][NodeList[i]] 
    originAlphaValue = []
    originAlphaValue.append(Alpha[0][NodeList[i]])

    while True:
        if t == 0:
            # print "表明当前节点能量已经为0，已经不能在运动了，视为死亡节点"
            break
        # 判断坐标是否越界
        # print "begin check coordinate"
        # 当前转向角不适合时，可以对转向角每次都进行微调
        AlphaValueStep = 1
        AlphaValueSum = 0
        while((NodeXCoordinateNew[0][NodeList[i]] < 1 or NodeXCoordinateNew[0][NodeList[i]] > EdgeLength) or (
                NodeYCoordinateNew[0][NodeList[i]] < 1 or NodeYCoordinateNew[0][NodeList[i]] > EdgeLength)): 
            # 更新电单车运动方向 改变方向幅度不能太大 
            # print "originAlphaValue =", originAlphaValue
            # print "AlphaValue =", AlphaValue
            AlphaValueSum = AlphaValueSum + AlphaValue
            Alpha[0][NodeList[i]] = (Alpha[0][NodeList[i]] + AlphaValue)%360
            # print "really operation time t =", t, 's'
            # print "Alpha[0][", NodeList[i], "] =", Alpha[0][NodeList[i]] 
            # print "N_x_new_temp =", N_x_new_temp
            # print "N_y_new_temp =", N_y_new_temp
            # 用上面备份的当前的坐标，重新往重新生成的方向前进
            NodeXCoordinateNew[0][NodeList[i]] = round((N_x_new_temp + V*t*math.cos((Alpha[0][NodeList[i]]*math.pi)/180.0)), 2)
            NodeYCoordinateNew[0][NodeList[i]] = round((N_y_new_temp + V*t*math.sin((Alpha[0][NodeList[i]]*math.pi)/180.0)), 2)
            # print "beforex =", beforex
            # print "beforey =", beforey
            # print "NodeXCoordinateNew[0][", NodeList[i], "] =", NodeXCoordinateNew[0][NodeList[i]]
            # print "NodeYCoordinateNew[0][", NodeList[i], "] =", NodeYCoordinateNew[0][NodeList[i]]
            # print "AlphaValueSum =", AlphaValueSum
            # 如果修改了电单车偏向角后，还会出现陷入死循环的情况，则方向角增加
            if ((beforex == NodeXCoordinateNew[0][NodeList[i]]) and (beforey == NodeYCoordinateNew[0][NodeList[i]])) or (AlphaValueSum%360 == 0):
                AlphaValueSum = 0
                # print "dead while delay(1)"
                # time.sleep(1)
                # 说明AlphaValueStep已经循环了一圈，只能从修改t下手了
                # print "operation time t =", t, 's'
                # 变化90°范围就差不多了的
                if AlphaValueStep == 90:
                    # 因为感觉，修改角度没什么用了
                    # 只好从时间上做修改，每次时间剩余为上次的99%
                    t = t*(99.0/100.0)       
                    # 顺便把AlphaValueStep 初始化为1
                    AlphaValueStep = 1
                # print "修改后的 t =", t, 's'
                # AlphaValue不符合当前的那个数值，我们将其局部修改
                AlphaValue = AlphaValueStep
                # 步长加1
                AlphaValueStep = AlphaValueStep + 1
            # print "delay(3)s"
            # time.sleep(3)
        # print "check coordinate over"    
        # 越界处理完成的标志，表示已经不越界
        crossing_flag = True
        # 电单车转向减小一点
        AlphaValue = 1
        # 还得保证当前终点不属于任何一个障碍区域内
        first_coordinate = []
        second_coordinate = []
        first_coordinate.append(N_x_new_temp)
        first_coordinate.append(N_y_new_temp)
        second_coordinate.append(NodeXCoordinateNew[0][NodeList[i]])
        second_coordinate.append(NodeYCoordinateNew[0][NodeList[i]])
        # print "有障碍 的二维空间"
        distance_obstacle =A.a_star_algorithm(EdgeLength, first_coordinate, second_coordinate, ObstacleCoordinate, ObstaclesNum, True)
        # print "退出有障碍空间"
        # 判断是否需要变向
        change_direction_flag = True
        # 如果终点坐标在障碍区域内，也需要改变运动方向
        if distance_obstacle[0] is False:
            change_direction_flag = False
            # 更新电单车运动方向 改变方向幅度不能太大
            Alpha[0][NodeList[i]] = (Alpha[0][NodeList[i]] + AlphaValue)%360
            # 用上面备份的当前的坐标，重新往重新生成的方向前进
            NodeXCoordinateNew[0][NodeList[i]] = round((N_x_new_temp + V*t*math.cos((Alpha[0][NodeList[i]]*math.pi)/180.0)), 2)
            NodeYCoordinateNew[0][NodeList[i]] = round((N_y_new_temp + V*t*math.sin((Alpha[0][NodeList[i]]*math.pi)/180.0)), 2)                    
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
            if np.abs(distance_no_obstacle[3] - distance_obstacle[3]) != 0.0:
                change_direction_flag = False
                # 更新电单车运动方向 改变方向幅度不能太大
                Alpha[0][NodeList[i]] = (Alpha[0][NodeList[i]] + AlphaValue)%360
                # 用上面备份的当前的坐标，重新往重新生成的方向前进
                NodeXCoordinateNew[0][NodeList[i]] = round((N_x_new_temp + V*t*math.cos((Alpha[0][NodeList[i]]*math.pi)/180.0)), 2)
                NodeYCoordinateNew[0][NodeList[i]] = round((N_y_new_temp + V*t*math.sin((Alpha[0][NodeList[i]]*math.pi)/180.0)), 2)
                # print "遇到障碍重新更新坐标为：", (N_x_new[0][N_i[i]], N_y_new[0][N_i[i]])
            if crossing_flag is True and change_direction_flag is True:
                # 起点到终点途中不存在障碍或终点不存在与障碍区域
                # 退出while()
                break
    return t
# 定义选择插入算法
def SelectInsertAlgorithm(i, Simulation_time, BestNodeTime,N_distance, NodeEsValue,NodeEs,ServiceTime,Vm,Qc, R_list, R_list_Backup, R_New, NodeList, NodeRequest, NodeALERT, Ax, Ay, Bx, By, MinAngle, MaxAngle, NodeRequestFlag_New):
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
    t = 0.0
    if cosr < MinAngle and cosr >= MaxAngle:
        # 只插入一个中途节点就好
        # 当该节点被选为最佳下一服务点时，将当前时间进行统计
        BestNodeTime[0][R_list[i]] = Simulation_time
        if DebugFlag is True:
            print "BestNodeTime[0][", R_list[i], "] =", BestNodeTime[0][R_list[i]] 
        if DebugFlag is True:
            # print "进入发送过ALERT信息的序列中~~~~~~~~~"
            print "InsertNode ", R_list[i]
        R_New_FisrtValue = R_New[0]
        R_New[0] = 0
        R_NewInser = R_New[len(R_New) - 1]
        R_New.remove(R_NewInser)
        R_New.append(R_list[i])
        R_New.append(R_NewInser)
        R_New[0] = R_New_FisrtValue
        
        
        D = N_distance[R_New[len(R_New) - 2]][R_New[len(R_New) - 3]]
        t1 = D/Vm
         # 消耗的能量
        last_Es = NodeEsValue - NodeEs[0][R_New[len(R_New) - 2]]
        # 充电需要的时间
        t2 = last_Es/Qc
        # 将R_New[len(R_New) - 1]加入充电子回路需要消耗的总时间为t
        t_sum = t1 + t2 
        t = t_sum
        Simulation_time = Simulation_time + t
        if DebugFlag is True:
            print "Simulation_time =", Simulation_time, 's'
            # if DebugFlag is True:
            print "InsertNode Spend t =", t, 's'
        
        ServiceTime[0][R_list[i]] = t
        if DebugFlag is True:
            print "ServiceTime[0][", R_list[i], "] =", ServiceTime[0][R_list[i]] 
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
        NodeRequestFlag_New = True
        if DebugFlag is True:
            # print "ALERT信息中插入节点的结果检查"
            print "R_New =", R_New
            print "R_list =", R_list
            print "R_list_Backup =", R_list_Backup
            print "NodeList =", NodeList
    result = []
    result.append(NodeRequestFlag_New)
    result.append(t)
    result.append(Simulation_time)
    result.append(BestNodeTime)
    result.append(ServiceTime)
    return result
# 定义选择排序 A 对二维列表排序，从第1个开始
def select_sort_Numsort(array):
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

if __name__ == "__main__":
    print "Programming is Begin"
    ProgrammingStartTime = time.time()
    # 不使用备份数据时，备份数据标志为False
    # 数据初始化
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
        
    if SecondFlag is True:
        print "Second Task MCV move to Charge when Accept ALERT "
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
        
        # 程序开始运行，计时开始
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
            print "程序正在执行,请等待"
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
                    Simulation_time = Simulation_time + t
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
                            print "NodeEsBackup =", NodeEsBackup
                            ConsumptionSum = 0.0
                            # 每个节点运动t时间后，统计所消耗的能量，以及成为静态节点的个数
                            for i in range(1, len(NodeList)):
                                # 修改节点能量
                                NodeEs[0][NodeList[i]] = round((NodeEs[0][NodeList[i]] - NodeP[0][NodeList[i]]*t), 2)
                                ConsumptionSum = ConsumptionSum + NodeP[0][NodeList[i]]*t 
                                if DebugFlag is True:
                                    print "预计消耗的能量 NodeP[0][NodeList[i]]*t=", NodeP[0][NodeList[i]]*t, 'j'
                                    print "能量阈值下限El=", El, 'j'
                                
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
                                print "静态节点个数StaticNodeNum=", StaticNodeNum, '个' 
                                print "节点运行总能量NodeOperateEs=", NodeOperateEs, 'j'
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
                        np.savetxt(Second_AECR_Txt, ElNodeOperateEsAllList,fmt='%0.5f')
                        np.savetxt(Second_AFP_Txt, ElStaticNodeAllList, fmt='%0.5f')
                        
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
                        # w > 1 说明侧重点不一样而已
                        w = 1.0
                        wBest = 0.0
                        ElBefore = 0.0
                        # 可以重复很多次的，自己设定就好
                        while wBest == 0.0:
                            ElNew = -((A1*B1+w*A2*B2)/(np.power(A1, 2)+ w*np.power(A2, 2)))
                            wList.append(w)
                            ElList.append(ElNew)
                            if np.abs(ElBefore - ElNew) < WAccuracy:
                                # 表明前后两个El值非常接近
                                ElFinal = ElNew
                                wBest = w 
                                break
                            ElBefore = ElNew
                            # 每迭代一次，w + 0.1
                            w = w + 0.1
                        f = np.polyfit(wList,ElList,2)
                        p = np.poly1d(f)
                        if DebugFlag is True:
                            print "f =", f
                            print "p =", p
                        if DebugFlag is True:
                            print "wBest =", wBest
                            print "阈值上限为 Et=", Et
                            print "在这次运行过程中比较适合的阈值下限ElFinal=", ElFinal
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
                            El = 0
                            if DebugFlag is True:
                                print "El修改为El =", El
                        Ellist = []
                        Ellist.append(El)
                        Ellist.append(wBest)
                        np.savetxt(Second_El_wBest_Txt, Ellist,fmt='%0.2f')
                       
                    # 每次能量都需要恢复
                    for i in range(0, len(NodeList)):
                        NodeEs[0][NodeList[i]] = NodeEsBackup[i]
                    if DebugFlag is True:
                        print "NodeEs[0] =", NodeEs[0]  
                        
                    # 以上部分是为了计算得到能量阈值下限El
                    # 更新节点剩余的能量  # 不仅仅是考虑边界，还得考虑运动过程中，遇到障碍应该避开
                    DeadNodeNumber = 0
                    for i in range(1, len(NodeList)):
                        # 修改节点能量
                        t = ChangeCoordinate(i, El, Et, Second_RequestTime, Simulation_time, NodeList, NodeEs, NodeP, t_sum, NodeEsBackup, NodeMoveTime, NodeXCoordinateNew, NodeYCoordinateNew, V, Alpha, AlphaValue, EdgeLength, ObstacleCoordinate, ObstaclesNum)
                        if t == 0:
                            # t == 0 说明节点已经死亡
                            DeadNodeNumber = DeadNodeNumber + 1
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
                            
                            plt.plot(wList, ElList, 'b:*', label = 'El')
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
                            # plt,show()
                        if (i == (len(NodeList) - 1)) and ((len(NodeList) - 1) != DeadNodeNumber):
                            if DebugFlag is True:
                                print "死亡节点个数 DeadNodeNumber =", DeadNodeNumber
                                print "节点个数 len(NodeList)-1 =", len(NodeList) - 1
                            print "coordinate have been changed, change distance"
                            N_distance_Road_result = F.CreateDistanceNewMatrix(Road_information, N_distance, EdgeLength, NodeXCoordinateNew, NodeYCoordinateNew, NodeListBackup, NodeList, ObstacleCoordinate, ObstaclesNum)
                            N_distance = N_distance_Road_result[0]
                            Road_information = N_distance_Road_result[1]
                           
                # 就在这个环节，将一部分节点（R_list中的节点）从原NodeList中删掉
                elif NodeALERTFlag is True:
                    if DebugFlag is True:
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
                    # MCV给电单车节点的充电功率
                    qc = Qc
                    # 类似于效率一样，占比多少 n = 0.5
                    Nl = nl  
                    JudgeResult = B.judging_whether_scheduled(NodeP,Em, qc, Qm, Nl, R_New, Vm, N_distance)
                    # 使用调度性条件判断是否满足决策条件
                    print "JudgeResult =", JudgeResult
                    if JudgeResult is True:
                        # 当该节点被选为最佳下一服务点时，将当前时间进行统计
                        Second_BestNodeTime[0][BestNode] = Simulation_time
                        if DebugFlag is True:
                            print "Second_BestNodeTime[0][", BestNode, "] =", Second_BestNodeTime[0][BestNode] 
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
                        SelectInsertAlgorithmTime = 0.0
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
                                    SelectInsertAlgorithmResult = SelectInsertAlgorithm(i, Simulation_time, Second_BestNodeTime,N_distance, NodeEsValue,NodeEs,Second_ServiceTime,Vm,Qc, R_list, R_list_Backup, R_New, NodeList, NodeRequest, NodeALERT, Ax, Ay, Bx, By, MinAngle, MaxAngle, NodeALERTFlag_New)
                                    # print "插入算法结果 SelectInsertAlgorithmResult =", SelectInsertAlgorithmResult
                                    NodeALERTFlag_New = SelectInsertAlgorithmResult[0]
                                    SelectInsertAlgorithmTime = SelectInsertAlgorithmResult[1]
                                    Simulation_time = SelectInsertAlgorithmResult[2]
                                    Second_BestNodeTime = SelectInsertAlgorithmResult[3]
                                    Second_ServiceTime = SelectInsertAlgorithmResult[4]
                                    if NodeALERTFlag_New == True:
                                        # 节点运行过程中，只允许插入一个节点
                                        break
                            # 剩余节点没有发送过ALERT信息
                            if NodeALERTFlag_New is False:
                                for i in range(1, len(R_list)):
                                    if NodeRequest[0][R_list[i]] == 1:
                                        SelectInsertAlgorithmResult = SelectInsertAlgorithm(i, Simulation_time, Second_BestNodeTime,N_distance, NodeEsValue,NodeEs,Second_ServiceTime,Vm,Qc, R_list, R_list_Backup, R_New, NodeList, NodeRequest, NodeALERT, Ax, Ay, Bx, By, MinAngle, MaxAngle, NodeALERTFlag_New)
                                        # print "插入算法结果 SelectInsertAlgorithmResult =", SelectInsertAlgorithmResult
                                        NodeALERTFlag_New = SelectInsertAlgorithmResult[0]
                                        SelectInsertAlgorithmTime = SelectInsertAlgorithmResult[1]
                                        Simulation_time = SelectInsertAlgorithmResult[2]
                                        Second_BestNodeTime = SelectInsertAlgorithmResult[3]
                                        Second_ServiceTime = SelectInsertAlgorithmResult[4]
                                        if NodeALERTFlag_New == True:
                                            # 节点运行过程中，只允许插入一个节点
                                            break
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
                        t = t1 + t2
                        
                        Simulation_time = Simulation_time + t
                        if DebugFlag is True:
                            print "Simulation_time =", Simulation_time, 's'
                        t_sum = t1 + t2 + SelectInsertAlgorithmTime
                        if DebugFlag is True:
                            print "将某点加入充电子回路中需要消耗的总时间为t =", t, 's'
                        Second_ServiceTime[0][BestNode] = t
                        if DebugFlag is True:
                            print "Second_ServiceTime[0][", BestNode, "] =", Second_ServiceTime[0][BestNode] 
                        
                        # 添加平均响应时间和平均服务时间
                        if DebugFlag is True:
                            print "NodeListBackup =", NodeListBackup
                            print "len(NodeListBackup) =", len(NodeListBackup)
                        # 被服务过的节点个数的初始化
                        ServiceNode = 0 
                        # 每个节点响应时间的总和
                        ResponseNodeTime = 0.0
                        # 每个节点服务时间求总和
                        ServiceNodeTime = 0.0
                        
                        for m in range(1, len(NodeListBackup)):
                            # 说明这个节点已经被添加到回路了，或者说已经被操作过的
                            if Second_ServiceTime[0][NodeListBackup[m]] != 0:
                               # 当前选中的节点的相关时间的统计
                               # 响应时间
                               Second_RequestTime[0][NodeListBackup[m]]
                               # 选择下一最佳服务节点的时间
                               Second_BestNodeTime[0][NodeListBackup[m]]
                               
                               # 已经被操作过（被添加回路）的节点个数的统计
                               ServiceNode = ServiceNode + 1 
                               # 节点响应时间求总和
                               ResponseNodeTime = ResponseNodeTime + (Second_BestNodeTime[0][NodeListBackup[m]] - Second_RequestTime[0][NodeListBackup[m]])
                               # 节点服务时间求总和
                               ServiceNodeTime = ServiceNodeTime + Second_ServiceTime[0][NodeListBackup[m]]
                        
                        # 添加平均响应时间
                        Second_AverageResponseTime_list.append(round(ResponseNodeTime/ServiceNode, 2))
                        # 添加平均服务时间
                        Second_AverageServiceTime_list.append(round(ServiceNodeTime/ServiceNode, 2))
                        
                        # 添加当前的仿真时间
                        Second_ResponseTimeAndServiceTimeSimulationTime_list.append(Simulation_time)
                        
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
                        DeadNodeNumber = 0
                        for i in range(1, len(NodeList)):
                            # 修改节点能量
                            t = ChangeCoordinate(i, El, Et, Second_RequestTime, Simulation_time, NodeList, NodeEs, NodeP, t_sum, NodeEsBackup, NodeMoveTime, NodeXCoordinateNew, NodeYCoordinateNew, V, Alpha, AlphaValue, EdgeLength, ObstacleCoordinate, ObstaclesNum)
                            if t == 0:
                                # t == 0时，说明节点已经死亡
                                DeadNodeNumber = DeadNodeNumber + 1
                            # 测试数据用的 事后需要将其注释掉
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
                                        # 判断当前发送Request信号的节点是否存在与R_list中
                                        R_listExistFlag = False
                                        for j in range(0, len(R_list)):
                                           if NodeList[i] == R_list[j]: 
                                                # 当前发送Request的节点在R_list中
                                                R_listExistFlag = True
                                        if R_listExistFlag is False:
                                            # 当前发送了Request的节点不存在R_list，将其添加入R_list中
                                            R_list.append(NodeList[i])
                                NodeALERTFlag = True
                            # 修改最后一个节点的位置，随之改变距离邻接矩阵
                            # 死亡节点个数不等于剩余节点个数
                            if (i == (len(NodeList) - 1)) and ((len(NodeList) - 1) != DeadNodeNumber):
                                if DebugFlag is True:
                                    print "死亡节点个数 DeadNodeNumber =", DeadNodeNumber
                                    print "节点个数 len(NodeList) - 1=", len(NodeList) - 1
                                print "change the distance"
                                # 主要是获取各节点之间的距离
                                N_distance_Road_result = F.CreateDistanceNewMatrix(Road_information, N_distance, EdgeLength, NodeXCoordinateNew, NodeYCoordinateNew, NodeListBackup, NodeList, ObstacleCoordinate, ObstaclesNum)
                                N_distance = N_distance_Road_result[0]
                                Road_information = N_distance_Road_result[1] 
                                if DebugFlag is True:
                                    print "节点距离修改完毕"
                        # 临时的回路汇总，为R_Sum和R_New的集合
                        # print "添加相关性能信息~"
                        R_Sum_Temp = []
                        # print "R_Sum =", R_Sum
                        for i in range(0, len(R_Sum)):
                            R_Sum_Temp.append(R_Sum[i])
                        # print "R_Sum_Temp =", R_Sum_Temp
                        # print "R_New =", R_New
                        R_Sum_Temp.append(R_New)
                        # print "R_Sum_Temp =", R_Sum_Temp                        
                        SummaryByTime(El, 'Second', R_Sum_Temp, Simulation_time, NodeEs, N_distance, NodeXCoordinateNew, NodeYCoordinateNew, ObstacleCoordinate)
                        
                    else:
                        R_New_FirstValue = R_New[0]
                        R_New[0] = 0
                        R_New.remove(BestNode)
                        R_New[0] = R_New_FirstValue
                    if DebugFlag is True:
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
                            # SummaryByTime(El, 'Second', R_Sum, Simulation_time, NodeEs, N_distance, NodeXCoordinateNew, NodeYCoordinateNew, ObstacleCoordinate)
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
                        # SummaryByTime(El, 'Second', R_Sum, Simulation_time, NodeEs, N_distance, NodeXCoordinateNew, NodeYCoordinateNew, ObstacleCoordinate)
                        # 发送请求的节点已经全部加入充电回路了,退出while循环，重新获取充电请求
                        break
        # 添加构建回路的相关信息
        TourConstructionInformation(El, R_Sum, NodeEs, Second_Dead_NodeNum_txt, N_distance, NodeXCoordinateNew, NodeYCoordinateNew, Second_MCV_Tour_Set_txt, Second_MCV_Tour_Information_txt, ObstacleCoordinate, 'Second')
        
        result_list = []
        # 仿真时间
        result_list.append(Second_PerformanceSimulationTime_list)
        # 死亡节点和吞吐量
        result_list.append(Second_DeadNodeNum_list)
        result_list.append(Second_Throughput_Num_list)
        # 充电能量和移动能量
        result_list.append(Second_MCVChargeEs_list)
        result_list.append(Second_MCVMoveEs_list)
        # 充电时间和移动时间
        result_list.append(Second_MCVChargeTime_list)
        result_list.append(Second_MCVMoveTime_list)
        # 实际距离和欧几里得距离
        result_list.append(Second_MCVRealDistance_list)
        result_list.append(Second_MCVEuclidDistance_list)
        # 保存性能数据
        np.savetxt(Second_PerformanceSimulation_list_txt, result_list, fmt='%0.2f')
        if DebugFlag is True:
            print "Second_RequestTime[0] =\n", Second_RequestTime[0]
            print "Second_BestNodeTime[0] =\n", Second_BestNodeTime[0]
            print "Second_ServiceTime[0] =\n", Second_ServiceTime[0]
            print "Second_ResponseTimeAndServiceTimeSimulationTime_list =\n", Second_ResponseTimeAndServiceTimeSimulationTime_list
            print "Second_AverageResponseTime_list =\n", Second_AverageResponseTime_list
            print "Second_AverageServiceTime_list =\n", Second_AverageServiceTime_list
        result_list = []
        result_list.append(Second_ResponseTimeAndServiceTimeSimulationTime_list)
        result_list.append(Second_AverageResponseTime_list)
        result_list.append(Second_AverageServiceTime_list)
        # 保存平均响应时间和平均服务时间的仿真数据
        np.savetxt(Second_ResponseTimeAndServiceTimeSimulation_list_txt, result_list, fmt='%0.2f')
        # 回路分配算法
        print "TourDistributioning"
        MCVSum = TD.TourDistributionProgramming(Second_MCV_Tour_Information_txt)
        print "len(MCVSum) =", len(MCVSum)
        print "over TourDistribution"
    
    if NJNPFlag is True:
        # 统一El的值
        # Eldata = np.loadtxt(El_wBest_Txt)
        # print "Eldata[0] =", Eldata[0]
        # El初始化为0
        El = 0
        Simulation_time = 0.0
        print "NJNP Task"
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
        # 充电回路汇总
        R_Sum = []
        # 初始化节点发送Request请求的数量
        RequestNum = 0 
        # 将发送Request的节点序列进行保存
        Request_list = []
        # 需要对每个节点进行操作，直到剩余服务站S的标号
        while 1 != len(NodeList):
            NodeALERTFlag = False
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
                    Simulation_time = Simulation_time + t
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
                            print "NodeEsBackup =", NodeEsBackup
                            ConsumptionSum = 0.0
                            # 每个节点运动t时间后，统计所消耗的能量，以及成为静态节点的个数
                            for i in range(1, len(NodeList)):
                                # 修改节点能量
                                NodeEs[0][NodeList[i]] = round((NodeEs[0][NodeList[i]] - NodeP[0][NodeList[i]]*t), 2)
                                ConsumptionSum = ConsumptionSum + NodeP[0][NodeList[i]]*t 
                                if DebugFlag is True:
                                    print "预计消耗的能量 NodeP[0][NodeList[i]]*t=", NodeP[0][NodeList[i]]*t, 'j'
                                    print "能量阈值下限El=", El, 'j'
                                
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
                                print "静态节点个数StaticNodeNum=", StaticNodeNum, '个' 
                                print "节点运行总能量NodeOperateEs=", NodeOperateEs, 'j'
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
                        np.savetxt(NJNP_AECR_Txt, ElNodeOperateEsAllList,fmt='%0.5f')
                        np.savetxt(NJNP_AFP_Txt, ElStaticNodeAllList, fmt='%0.5f')
                        
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
                        # w > 1 说明侧重点不一样而已
                        w = 1.0
                        wBest = 0.0
                        ElBefore = 0.0
                        # 可以重复很多次的，自己设定就好
                        while wBest == 0.0:
                            ElNew = -((A1*B1+w*A2*B2)/(np.power(A1, 2)+ w*np.power(A2, 2)))
                            wList.append(w)
                            ElList.append(ElNew)
                            if np.abs(ElBefore - ElNew) < WAccuracy:
                                # 表明前后两个El值非常接近
                                ElFinal = ElNew
                                wBest = w 
                                break
                            ElBefore = ElNew
                            # 每迭代一次，w + 0.1
                            w = w + 0.1
                        f = np.polyfit(wList,ElList,2)
                        p = np.poly1d(f)
                        if DebugFlag is True:
                            print "f =", f
                            print "p =", p
                        if DebugFlag is True:
                            print "wBest =", wBest
                            print "阈值上限为 Et=", Et
                            print "在这次运行过程中比较适合的阈值下限ElFinal=", ElFinal
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
                            El = 0
                            if DebugFlag is True:
                                print "El修改为El =", El
                        Ellist = []
                        Ellist.append(El)
                        Ellist.append(wBest)
                        np.savetxt(NJNP_El_wBest_Txt, Ellist,fmt='%0.2f')
                    # 每次能量都需要恢复
                    for i in range(0, len(NodeList)):
                        NodeEs[0][NodeList[i]] = NodeEsBackup[i]
                    if DebugFlag is True:
                        print "NodeEs[0] =", NodeEs[0]
                    # 以上部分是为了计算得到能量阈值下限El
                    # 更新节点剩余的能量  # 不仅仅是考虑边界，还得考虑运动过程中，遇到障碍应该避开
                    DeadNodeNumber = 0
                    for i in range(1, len(NodeList)):
                        # 修改节点能量
                        t = ChangeCoordinate(i, El, Et, NJNP_RequestTime, Simulation_time, NodeList, NodeEs, NodeP, t_sum, NodeEsBackup, NodeMoveTime, NodeXCoordinateNew, NodeYCoordinateNew, V, Alpha, AlphaValue, EdgeLength, ObstacleCoordinate, ObstaclesNum)
                        if t == 0:
                            # t == 0 表示该节点为死亡节点
                            DeadNodeNumber = DeadNodeNumber + 1
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
                            origin_path =  'NJNPAECR&AFP.png'  
                            All_path = os.path.join(NJNP_path, origin_path)
                            plt.savefig(All_path)
                            
                            plt.show()
                            
                            plt.plot(wList, ElList, 'b:*', label = 'El')
                            plt.xlabel('w')
                            plt.ylabel('El')
                            plt.legend(loc=2) #指定legend的位置右下角
                            plt.grid()
                            plt.title('polyfitting')
                            # 保存生成的图片
                            origin_path =  'NJNPAEl.png'  
                            All_path = os.path.join(NJNP_path, origin_path)
                            plt.savefig(All_path)
                            plt.show()
                            # plt,show()
                        if (i == (len(NodeList) - 1)) and ((len(NodeList) - 1) != DeadNodeNumber):
                            if DebugFlag is True:
                                print "死亡节点个数 DeadNodeNumber =", DeadNodeNumber
                                print "节点个数 len(NodeList)-1 =", len(NodeList) - 1
                            print "coordinate have been changed, change distance"
                            N_distance_Road_result = F.CreateDistanceNewMatrix(Road_information, N_distance, EdgeLength, NodeXCoordinateNew, NodeYCoordinateNew, NodeListBackup, NodeList, ObstacleCoordinate, ObstaclesNum)
                            N_distance = N_distance_Road_result[0]
                            Road_information = N_distance_Road_result[1]
                           
                # 就在这个环节，将一部分节点（R_list中的节点）从原NodeList中删掉
                elif NodeALERTFlag is True:
                    if DebugFlag is True:
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
                    # if DebugFlag is True:
                    print "R_list =", R_list
                    print "R_list_Backup =", R_list_Backup
                    
                    # 选择最佳下一服务点的方法为：选择距离MCV近的节点插入充电回路
                    MCVAndServiceNode_distance = 100000000.0
                    BestNode = 0
                    for i in range(1, len(R_list_Backup)):
                        # print "N_distance[R_New[len(R_New) - 1]][Request_list[i]] =", N_distance[R_New[len(R_New) - 1]][Request_list[i]]
                        # 如果初始化的MCV到服务节点的距离比实际距离大，则修改初始化的值，并找出当前较优下一服务节点
                        if MCVAndServiceNode_distance > N_distance[R_New[len(R_New) - 1]][R_list_Backup[i]]:
                            MCVAndServiceNode_distance = N_distance[R_New[len(R_New) - 1]][R_list_Backup[i]]
                            # 找出当前较优节点
                            BestNode = R_list_Backup[i]
                    # 本次寻找最好的节点为 BestNode
                    # 该点不再运动
                    
                    print "BestNode =", BestNode
                    # 将最好的节点加入构建的充电回路中
                    R_New.append(BestNode)
                    # 当该节点被选为最佳下一服务点时，将当前时间进行统计
                    # R_list_Backup中的值每次都得进行删除操作，主要是怕不能依次对每个节点进行查询操作
                    R_list_Backup_FirstValue = R_list_Backup[0]
                    R_list_Backup[0] = 0
                    R_list_Backup.remove(BestNode)
                    R_list_Backup[0] = R_list_Backup_FirstValue
                    if DebugFlag is True:
                        print "R_list_Backup =", R_list_Backup

                    NJNP_BestNodeTime[0][BestNode] = Simulation_time
                    if DebugFlag is True:
                        print "NJNP_BestNodeTime[0][", BestNode, "] =", NJNP_BestNodeTime[0][BestNode] 
                
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
                    Simulation_time = Simulation_time + t
                    NJNP_ServiceTime[0][BestNode] = t
                    if DebugFlag is True:
                        print "NJNP_ServiceTime[0][", BestNode, "] =", NJNP_ServiceTime[0][BestNode] 
                        
                        # 添加平均响应时间和平均服务时间
                        print "NodeListBackup =", NodeListBackup
                        print "len(NodeListBackup) =", len(NodeListBackup)
                    # 被服务过的节点个数的初始化
                    ServiceNode = 0 
                    # 每个节点响应时间的总和
                    ResponseNodeTime = 0.0
                    # 每个节点服务时间求总和
                    ServiceNodeTime = 0.0
                    
                    for m in range(1, len(NodeListBackup)):
                        # 说明这个节点已经被添加到回路了，或者说已经被操作过的
                        if NJNP_ServiceTime[0][NodeListBackup[m]] != 0:
                           # 当前选中的节点的相关时间的统计
                           # 响应时间
                           NJNP_RequestTime[0][NodeListBackup[m]]
                           # 选择下一最佳服务节点的时间
                           NJNP_BestNodeTime[0][NodeListBackup[m]]
                           
                           # 已经被操作过（被添加回路）的节点个数的统计
                           ServiceNode = ServiceNode + 1 
                           # 节点响应时间求总和
                           ResponseNodeTime = ResponseNodeTime + (NJNP_BestNodeTime[0][NodeListBackup[m]] - NJNP_RequestTime[0][NodeListBackup[m]])
                           # 节点服务时间求总和
                           ServiceNodeTime = ServiceNodeTime + NJNP_ServiceTime[0][NodeListBackup[m]]
                    
                    # 添加平均响应时间
                    NJNP_AverageResponseTime_list.append(round(ResponseNodeTime/ServiceNode, 2))
                    # 添加平均服务时间
                    NJNP_AverageServiceTime_list.append(round(ServiceNodeTime/ServiceNode, 2))
                    
                    # 添加当前的仿真时间
                    NJNP_ResponseTimeAndServiceTimeSimulationTime_list.append(Simulation_time)
                    
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
                    DeadNodeNumber = 0
                    for i in range(1, len(NodeList)):
                        # 修改节点能量
                        t = ChangeCoordinate(i, El, Et, NJNP_RequestTime, Simulation_time, NodeList, NodeEs, NodeP, t_sum, NodeEsBackup, NodeMoveTime, NodeXCoordinateNew, NodeYCoordinateNew, V, Alpha, AlphaValue, EdgeLength, ObstacleCoordinate, ObstaclesNum)
                        if t == 0:
                            # 当t == 0时，说明当前节点为死亡节点
                            DeadNodeNumber = DeadNodeNumber + 1
                        # 测试数据用的 事后需要将其注释掉
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
                                    # 判断当前发送Request信号的节点是否存在与R_list中
                                    R_listExistFlag = False
                                    for j in range(0, len(R_list)):
                                       if NodeList[i] == R_list[j]: 
                                            # 当前发送Request的节点在R_list中
                                            R_listExistFlag = True
                                    if R_listExistFlag is False:
                                        # 当前发送了Request的节点不存在R_list，将其添加入R_list中
                                        R_list.append(NodeList[i])
                            NodeALERTFlag = True
                        # 修改最后一个节点的位置，随之改变距离邻接矩阵
                        # 死亡节点个数不等于剩余节点个数
                        if (i == (len(NodeList) - 1)) and ((len(NodeList) - 1) != DeadNodeNumber):
                            if DebugFlag is True:
                                print "死亡节点个数 DeadNodeNumber =", DeadNodeNumber
                                print "节点个数 len(NodeList) - 1=", len(NodeList) - 1
                            print "change the distance"
                            # 主要是获取各节点之间的距离
                            N_distance_Road_result = F.CreateDistanceNewMatrix(Road_information, N_distance, EdgeLength, NodeXCoordinateNew, NodeYCoordinateNew, NodeListBackup, NodeList, ObstacleCoordinate, ObstaclesNum)
                            N_distance = N_distance_Road_result[0]
                            Road_information = N_distance_Road_result[1] 
                            if DebugFlag is True:
                                print "节点距离修改完毕" 
                    # 临时的回路汇总，为R_Sum和R_New的集合
                    # print "添加相关性能信息~"
                    R_Sum_Temp = []
                    # print "R_Sum =", R_Sum
                    for i in range(0, len(R_Sum)):
                        R_Sum_Temp.append(R_Sum[i])
                    # print "R_Sum_Temp =", R_Sum_Temp
                    # print "R_New =", R_New
                    R_Sum_Temp.append(R_New)
                    # print "R_Sum_Temp =", R_Sum_Temp                        
                    SummaryByTime(El, 'NJNP', R_Sum_Temp, Simulation_time, NodeEs, N_distance, NodeXCoordinateNew, NodeYCoordinateNew, ObstacleCoordinate)
                                                       
                    
                    if DebugFlag is True:
                        print "R_New =", R_New
                        print "R_list =", R_list
                        print "R_list_Backup =", R_list_Backup
                        print "NodeList =", NodeList
                        print "Request_list =", Request_list
                    
                    if len(R_list_Backup) == 1 and len(R_list) != 1:
                        if DebugFlag is True:
                            print "重新创建充电回路"
                            print "len(R_New) =", len(R_New)
                        # 重新备份
                        # 将所有充电回路进行汇总
                        # 当充电回路中只有服务站S时，不需要添加到充电回路集合中
                        if len(R_New) != 1:
                            R_Sum.append(R_New)
                            # SummaryByTime(El, 'Second', R_Sum, Simulation_time, NodeEs, N_distance, NodeXCoordinateNew, NodeYCoordinateNew, ObstacleCoordinate)
                        R_list_Backup_flag = True

                    if len(R_list) == 1:
                        # 统计当前节点序号列表中的个数，即未进行操作节点的个数
                        if DebugFlag is True:
                            print "对发送过Request请求的节点已经操作完毕"
                        NodeListNum = len(NodeList)
                        R_list_Backup_flag = True
                        NodeALERTFlag = False
                        R_Sum.append(R_New)
                        # SummaryByTime(El, 'Second', R_Sum, Simulation_time, NodeEs, N_distance, NodeXCoordinateNew, NodeYCoordinateNew, ObstacleCoordinate)
                        # 发送请求的节点已经全部加入充电回路了,退出while循环，重新获取充电请求
                        break
        # 添加构建回路的相关信息
        TourConstructionInformation(El, R_Sum, NodeEs, NJNP_DeadNodeNum_data_txt, N_distance, NodeXCoordinateNew, NodeYCoordinateNew, NJNP_MCV_Tour_Set_txt, NJNP_MCV_Tour_Information_txt, ObstacleCoordinate, 'NJNP')
        # 添加几个性能与仿真时间的关系
        result_list = []
        # 仿真时间
        result_list.append(NJNP_PerformanceSimulationTime_list)
        # 死亡节点和吞吐量
        result_list.append(NJNP_DeadNodeNum_list)
        result_list.append(NJNP_Throughput_Num_list)
        # 充电能量和移动能量
        result_list.append(NJNP_MCVChargeEs_list)
        result_list.append(NJNP_MCVMoveEs_list)
        # 充电时间和移动时间
        result_list.append(NJNP_MCVChargeTime_list)
        result_list.append(NJNP_MCVMoveTime_list)
        # 实际距离和欧几里得距离
        result_list.append(NJNP_MCVRealDistance_list)
        result_list.append(NJNP_MCVEuclidDistance_list)
        
        np.savetxt(NJNP_PerformanceSimulation_list_txt, result_list, fmt='%0.2f')
        if DebugFlag is True:
            print "NJNP_RequestTime[0] =\n", NJNP_RequestTime[0]
            print "NJNP_BestNodeTime[0] =\n", NJNP_BestNodeTime[0]
            print "NJNP_ServiceTime[0] =\n", NJNP_ServiceTime[0]
            print "NJNP_ResponseTimeAndServiceTimeSimulationTime_list =\n", NJNP_ResponseTimeAndServiceTimeSimulationTime_list
            print "NJNP_AverageResponseTime_list =\n", NJNP_AverageResponseTime_list
            print "NJNP_AverageServiceTime_list =\n", NJNP_AverageServiceTime_list
        result_list = []
        result_list.append(NJNP_ResponseTimeAndServiceTimeSimulationTime_list)
        result_list.append(NJNP_AverageResponseTime_list)
        result_list.append(NJNP_AverageServiceTime_list)
        
        np.savetxt(NJNP_ResponseTimeAndServiceTimeSimulation_list_txt, result_list, fmt='%0.2f')
        # 回路分配算法
        print "TourDistributioning"
        MCVSum = TD.TourDistributionProgramming(NJNP_MCV_Tour_Information_txt)
        print "len(MCVSum) =", len(MCVSum)
        print "over TourDistribution"
    
    if TADPFlag is True:
        # 统一El的值
        # Eldata = np.loadtxt(El_wBest_Txt)
        # print "Eldata[0] =", Eldata[0]
        # El初始化为0
        El = 0

        Simulation_time = 0.0
        print "TADP Task"
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
                    Simulation_time = Simulation_time + t
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
                            print "NodeEsBackup =", NodeEsBackup
                            ConsumptionSum = 0.0
                            # 每个节点运动t时间后，统计所消耗的能量，以及成为静态节点的个数
                            for i in range(1, len(NodeList)):
                                # 修改节点能量
                                NodeEs[0][NodeList[i]] = round((NodeEs[0][NodeList[i]] - NodeP[0][NodeList[i]]*t), 2)
                                ConsumptionSum = ConsumptionSum + NodeP[0][NodeList[i]]*t 
                                if DebugFlag is True:
                                    print "预计消耗的能量 NodeP[0][NodeList[i]]*t=", NodeP[0][NodeList[i]]*t, 'j'
                                    print "能量阈值下限El=", El, 'j'
                                
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
                                print "静态节点个数StaticNodeNum=", StaticNodeNum, '个' 
                                print "节点运行总能量NodeOperateEs=", NodeOperateEs, 'j'
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
                        np.savetxt(TADP_AECR_Txt, ElNodeOperateEsAllList,fmt='%0.5f')
                        np.savetxt(TADP_AFP_Txt, ElStaticNodeAllList, fmt='%0.5f')
                        
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
                        # w > 1 说明侧重点不一样而已
                        w = 1.0
                        wBest = 0.0
                        ElBefore = 0.0
                        # 可以重复很多次的，自己设定就好
                        while wBest == 0.0:
                            ElNew = -((A1*B1+w*A2*B2)/(np.power(A1, 2)+ w*np.power(A2, 2)))
                            wList.append(w)
                            ElList.append(ElNew)
                            if np.abs(ElBefore - ElNew) < WAccuracy:
                                # 表明前后两个El值非常接近
                                ElFinal = ElNew
                                wBest = w 
                                break
                            ElBefore = ElNew
                            # 每迭代一次，w + 0.1
                            w = w + 0.1
                        f = np.polyfit(wList,ElList,2)
                        p = np.poly1d(f)
                        if DebugFlag is True:
                            print "f =", f
                            print "p =", p
                        if DebugFlag is True:
                            print "wBest =", wBest
                            print "阈值上限为 Et=", Et
                            print "在这次运行过程中比较适合的阈值下限ElFinal=", ElFinal
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
                            El = 0
                            if DebugFlag is True:
                                print "El修改为El =", El
                        Ellist = []
                        Ellist.append(El)
                        Ellist.append(wBest)
                        np.savetxt(TADP_El_wBest_Txt, Ellist,fmt='%0.2f')
                    # 每次能量都需要恢复
                    for i in range(0, len(NodeList)):
                        NodeEs[0][NodeList[i]] = NodeEsBackup[i]
                    if DebugFlag is True:
                        print "NodeEs[0] =", NodeEs[0]
                    # 以上部分是为了计算得到能量阈值下限El
                    # 更新节点剩余的能量  # 不仅仅是考虑边界，还得考虑运动过程中，遇到障碍应该避开
                    DeadNodeNumber = 0
                    for i in range(1, len(NodeList)):
                        # 修改节点能量
                        t = ChangeCoordinate(i, El, Et, TADP_RequestTime, Simulation_time, NodeList, NodeEs, NodeP, t_sum, NodeEsBackup, NodeMoveTime, NodeXCoordinateNew, NodeYCoordinateNew, V, Alpha, AlphaValue, EdgeLength, ObstacleCoordinate, ObstaclesNum)
                        if t == 0:
                            # t == 0 表示该节点为死亡节点
                            DeadNodeNumber = DeadNodeNumber + 1
                        # 测试数据用的 事后需要将其注释掉
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
                            origin_path =  'TADPAECR&AFP.png'  
                            All_path = os.path.join(TADP_path, origin_path)
                            plt.savefig(All_path)
                            
                            plt.show()
                            
                            plt.plot(wList, ElList, 'b:*', label = 'El')
                            plt.xlabel('w')
                            plt.ylabel('El')
                            plt.legend(loc=2) #指定legend的位置右下角
                            plt.grid()
                            plt.title('polyfitting')
                            # 保存生成的图片
                            origin_path =  'TADPAEl.png'  
                            All_path = os.path.join(TADP_path, origin_path)
                            plt.savefig(All_path)
                            plt.show()
                            # plt,show()
                        if (i == (len(NodeList) - 1)) and ((len(NodeList) - 1) != DeadNodeNumber):
                            if DebugFlag is True:
                                print "死亡节点个数 DeadNodeNumber =", DeadNodeNumber
                                print "节点个数 len(NodeList)-1 =", len(NodeList) - 1
                            print "coordinate have been changed, change distance"
                            N_distance_Road_result = F.CreateDistanceNewMatrix(Road_information, N_distance, EdgeLength, NodeXCoordinateNew, NodeYCoordinateNew, NodeListBackup, NodeList, ObstacleCoordinate, ObstaclesNum)
                            N_distance = N_distance_Road_result[0]
                            Road_information = N_distance_Road_result[1]
                           
                # 就在这个环节，将一部分节点（R_list中的节点）从原NodeList中删掉
                elif NodeALERTFlag is True:
                    if DebugFlag is True:
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
                    # if DebugFlag is True:
                    print "R_list =", R_list
                    # 选择最佳下一服务点的方法为：综合考虑剩余能量和与MCV之间的距离
                    # 计算得到P，以P最小为下一最佳服务节点
                    BestNode = 0
                    # a 为剩余能量所占比重
                    # b 为S到各电单车距离所占比重
                    # P 为综合考虑剩余能量和S到各电单车距离所占比重考虑的最终值
                    # 尽可能把剩余能量少的和与MCV距离短的节点优先选择为下一服务节点
                    Es_sort = []
                    N_distance_sort = []
                    for i in range(1, len(R_list_Backup)):
                        # 获取节点及其剩余能量，组成(NodeNum, NodeEs[0][NodeNum])
                        # 这样操作存在，剩余能量的排名的变化同步节点序号的变化
                        Node_number_Es = []
                        Node_number_Es.append(R_list_Backup[i])
                        # 获取节点剩余能量
                        Node_number_Es.append(round(NodeEs[0][R_list_Backup[i]],2))
                        Es_sort.append(Node_number_Es)
                        # 获取节点及其与MCV的距离，组成(NodeNum, N_distance[NodeNum(MCV)][NodeNum]
                        # 这样操作存在，与MCV的距离排名的变化同步节点序号的变化
                        Node_number_distance= []
                        Node_number_distance.append(R_list_Backup[i])
                        # 获取MCV到当前节点的距离
                        Node_number_distance.append(round(N_distance[R_New[len(R_New) - 1]][R_list_Backup[i]], 2))
                        N_distance_sort.append(Node_number_distance)
                    print "before Es_sort =", Es_sort
                    print "before N_distance_sort =", N_distance_sort
                    # 已经将剩余能量和与MCV的距离按从小到大排序
                    select_sort_Numsort(Es_sort)
                    select_sort_Numsort(N_distance_sort)
                    print "after Es_sort =", Es_sort
                    print "after N_distance_sort =", N_distance_sort

                    a = 0.2
                    b = 1.0 - a
                    # 只考虑了当前一次，所以得看看如何把所有的点考虑进去，每个点都要考虑到
                    # 只考虑当前所剩下的节点，不包括S点（N_i[0]）
                    # 假设开始时的P非常非常的大
                    P = []
                    # 这里应该计算剩余所有节点的P值
                    for i in range(1 , len(R_list_Backup)):
                        # 临时列表
                        P_Node_number = []
                        # 添加节点序号
                        P_Node_number.append(R_list_Backup[i])
                        Es_Rank = 0
                        for l in range(1, len(R_list_Backup)):
                            if R_list_Backup[i] == Es_sort[l - 1][0]:
                                # 当前剩余能量节点的排名
                                Es_Rank = l
                                break
                        Distance_Rank = 0
                        for m in range(1, len(R_list_Backup)):
                            if R_list_Backup[i] == N_distance_sort[m - 1][0]:
                                # 当前 节点距离S的排名
                                Distance_Rank = m
                                break

                        P_value = a*Es_Rank + b*Distance_Rank
                        
                        P_Node_number.append(round(P_value, 2))
                        # 添加节点的排名的比重
                        P.append(P_Node_number)
            
                    print "before P =", P
                    select_sort_Numsort(P)
                    print "after P =", P
                    # 本次寻找最好的节点为 BestNode
                    # 该点不再运动
                    BestNode = P[0][0]
                    print "BestNode =", BestNode
                    # print "delay(10)"
                    # time.sleep(10)
                    # 将最好的节点加入构建的充电回路中
                    R_New.append(BestNode)
                    # 当该节点被选为最佳下一服务点时，将当前时间进行统计
                    # R_list_Backup中的值每次都得进行删除操作，主要是怕不能依次对每个节点进行查询操作
                    R_list_Backup_FirstValue = R_list_Backup[0]
                    R_list_Backup[0] = 0
                    R_list_Backup.remove(BestNode)
                    R_list_Backup[0] = R_list_Backup_FirstValue
                    if DebugFlag is True:
                        print "R_list_Backup =", R_list_Backup

                    # 当该节点被选为最佳下一服务点时，将当前时间进行统计
                    TADP_BestNodeTime[0][BestNode] = Simulation_time
                    if DebugFlag is True:
                        print "TADP_BestNodeTime[0][", BestNode, "] =", TADP_BestNodeTime[0][BestNode] 
                
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
                    Simulation_time = Simulation_time + t
                    TADP_ServiceTime[0][BestNode] = t
                    if DebugFlag is True:
                        print "TADP_ServiceTime[0][", BestNode, "] =", TADP_ServiceTime[0][BestNode] 
                        
                        # 添加平均响应时间和平均服务时间
                        print "NodeListBackup =", NodeListBackup
                        print "len(NodeListBackup) =", len(NodeListBackup)
                    # 被服务过的节点个数的初始化
                    ServiceNode = 0 
                    # 每个节点响应时间的总和
                    ResponseNodeTime = 0.0
                    # 每个节点服务时间求总和
                    ServiceNodeTime = 0.0
                    
                    for m in range(1, len(NodeListBackup)):
                        # 说明这个节点已经被添加到回路了，或者说已经被操作过的
                        if TADP_ServiceTime[0][NodeListBackup[m]] != 0:
                           # 当前选中的节点的相关时间的统计
                           # 响应时间
                           TADP_RequestTime[0][NodeListBackup[m]]
                           # 选择下一最佳服务节点的时间
                           TADP_BestNodeTime[0][NodeListBackup[m]]
                           
                           # 已经被操作过（被添加回路）的节点个数的统计
                           ServiceNode = ServiceNode + 1 
                           # 节点响应时间求总和
                           ResponseNodeTime = ResponseNodeTime + (TADP_BestNodeTime[0][NodeListBackup[m]] - TADP_RequestTime[0][NodeListBackup[m]])
                           # 节点服务时间求总和
                           ServiceNodeTime = ServiceNodeTime + TADP_ServiceTime[0][NodeListBackup[m]]
                    
                    # 添加平均响应时间
                    TADP_AverageResponseTime_list.append(round(ResponseNodeTime/ServiceNode, 2))
                    # 添加平均服务时间
                    TADP_AverageServiceTime_list.append(round(ServiceNodeTime/ServiceNode, 2))
                    
                    # 添加当前的仿真时间
                    TADP_ResponseTimeAndServiceTimeSimulationTime_list.append(Simulation_time)
                    
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
                    DeadNodeNumber = 0
                    for i in range(1, len(NodeList)):
                        # 修改节点能量
                        t = ChangeCoordinate(i, El, Et, TADP_RequestTime, Simulation_time, NodeList, NodeEs, NodeP, t_sum, NodeEsBackup, NodeMoveTime, NodeXCoordinateNew, NodeYCoordinateNew, V, Alpha, AlphaValue, EdgeLength, ObstacleCoordinate, ObstaclesNum)
                        if t == 0:
                            # 当t == 0时，说明当前节点为死亡节点
                            DeadNodeNumber = DeadNodeNumber + 1
                            
                        # 测试数据用的 事后需要将其注释掉
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
                                    # 判断当前发送Request信号的节点是否存在与R_list中
                                    R_listExistFlag = False
                                    for j in range(0, len(R_list)):
                                       if NodeList[i] == R_list[j]: 
                                            # 当前发送Request的节点在R_list中
                                            R_listExistFlag = True
                                    if R_listExistFlag is False:
                                        # 当前发送了Request的节点不存在R_list，将其添加入R_list中
                                        R_list.append(NodeList[i])
                            NodeALERTFlag = True
                        # 修改最后一个节点的位置，随之改变距离邻接矩阵
                        # 死亡节点个数不等于剩余节点个数
                        if (i == (len(NodeList) - 1)) and ((len(NodeList) - 1) != DeadNodeNumber):
                            if DebugFlag is True:
                                print "死亡节点个数 DeadNodeNumber =", DeadNodeNumber
                                print "节点个数 len(NodeList) - 1=", len(NodeList) - 1
                            print "change the distance"
                            # 主要是获取各节点之间的距离
                            N_distance_Road_result = F.CreateDistanceNewMatrix(Road_information, N_distance, EdgeLength, NodeXCoordinateNew, NodeYCoordinateNew, NodeListBackup, NodeList, ObstacleCoordinate, ObstaclesNum)
                            N_distance = N_distance_Road_result[0]
                            Road_information = N_distance_Road_result[1] 
                            if DebugFlag is True:
                                print "节点距离修改完毕" 
                    # 临时的回路汇总，为R_Sum和R_New的集合
                    # print "添加相关性能信息~"
                    R_Sum_Temp = []
                    # print "R_Sum =", R_Sum
                    for i in range(0, len(R_Sum)):
                        R_Sum_Temp.append(R_Sum[i])
                    # print "R_Sum_Temp =", R_Sum_Temp
                    # print "R_New =", R_New
                    R_Sum_Temp.append(R_New)
                    # print "R_Sum_Temp =", R_Sum_Temp                        
                    SummaryByTime(El, 'TADP', R_Sum_Temp, Simulation_time, NodeEs, N_distance, NodeXCoordinateNew, NodeYCoordinateNew, ObstacleCoordinate)
                                                        
                    
                    if DebugFlag is True:
                        print "R_New =", R_New
                        print "R_list =", R_list
                        print "R_list_Backup =", R_list_Backup
                        print "NodeList =", NodeList
                        print "Request_list =", Request_list
                    
                    if len(R_list_Backup) == 1 and len(R_list) != 1:
                        if DebugFlag is True:
                            print "重新创建充电回路"
                            print "len(R_New) =", len(R_New)
                        # 重新备份
                        # 将所有充电回路进行汇总
                        # 当充电回路中只有服务站S时，不需要添加到充电回路集合中
                        if len(R_New) != 1:
                            R_Sum.append(R_New)
                            # SummaryByTime(El, 'Second', R_Sum, Simulation_time, NodeEs, N_distance, NodeXCoordinateNew, NodeYCoordinateNew, ObstacleCoordinate)
                        R_list_Backup_flag = True

                    if len(R_list) == 1:
                        # 统计当前节点序号列表中的个数，即未进行操作节点的个数
                        if DebugFlag is True:
                            print "对发送过Request请求的节点已经操作完毕"
                        NodeListNum = len(NodeList)
                        R_list_Backup_flag = True
                        NodeALERTFlag = False
                        R_Sum.append(R_New)
                        # SummaryByTime(El, 'Second', R_Sum, Simulation_time, NodeEs, N_distance, NodeXCoordinateNew, NodeYCoordinateNew, ObstacleCoordinate)
                        # 发送请求的节点已经全部加入充电回路了,退出while循环，重新获取充电请求
                        break
        # 添加构建回路的相关信息
        TourConstructionInformation(El, R_Sum, NodeEs, TADP_DeadNodeNum_data_txt, N_distance, NodeXCoordinateNew, NodeYCoordinateNew, TADP_MCV_Tour_Set_txt, TADP_MCV_Tour_Information_txt, ObstacleCoordinate, 'TADP')
        # 添加几个性能与仿真时间的关系
        result_list = []
        # 仿真时间
        result_list.append(TADP_PerformanceSimulationTime_list)
        # 死亡节点和吞吐量
        result_list.append(TADP_DeadNodeNum_list)
        result_list.append(TADP_Throughput_Num_list)
        # 充电能量和移动能量
        result_list.append(TADP_MCVChargeEs_list)
        result_list.append(TADP_MCVMoveEs_list)
        # 充电时间和移动时间
        result_list.append(TADP_MCVChargeTime_list)
        result_list.append(TADP_MCVMoveTime_list)
        # 实际距离和欧几里得距离
        result_list.append(TADP_MCVRealDistance_list)
        result_list.append(TADP_MCVEuclidDistance_list)


        np.savetxt(TADP_PerformanceSimulation_list_txt, result_list, fmt='%0.2f')
        if DebugFlag is True:
            print "TADP_RequestTime[0] =\n", TADP_RequestTime[0]
            print "TADP_BestNodeTime[0] =\n", TADP_BestNodeTime[0]
            print "TADP_ServiceTime[0] =\n", TADP_ServiceTime[0]
            print "TADP_ResponseTimeAndServiceTimeSimulationTime_list =\n", TADP_ResponseTimeAndServiceTimeSimulationTime_list
            print "TADP_AverageResponseTime_list =\n", TADP_AverageResponseTime_list
            print "TADP_AverageServiceTime_list =\n", TADP_AverageServiceTime_list
        result_list = []
        result_list.append(TADP_ResponseTimeAndServiceTimeSimulationTime_list)
        result_list.append(TADP_AverageResponseTime_list)
        result_list.append(TADP_AverageServiceTime_list)
        
        np.savetxt(TADP_ResponseTimeAndServiceTimeSimulation_list_txt, result_list, fmt='%0.2f')
        # 回路分配算法
        print "TourDistributioning"
        MCVSum = TD.TourDistributionProgramming(TADP_MCV_Tour_Information_txt)
        print "len(MCVSum) =", len(MCVSum)
        print "over TourDistribution"

    if RCSSFlag is True:
        # 统一El的值
        # Eldata = np.loadtxt(El_wBest_Txt)
        # print "Eldata[0] =", Eldata[0]
        # El初始化为0
        El = 0
        Simulation_time = 0.0
        print "RCSS Task"
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
                    Simulation_time = Simulation_time + t
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
                            print "NodeEsBackup =", NodeEsBackup
                            ConsumptionSum = 0.0
                            # 每个节点运动t时间后，统计所消耗的能量，以及成为静态节点的个数
                            for i in range(1, len(NodeList)):
                                # 修改节点能量
                                NodeEs[0][NodeList[i]] = round((NodeEs[0][NodeList[i]] - NodeP[0][NodeList[i]]*t), 2)
                                ConsumptionSum = ConsumptionSum + NodeP[0][NodeList[i]]*t 
                                if DebugFlag is True:
                                    print "预计消耗的能量 NodeP[0][NodeList[i]]*t=", NodeP[0][NodeList[i]]*t, 'j'
                                    print "能量阈值下限El=", El, 'j'
                                
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
                                print "静态节点个数StaticNodeNum=", StaticNodeNum, '个' 
                                print "节点运行总能量NodeOperateEs=", NodeOperateEs, 'j'
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
                        np.savetxt(RCSS_AECR_Txt, ElNodeOperateEsAllList,fmt='%0.5f')
                        np.savetxt(RCSS_AFP_Txt, ElStaticNodeAllList, fmt='%0.5f')
                        
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
                        # w > 1 说明侧重点不一样而已
                        w = 1.0
                        wBest = 0.0
                        ElBefore = 0.0
                        # 可以重复很多次的，自己设定就好
                        while wBest == 0.0:
                            ElNew = -((A1*B1+w*A2*B2)/(np.power(A1, 2)+ w*np.power(A2, 2)))
                            wList.append(w)
                            ElList.append(ElNew)
                            if np.abs(ElBefore - ElNew) < WAccuracy:
                                # 表明前后两个El值非常接近
                                ElFinal = ElNew
                                wBest = w 
                                break
                            ElBefore = ElNew
                            # 每迭代一次，w + 0.1
                            w = w + 0.1
                        f = np.polyfit(wList,ElList,2)
                        p = np.poly1d(f)
                        if DebugFlag is True:
                            print "f =", f
                            print "p =", p
                        if DebugFlag is True:
                            print "wBest =", wBest
                            print "阈值上限为 Et=", Et
                            print "在这次运行过程中比较适合的阈值下限ElFinal=", ElFinal
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
                            El = 0
                            if DebugFlag is True:
                                print "El修改为El =", El
                        Ellist = []
                        Ellist.append(El)
                        Ellist.append(wBest)
                        np.savetxt(RCSS_El_wBest_Txt, Ellist,fmt='%0.2f')
                    # 每次能量都需要恢复
                    for i in range(0, len(NodeList)):
                        NodeEs[0][NodeList[i]] = NodeEsBackup[i]
                    if DebugFlag is True:
                        print "NodeEs[0] =", NodeEs[0]
                    # 以上部分是为了计算得到能量阈值下限El
                    # 更新节点剩余的能量  # 不仅仅是考虑边界，还得考虑运动过程中，遇到障碍应该避开
                    DeadNodeNumber = 0
                    for i in range(1, len(NodeList)):
                        # 修改节点能量
                        t = ChangeCoordinate(i, El, Et, RCSS_RequestTime, Simulation_time, NodeList, NodeEs, NodeP, t_sum, NodeEsBackup, NodeMoveTime, NodeXCoordinateNew, NodeYCoordinateNew, V, Alpha, AlphaValue, EdgeLength, ObstacleCoordinate, ObstaclesNum)
                        if t == 0:
                            # t == 0 表示该节点为死亡节点
                            DeadNodeNumber = DeadNodeNumber + 1
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
                            origin_path =  'RCSSAECR&AFP.png'  
                            All_path = os.path.join(RCSS_path, origin_path)
                            plt.savefig(All_path)
                            
                            plt.show()
                            
                            plt.plot(wList, ElList, 'b:*', label = 'El')
                            plt.xlabel('w')
                            plt.ylabel('El')
                            plt.legend(loc=2) #指定legend的位置右下角
                            plt.grid()
                            plt.title('polyfitting')
                            # 保存生成的图片
                            origin_path =  'RCSSAEl.png'  
                            All_path = os.path.join(RCSS_path, origin_path)
                            plt.savefig(All_path)
                            plt.show()
                            # plt,show()
                        if (i == (len(NodeList) - 1)) and ((len(NodeList) - 1) != DeadNodeNumber):
                            if DebugFlag is True:
                                print "死亡节点个数 DeadNodeNumber =", DeadNodeNumber
                                print "节点个数 len(NodeList)-1 =", len(NodeList) - 1
                            print "coordinate have been changed, change distance"
                            N_distance_Road_result = F.CreateDistanceNewMatrix(Road_information, N_distance, EdgeLength, NodeXCoordinateNew, NodeYCoordinateNew, NodeListBackup, NodeList, ObstacleCoordinate, ObstaclesNum)
                            N_distance = N_distance_Road_result[0]
                            Road_information = N_distance_Road_result[1]
                           
                # 就在这个环节，将一部分节点（R_list中的节点）从原NodeList中删掉
                elif NodeALERTFlag is True:
                    if DebugFlag is True:
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
                    # if DebugFlag is True:
                    print "R_list =", R_list
                    
                    # 选择最佳下一服务点的方法为：综合考虑剩余能量和与MCV之间的距离
                    # 计算得到P，以P最小为下一最佳服务节点
                    BestNode = 0
            
                    # a 为节点功耗倒数所占比重
                    # b 为S到各电单车距离所占比重
                    # P 为综合节点功耗倒数和S到各电单车距离所占比重考虑的最终值
                    # 尽可能把节点功耗大(节点功耗倒数小)的和与MCV距离短的节点优先选择为下一服务节点
                    NodeReciprocalP_sort = []
                    N_distance_sort = []
                    for i in range(1, len(R_list_Backup)):
                        # 获取节点及其功耗的倒数，组成(NodeNum, ReciprocalP[0][NodeNum])
                        # 这样操作存在，剩余能量的排名的变化同步节点序号的变化
                        Node_number_ReciprocalP = []
                        Node_number_ReciprocalP.append(R_list_Backup[i])
                        # 获取功耗的倒数【因为需要从将功耗从大到小，为了便于我的计算，用了
                        # 功耗的倒数，排序时，从小达到大排序】
                        Node_number_ReciprocalP.append(round((1.0/NodeP[0][R_list_Backup[i]]),4))
                        NodeReciprocalP_sort.append(Node_number_ReciprocalP)
                        # 获取节点及其与MCV的距离，组成(NodeNum, N_distance[NodeNum(MCV)][NodeNum]
                        # 这样操作存在，与MCV的距离排名的变化同步节点序号的变化
                        Node_number_distance= []
                        Node_number_distance.append(R_list_Backup[i])
                        # 获取MCV到当前节点的距离
                        Node_number_distance.append(round(N_distance[R_New[len(R_New) - 1]][R_list_Backup[i]], 2))
                        N_distance_sort.append(Node_number_distance)
                    print "NodeReciprocalP_sort =", NodeReciprocalP_sort
                    print "before N_distance_sort =", N_distance_sort
                    # 已经将剩余能量和与MCV的距离按从小到大排序
                    select_sort_Numsort(NodeReciprocalP_sort)
                    select_sort_Numsort(N_distance_sort)
                    print "after NodeReciprocalP_sort =", NodeReciprocalP_sort
                    print "after N_distance_sort =", N_distance_sort

                    a = 0.2
                    b = 1.0 - a
                    # 只考虑了当前一次，所以得看看如何把所有的点考虑进去，每个点都要考虑到
                    # 只考虑当前所剩下的节点，不包括S点（N_i[0]）
                    # 假设开始时的P非常非常的大
                    P = []
                    # 这里应该计算剩余所有节点的P值
                    for i in range(1 , len(R_list_Backup)):
                        # 临时列表
                        P_Node_number = []
                        # 添加节点序号
                        P_Node_number.append(R_list_Backup[i])
                        ReciprocalP_Rank = 0
                        for l in range(1, len(R_list_Backup)):
                            if R_list_Backup[i] == NodeReciprocalP_sort[l -1][0]:
                                # 当前剩余能量节点的排名
                                ReciprocalP_Rank = l
                                break
                        Distance_Rank = 0
                        for m in range(1, len(R_list_Backup)):
                            if R_list_Backup[i] == N_distance_sort[m - 1][0]:
                                # 当前 节点距离S的排名
                                Distance_Rank = m
                                break

                        P_value = a*ReciprocalP_Rank + b*Distance_Rank
                        
                        P_Node_number.append(round(P_value, 2))
                        # 添加节点的排名的比重
                        P.append(P_Node_number)
            
                    print "before P =", P
                    select_sort_Numsort(P)
                    print "after P =", P
                    # 本次寻找最好的节点为 BestNode
                    # 该点不再运动
                    BestNode = P[0][0]
                    print "BestNode =", BestNode
                    # print "delay(10)"
                    # time.sleep(10)
                    # 将最好的节点加入构建的充电回路中
                    R_New.append(BestNode)
                    # 当该节点被选为最佳下一服务点时，将当前时间进行统计
                    # R_list_Backup中的值每次都得进行删除操作，主要是怕不能依次对每个节点进行查询操作
                    R_list_Backup_FirstValue = R_list_Backup[0]
                    R_list_Backup[0] = 0
                    R_list_Backup.remove(BestNode)
                    R_list_Backup[0] = R_list_Backup_FirstValue
                    if DebugFlag is True:
                        print "R_list_Backup =", R_list_Backup
                    # 当该节点被选为最佳下一服务点时，将当前时间进行统计
                    RCSS_BestNodeTime[0][BestNode] = Simulation_time
                    if DebugFlag is True:
                        print "RCSS_BestNodeTime[0][", BestNode, "] =", RCSS_BestNodeTime[0][BestNode] 
                
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
                    Simulation_time = Simulation_time + t
                    RCSS_ServiceTime[0][BestNode] = t
                    if DebugFlag is True:
                        print "RCSS_ServiceTime[0][", BestNode, "] =", RCSS_ServiceTime[0][BestNode] 
                        
                        # 添加平均响应时间和平均服务时间
                        print "NodeListBackup =", NodeListBackup
                        print "len(NodeListBackup) =", len(NodeListBackup)
                    # 被服务过的节点个数的初始化
                    ServiceNode = 0 
                    # 每个节点响应时间的总和
                    ResponseNodeTime = 0.0
                    # 每个节点服务时间求总和
                    ServiceNodeTime = 0.0
                    
                    for m in range(1, len(NodeListBackup)):
                        # 说明这个节点已经被添加到回路了，或者说已经被操作过的
                        if RCSS_ServiceTime[0][NodeListBackup[m]] != 0:
                           # 当前选中的节点的相关时间的统计
                           # 响应时间
                           RCSS_RequestTime[0][NodeListBackup[m]]
                           # 选择下一最佳服务节点的时间
                           RCSS_BestNodeTime[0][NodeListBackup[m]]
                           
                           # 已经被操作过（被添加回路）的节点个数的统计
                           ServiceNode = ServiceNode + 1 
                           # 节点响应时间求总和
                           ResponseNodeTime = ResponseNodeTime + (RCSS_BestNodeTime[0][NodeListBackup[m]] - RCSS_RequestTime[0][NodeListBackup[m]])
                           # 节点服务时间求总和
                           ServiceNodeTime = ServiceNodeTime + RCSS_ServiceTime[0][NodeListBackup[m]]
                    
                    # 添加平均响应时间
                    RCSS_AverageResponseTime_list.append(round(ResponseNodeTime/ServiceNode, 2))
                    # 添加平均服务时间
                    RCSS_AverageServiceTime_list.append(round(ServiceNodeTime/ServiceNode, 2))
                    
                    # 添加当前的仿真时间
                    RCSS_ResponseTimeAndServiceTimeSimulationTime_list.append(Simulation_time)
                    
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
                    DeadNodeNumber = 0
                    for i in range(1, len(NodeList)):
                        # 修改节点能量
                        t = ChangeCoordinate(i, El, Et, RCSS_RequestTime, Simulation_time, NodeList, NodeEs, NodeP, t_sum, NodeEsBackup, NodeMoveTime, NodeXCoordinateNew, NodeYCoordinateNew, V, Alpha, AlphaValue, EdgeLength, ObstacleCoordinate, ObstaclesNum)
                        if t == 0:
                            # 当t == 0时，说明当前节点为死亡节点
                            DeadNodeNumber = DeadNodeNumber + 1
                            
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
                                    # 判断当前发送Request信号的节点是否存在与R_list中
                                    R_listExistFlag = False
                                    for j in range(0, len(R_list)):
                                       if NodeList[i] == R_list[j]: 
                                            # 当前发送Request的节点在R_list中
                                            R_listExistFlag = True
                                    if R_listExistFlag is False:
                                        # 当前发送了Request的节点不存在R_list，将其添加入R_list中
                                        R_list.append(NodeList[i])
                            NodeALERTFlag = True
                        # 修改最后一个节点的位置，随之改变距离邻接矩阵
                        # 死亡节点个数不等于剩余节点个数
                        if (i == (len(NodeList) - 1)) and ((len(NodeList) - 1) != DeadNodeNumber):
                            if DebugFlag is True:
                                print "死亡节点个数 DeadNodeNumber =", DeadNodeNumber
                                print "节点个数 len(NodeList) - 1=", len(NodeList) - 1
                            print "change the distance"
                            # 主要是获取各节点之间的距离
                            N_distance_Road_result = F.CreateDistanceNewMatrix(Road_information, N_distance, EdgeLength, NodeXCoordinateNew, NodeYCoordinateNew, NodeListBackup, NodeList, ObstacleCoordinate, ObstaclesNum)
                            N_distance = N_distance_Road_result[0]
                            Road_information = N_distance_Road_result[1] 
                            if DebugFlag is True:
                                print "节点距离修改完毕"  
                    # 临时的回路汇总，为R_Sum和R_New的集合
                    # print "添加相关性能信息~"
                    R_Sum_Temp = []
                    # print "R_Sum =", R_Sum
                    for i in range(0, len(R_Sum)):
                        R_Sum_Temp.append(R_Sum[i])
                    # print "R_Sum_Temp =", R_Sum_Temp
                    # print "R_New =", R_New
                    R_Sum_Temp.append(R_New)
                    # print "R_Sum_Temp =", R_Sum_Temp                        
                    SummaryByTime(El, 'RCSS', R_Sum_Temp, Simulation_time, NodeEs, N_distance, NodeXCoordinateNew, NodeYCoordinateNew, ObstacleCoordinate)
                                                        
                    
                    if DebugFlag is True:
                        print "R_New =", R_New
                        print "R_list =", R_list
                        print "R_list_Backup =", R_list_Backup
                        print "NodeList =", NodeList
                        print "Request_list =", Request_list
                    
                    if len(R_list_Backup) == 1 and len(R_list) != 1:
                        if DebugFlag is True:
                            print "重新创建充电回路"
                            print "len(R_New) =", len(R_New)
                        # 重新备份
                        # 将所有充电回路进行汇总
                        # 当充电回路中只有服务站S时，不需要添加到充电回路集合中
                        if len(R_New) != 1:
                            R_Sum.append(R_New)
                            # SummaryByTime(El, 'Second', R_Sum, Simulation_time, NodeEs, N_distance, NodeXCoordinateNew, NodeYCoordinateNew, ObstacleCoordinate)
                        R_list_Backup_flag = True

                    if len(R_list) == 1:
                        # 统计当前节点序号列表中的个数，即未进行操作节点的个数
                        if DebugFlag is True:
                            print "对发送过Request请求的节点已经操作完毕"
                        NodeListNum = len(NodeList)
                        R_list_Backup_flag = True
                        NodeALERTFlag = False
                        R_Sum.append(R_New)
                        # SummaryByTime(El, 'Second', R_Sum, Simulation_time, NodeEs, N_distance, NodeXCoordinateNew, NodeYCoordinateNew, ObstacleCoordinate)
                        # 发送请求的节点已经全部加入充电回路了,退出while循环，重新获取充电请求
                        break
        # 添加构建回路的相关信息
        TourConstructionInformation(El, R_Sum, NodeEs, RCSS_DeadNodeNum_data_txt, N_distance, NodeXCoordinateNew, NodeYCoordinateNew, RCSS_MCV_Tour_Set_txt, RCSS_MCV_Tour_Information_txt, ObstacleCoordinate, 'RCSS')
        # 添加几个性能与仿真时间的关系
        result_list = []
        # 仿真时间
        result_list.append(RCSS_PerformanceSimulationTime_list)
        # 死亡节点和吞吐量
        result_list.append(RCSS_DeadNodeNum_list)
        result_list.append(RCSS_Throughput_Num_list)
        # 充电能量和移动能量
        result_list.append(RCSS_MCVChargeEs_list)
        result_list.append(RCSS_MCVMoveEs_list)
        # 充电时间和移动时间
        result_list.append(RCSS_MCVChargeTime_list)
        result_list.append(RCSS_MCVMoveTime_list)
        # 实际距离和欧几里得距离
        result_list.append(RCSS_MCVRealDistance_list)
        result_list.append(RCSS_MCVEuclidDistance_list)


        np.savetxt(RCSS_PerformanceSimulation_list_txt, result_list, fmt='%0.2f')
        # 回路分配算法
        print "TourDistributioning"
        MCVSum = TD.TourDistributionProgramming(RCSS_MCV_Tour_Information_txt)
        print "len(MCVSum) =", len(MCVSum)
        print "over TourDistribution"

        if DebugFlag is True:
            print "RCSS_RequestTime[0] =\n", RCSS_RequestTime[0]
            print "RCSS_BestNodeTime[0] =\n", RCSS_BestNodeTime[0]
            print "RCSS_ServiceTime[0] =\n", RCSS_ServiceTime[0]
            print "RCSS_ResponseTimeAndServiceTimeSimulationTime_list =\n", RCSS_ResponseTimeAndServiceTimeSimulationTime_list
            print "RCSS_AverageResponseTime_list =\n", RCSS_AverageResponseTime_list
            print "RCSS_AverageServiceTime_list =\n", RCSS_AverageServiceTime_list
        result_list = []
        result_list.append(RCSS_ResponseTimeAndServiceTimeSimulationTime_list)
        result_list.append(RCSS_AverageResponseTime_list)
        result_list.append(RCSS_AverageServiceTime_list)
        
        np.savetxt(RCSS_ResponseTimeAndServiceTimeSimulation_list_txt, result_list, fmt='%0.2f')      

    
    # 导入测试数据
    # data = np.loadtxt(First_ResponseTimeAndServiceTimeSimulation_list_txt)
    # data1 = np.loadtxt(FirstCompare_ResponseTimeAndServiceTimeSimulation_list_txt)
    data = np.loadtxt(Second_ResponseTimeAndServiceTimeSimulation_list_txt)
    # data3 = np.loadtxt(SecondCompare_ResponseTimeAndServiceTimeSimulation_list_txt)
    data1 = np.loadtxt(NJNP_ResponseTimeAndServiceTimeSimulation_list_txt)
    data2 = np.loadtxt(TADP_ResponseTimeAndServiceTimeSimulation_list_txt)
    data3 = np.loadtxt(RCSS_ResponseTimeAndServiceTimeSimulation_list_txt)
    # print "data =", data

    # 平均响应时间和平均服务时间两个性能图表刻度
    kedu_new = 500
    # data的解释，例如：
    
    
    # SimulationTime = data[0]
    # RespondTime = data[1]
    # ServiceTime = data[2]
    
    '''
    SimulationTimeMinx = min(min(data[0]), min(data1[0]), min(data2[0]), min(data3[0]), min(data4[0]), min(data5[0]), min(data6[0]))
    print "SimulationTimeMinx =", SimulationTimeMinx

    SimulationTimeMaxx = max(max(data[0]), max(data1[0]), max(data2[0]), max(data3[0]), max(data4[0]), max(data5[0]), max(data6[0]))
    print "SimulationTimeMaxx =", SimulationTimeMaxx

    RespondTimeMiny = min(min(data[1]), min(data1[1]), min(data2[1]), min(data3[1]), min(data4[1]), min(data5[1]), min(data6[1]))
    print "RespondTimeMiny =", RespondTimeMiny

    RespondTimeMaxy = max(max(data[1]), max(data1[1]), max(data2[1]), max(data3[1]), max(data4[1]), max(data5[1]), max(data6[1]))
    print "RespondTimeMaxy =", RespondTimeMaxy

    ServiceTimeMiny = min(min(data[2]), min(data1[2]), min(data2[2]), min(data3[2]), min(data4[2]), min(data5[2]), min(data6[2]))
    print "ServiceTimeMiny =", ServiceTimeMiny

    ServiceTimeMaxy = max(max(data[2]), max(data1[2]), max(data2[2]), max(data3[2]), max(data4[2]), max(data5[2]), max(data6[2]))
    print "ServiceTimeMaxy =", ServiceTimeMaxy
    '''
    
    # 主要用于画图中进行操作，线条的颜色
    # LineColor =['b', 'g', 'r', 'c', 'm', 'y', 'k']
    # LineColor =['b']
    # 线条的风格
    # LineStyle = ['-', '--', '-.', ':']
    # 线条的标志
    # LineLogo = ['.', 'o', 'v', '^', '>', '<', '1', '2', '3', '4', 's', 'p', '*']
    
    ax = plt.gca()
    # 图片坐标刻度设置
    ax.xaxis.set_major_locator(MultipleLocator(kedu_new))
    # ax.yaxis.set_major_locator(MultipleLocator(200))
    plt.plot(data[0], data[1],'c-s',label = 'RespondTime2')
    plt.plot(data1[0], data1[1],'r-.s',label = 'RespondTimeNJNP')
    plt.plot(data2[0], data2[1], 'g--^',label = 'RespondTimeTADP')
    plt.plot(data3[0], data3[1],'b:p',label = 'RespondTimeRCSS')
    # plt.plot(data4[0], data4[1],'m-.v',label = 'RespondTimeNJNP')
    # plt.plot(data5[0], data5[1],'y:v',label = 'RespondTimeTADP')
    # plt.plot(data6[0], data6[1],'k:.',label = 'RespondTimeRCSS')

    plt.legend(loc='lower right', edgecolor='black')
    plt.xlabel('Simulation_time')
    plt.ylabel('RespondTime')
    origin_path = 'RespondTimeAndSimulation_time.png'  
    All_path = os.path.join(childern_result_name, origin_path)
    plt.grid()
    # plt.xlim([(SimulationTimeMinx%100)*100 - kedu_new, SimulationTimeMaxx + 200]) #设置绘图X边界                                                                                                   
    # plt.ylim([RespondTimeMiny , RespondTimeMaxy + 50]) #设置绘图Y边界
    # plt.axis([(SimulationTimeMinx%100)*100 - kedu_new, SimulationTimeMaxx + 200, 0, RespondTimeMaxy + 50])
    plt.savefig(All_path)
    plt.show()
    
    ax = plt.gca()
    # 图片坐标刻度设置
    ax.xaxis.set_major_locator(MultipleLocator(kedu_new))
    # ax.yaxis.set_major_locator(MultipleLocator(200))

    plt.plot(data[0], data[2], 'c-s',label = 'ServiceTime2')
    plt.plot(data1[0], data1[2], 'r-.s',label = 'ServiceTimeNJNP')
    plt.plot(data2[0], data2[2],'g--^',label = 'ServiceTimeTADP')
    plt.plot(data3[0], data3[2], 'b:p',label = 'ServiceTimeRCSS')
    # plt.plot(data4[0], data4[2],'m-.v',label = 'ServiceTimeNJNP')
    # plt.plot(data5[0], data5[2],'y:v',label = 'ServiceTimeTADP')
    # plt.plot(data6[0], data6[2],'k:.',label = 'ServiceTimeRCSS')

    plt.legend(loc='lower right', edgecolor='black')
    plt.xlabel('Simulation_time')
    plt.ylabel('ServiceTime')
    origin_path = 'ServiceTimeAndSimulation_time.png'  
    All_path = os.path.join(childern_result_name, origin_path)
    plt.grid()
    # plt.xlim([(SimulationTimeMinx%100)*100 - kedu_new, SimulationTimeMaxx + 200]) #设置绘图X边界                                                                                                   
    # plt.ylim([ServiceTimeMiny, ServiceTimeMaxy + 50]) #设置绘图Y边界
    # plt.axis([(SimulationTimeMinx%100)*100 - kedu_new, SimulationTimeMaxx + 200, 0, ServiceTimeMaxy + 50])
    plt.savefig(All_path)
    plt.show()

    print "程序即将结束"
    ProgrammingEndTime = time.time()    
    print "程序总共耗时 ", (ProgrammingEndTime- ProgrammingStartTime), 's'
          