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

# 几种出发机制运行的标志  True表示运行， False表示不运行
# 第一种出发机制对比实验
FirstCompareFlag = True
# 第二种出发机制
SecondFlag = True
# 第二种出发机制对比实验
SecondCompareFlag = True
# NJNP方法实验
NJNPFlag = True
# TADP算法实验
TADPFlag = False
# RCSS 算法实验
RCSSFlag = False

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
MinAngle = 0   # 对应最小角度为90°或者270°，如果要修改角度阈值，最好选择修改MinAngle

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
childern_result_name = 'Em_' + str(int(Em)) + '_vm_' +str(int(Vm)) + '_qm_' + str(int(Qm)) + '_qc_' + str(int(Qc))
childern_result_name = os.path.join(result_name, childern_result_name)
isExist = os.path.exists(childern_result_name)
if not isExist:
    print "不存在该路径，创建对应路径"
    os.makedirs(childern_result_name)
# 第一种出发机制对比实验【固定缓冲池】
FirstCompare_path = os.path.join(childern_result_name, "FirstCompare")
isExist = os.path.exists(FirstCompare_path)
if not isExist:
    print "不存在该路径，创建对应路径"
    os.makedirs(FirstCompare_path)
# 第二种出发机制实验【双阈值】
Second_path = os.path.join(childern_result_name, "Second")
isExist = os.path.exists(Second_path)
if not isExist:
    print "不存在该路径，创建对应路径"
    os.makedirs(Second_path)
# 第二种出发机制对比实验【单阈值】
SecondCompare_path = os.path.join(childern_result_name, "SecondCompare")
isExist = os.path.exists(SecondCompare_path)
if not isExist:
    print "不存在该路径，创建对应路径"
    os.makedirs(SecondCompare_path)
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

# 中间数据文件
Temp_MCV_Tour_Information_txt = os.path.join(FirstCompare_path, 'Temp_MCV_Tour_Information.txt')

# 第一种出发机制的生成的一些txt文档   
FirstCompare_MCV_Tour_Set_txt = os.path.join(FirstCompare_path, 'FirstCompare_MCV_Tour_Set.txt')
FirstCompare_MCV_Tour_Information_txt = os.path.join(FirstCompare_path, 'FirstCompare_MCV_Tour_Information.txt')
FirstCompare_DeadNodeNum_data_txt  = os.path.join(FirstCompare_path, 'FirstCompare_DeadNodeNum_data.txt')
FirstCompare_PerformanceSimulation_list_txt  = os.path.join(FirstCompare_path, 'FirstCompare_PerformanceSimulation_list.txt')
FirstCompare_ResponseTimeAndServiceTimeSimulation_list_txt = os.path.join(FirstCompare_path, 'FirstCompare_ResponseTimeAndServiceTimeSimulation_list.txt')

# 第一种出发机制对比实验
# 仿真时间统计

FirstCompare_PerformanceSimulationTime_list = []
# 死亡节点和吞吐量的统计
FirstCompare_DeadNodeNum_list = []
FirstCompare_Throughput_Num_list = []
# 充电能量和移动能量的统计
FirstCompare_MCVChargeEs_list = []
FirstCompare_MCVMoveEs_list = []
# 充电时间和移动时间的统计
FirstCompare_MCVChargeTime_list = []
FirstCompare_MCVMoveTime_list = []
# 实际距离和欧几里得距离的统计
FirstCompare_MCVRealDistance_list = []
FirstCompare_MCVEuclidDistance_list = []

# 平均响应时间：从发送Request信息到被确认为下一服务点时，中间间隔时间
# t = t2(被确认为下一服务节点时) - t1(发送Request信息)
# 定义数组用于保存当节点发送Request时的时间
FirstCompare_RequestTime = np.zeros((1, NodeNum + 1), dtype = np.float)
# 定义数组用于保存当前节点被选为下一服务节点(最佳节点)的时间
FirstCompare_BestNodeTime = np.zeros((1, NodeNum + 1), dtype = np.float)
# 平均响应时间序列
FirstCompare_AverageResponseTime_list = []

# 平均服务时间：从被确认为下一服务点时，到被充电完成，中间间隔时间
# t = MCV移动时间(从当前服务节点出发) + MCV充电时间(下一服务节点)
# 定义数组，用于保存服务时间
FirstCompare_ServiceTime = np.zeros((1, NodeNum + 1), dtype = np.float)
# 平均服务时间序列
FirstCompare_AverageServiceTime_list = []

# 平均响应时间和平均服务时间的仿真时间 
FirstCompare_ResponseTimeAndServiceTimeSimulationTime_list = []

# 第二种出发机制的生成的一些txt文档   
Second_MCV_Tour_Set_txt = os.path.join(Second_path, 'Second_MCV_Tour_Set.txt')
SecondCompare_MCV_Tour_Set_txt = os.path.join(SecondCompare_path, 'SecondCompare_MCV_Tour_Set.txt')

Second_MCV_Tour_Information_txt = os.path.join(Second_path, 'Second_MCV_Tour_Information.txt')
SecondCompare_MCV_Tour_Information_txt = os.path.join(SecondCompare_path, 'SecondCompare_MCV_Tour_Information.txt')

AECR_Txt = os.path.join(Second_path, 'SecondAECRData.txt')
AFP_Txt = os.path.join(Second_path, 'SecondAFPData.txt')

El_wBest_Txt = os.path.join(Second_path, 'SecondEl_wBestData.txt')

Second_Dead_NodeNum_txt = os.path.join(Second_path, 'Second_Dead_NodeNum.txt')
SecondCompare_DeadNodeNum_txt = os.path.join(SecondCompare_path, 'SecondCompare_DeadNodeNum.txt')

Second_PerformanceSimulation_list_txt  = os.path.join(Second_path, 'Second_PerformanceSimulation_list.txt')
SecondCompare_PerformanceSimulation_list_txt  = os.path.join(SecondCompare_path, 'SecondCompare_PerformanceSimulation_list.txt')

Second_ResponseTimeAndServiceTimeSimulation_list_txt = os.path.join(Second_path, 'Second_ResponseTimeAndServiceTimeSimulation_list.txt')
SecondCompare_ResponseTimeAndServiceTimeSimulation_list_txt = os.path.join(SecondCompare_path, 'SecondCompare_ResponseTimeAndServiceTimeSimulation_list.txt')

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

# 初始化第二出发机制对比实验

# 仿真时间统计
SecondCompare_PerformanceSimulationTime_list = []
# 死亡节点和吞吐量的统计
SecondCompare_DeadNodeNum_list = []
SecondCompare_Throughput_Num_list = []
# 充电能量和移动能量的统计
SecondCompare_MCVChargeEs_list = []
SecondCompare_MCVMoveEs_list = []
# 充电时间和移动时间的统计
SecondCompare_MCVChargeTime_list = []
SecondCompare_MCVMoveTime_list = []
# 实际距离和欧几里得距离的统计
SecondCompare_MCVRealDistance_list = []
SecondCompare_MCVEuclidDistance_list = []


# 平均响应时间：从发送Request信息到被确认为下一服务点时，中间间隔时间
# t = t2(被确认为下一服务节点时) - t1(发送Request信息)
# 定义数组用于保存当节点发送Request时的时间
SecondCompare_RequestTime = np.zeros((1, NodeNum + 1), dtype = np.float)
# 定义数组用于保存当前节点被选为下一服务节点(最佳节点)的时间
SecondCompare_BestNodeTime = np.zeros((1, NodeNum + 1), dtype = np.float)
# 平均响应时间序列
SecondCompare_AverageResponseTime_list = []

# 平均服务时间：从被确认为下一服务点时，到被充电完成，中间间隔时间
# t = MCV移动时间(从当前服务节点出发) + MCV充电时间(下一服务节点)
# 定义数组，用于保存服务时间
SecondCompare_ServiceTime = np.zeros((1, NodeNum + 1), dtype = np.float)
# 平均服务时间序列
SecondCompare_AverageServiceTime_list = []

# 平均响应时间和平均服务时间的仿真时间 
SecondCompare_ResponseTimeAndServiceTimeSimulationTime_list = []

# RCSS对比实验
# RCSS出发机制的生成的一些txt文档   
RCSS_MCV_Tour_Set_txt = os.path.join(RCSS_path, 'RCSS_MCV_Tour_Set.txt')
RCSS_MCV_Tour_Information_txt = os.path.join(RCSS_path, 'RCSS_MCV_Tour_Information.txt')
RCSS_DeadNodeNum_data_txt  = os.path.join(RCSS_path, 'RCSS_DeadNodeNum_data.txt')
RCSS_PerformanceSimulation_list_txt  = os.path.join(RCSS_path, 'RCSS_PerformanceSimulation_list.txt')
RCSS_ResponseTimeAndServiceTimeSimulation_list_txt = os.path.join(RCSS_path, 'RCSS_ResponseTimeAndServiceTimeSimulation_list.txt')

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

# 输入要操作的文件的路径【一般文件是txt文件】
# 函数功能：将输入的充电回路，分配给最少的MCV
# 函数输入：保存着充电回路数据的txt文档
# 函数输出：MCV数量，已经每个MCV为哪些充电回路工作的详细信息
def TourDistributionProgramming(MCV_Tour_Information_txt):
    # 充电回路周期T = MCV给服务点的充电时间 + MCV移动到服务点的时间
    # 充电时间Tj-working: MCV给服务点的充电时间
    # 导入测试数据
    TourSum = np.loadtxt(MCV_Tour_Information_txt, dtype = np.float)
    if DebugFlag is True:
        print "TourSum =\n", TourSum
        print "len(TourSum) =", len(TourSum)
    # 定义一个临时变量
    array = TourSum
    # 对导出的回路数据，按照充电回路最大周期进行从小到大排序
    for position_i in range(0, len(array) - 1):
        min_position = position_i
        for position_j in range(position_i + 1, len(array)):
            # 充电回路周期T比较大小
            if (array[min_position][4] + array[min_position][5]) > (array[position_j][4] + array[position_j][5]):
                min_position = position_j
        tmp = array[min_position].copy()
        array[min_position] = array[position_i]
        array[position_i] = tmp
    TourSum = array
    if DebugFlag is True: 
        print "TourSum =\n", TourSum
        print "len(TourSum) =", len(TourSum)

    # 当作回路是否已经被MCV充电的标志，标志位为1表示被充电了，标志位为0则表示未被充电
    Tourlabel = np.ones((1, len(TourSum) + 1), dtype = np.int)
    for i in range(0, len(TourSum)):
        # 充电回路未被覆盖，设置其标志位为0
        Tourlabel[0][i] = 0
    # 初始化充电回路分配给MCV的集合
    MCVSum = []
    
    for i in range(0, len(TourSum)):
        # 保存可以放到一个MCV中的充电回路
        MCV = []
        if Tourlabel[0][i] == 0:
            # 将当前充电回路分配给当前的MCV
            MCV.append(TourSum[i])
            if DebugFlag is True:
                print "MCV =", MCV 
            # 将Gm的元充电周期设置为第一个分配的回路的充电周期最大值
            Gm = (TourSum[i][4]+TourSum[i][5])
            if DebugFlag is True:
                print "before Gm =", Gm
            # 说明强调Gm需要为整数，并且要向上取整,周期至少要大于等于原来的最大周期Gm
            if (Gm - int(Gm)) != 0.0:
                Gm = (int(Gm)+1)
            else:
                Gm = int(Gm)
            if DebugFlag is True:
                print "after Gm =", Gm
            # 计算当前分配回路后MCV的负载率
            nl = TourSum[i][4]/Gm
            if DebugFlag is True:
                print "first Tour nl =", nl
            # 更改TourSum[i]标签的属性
            Tourlabel[0][i] = 1
            for k in range(i + 1, len(TourSum)):
                # 查询充电回路是否被分配给MCV
                if Tourlabel[0][k] == 0:
                    # 存在l的标志，existlFlag = False表示不存在
                    existlFlag = False
                    if DebugFlag is True:
                        print "TourSum[", k, "][4] =", TourSum[k][4]
                        print "TourSum[", k, "][4] + TourSum[", k, "][5] =", TourSum[k][4] + TourSum[k][5]
                    # 深复制充电周期的最小值
                    PeriodDonw = TourSum[k][4].copy()
                    # 深复制充电周期的最大值
                    PeriodUp = (TourSum[k][4] + TourSum[k][5]).copy()
                    # 周期上下限取整
                    if (PeriodDonw - int(PeriodDonw)) != 0.0:
                        # 周期下限为小数时，向上取整
                        PeriodDonw = int(PeriodDonw) + 1
                    else:
                        # 周期下限为浮点数，取整即可
                        PeriodDonw = int(PeriodDonw)
                    # 周期上限，取整即可
                    PeriodUp = int(PeriodUp)
                    if DebugFlag is True:
                        print "PeriodDonw =", PeriodDonw
                        print "PeriodUp =", PeriodUp
                    # 从当前充电回路周期最大最小值中，选出一个值为Gm的倍数
                    for l in range(PeriodDonw, (PeriodUp + 1)):
                        # 判断l%GM能得到整数
                        if l%Gm == 0:
                            # existlFlag = True 表示存在l在周期上下限内，并且能整除Gm
                            if DebugFlag is True:
                                print "l =", l
                                print "Gm = ", Gm
                                print "l","%","Gm =", l%Gm
                            existlFlag = True
                            break

                    # 存在整数l能够保证lxGm在Pk的充电周期的上下限内
                    if existlFlag == True:
                        W = nl + TourSum[k][4]/Gm
                        if DebugFlag is True:
                            print "W =", W
                        # 满足负载率小于等于1
                        if W <= 1:
                            # 负载率相加
                            nl = nl + W
                            if DebugFlag is True:
                                print "nl =", nl
                            # 将当前回路分配给当前的MCV
                            MCV.append(TourSum[k])
                            if DebugFlag is True:
                                print "MCV =", MCV
                            # 更改当前充电回路的标签
                            Tourlabel[0][k] = 1
            # 将当前的MCV中充电回路的集合添加到MCV集合中
            MCVSum.append(MCV)
    if DebugFlag is True:
        print "MCV_Tour_Information_txt =", MCV_Tour_Information_txt
    SplitPath = os.path.split(MCV_Tour_Information_txt)
    # SplitPath[0] = 当前目录下的文件夹路径
    if DebugFlag is True:
        print "SplitPath[0] =", SplitPath[0]
    # SplitPath[1] = 当前文件的名称
    if DebugFlag is True:
        print "SplitPath[1] =", SplitPath[1]
    fileName = 'New_' + SplitPath[1]
    filePath = os.path.join(SplitPath[0], fileName)
    if DebugFlag is True:
        print "filePath =", filePath
    MCVResult = []
    MCVResult.append(len(MCVSum))
    MCVResult.append(MCVSum)
    # 将产生的回路分配结果保存的新建的txt文档中，以便后期观察数据
    f1 = open(filePath, 'w')
    f1.write(str(MCVResult))
    f1.close()
    # 返回充电回路分配给MCV的详细信息
    return MCVSum 

if __name__ == "__main__":
    print "Programming is Begin"
    ProgrammingStartTime = time.time()

    MCVSum = TourDistributionProgramming(FirstCompare_MCV_Tour_Information_txt)
    for i in range(0, len(MCVSum)):
        print "MCVSum[", i, "] = \n", MCVSum[i]
    print "len(MCVSum) =", len(MCVSum)
    print "程序即将结束"
    ProgrammingEndTime = time.time()    
    print "程序总共耗时 ", (ProgrammingEndTime- ProgrammingStartTime), 's'
          