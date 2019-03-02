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

# 数据结果保存路径 (需要使用自己电脑的路径),自己建立一个result文件
result_path = "E:\\00000000000graduate-study\\GraduateStudentTask\\201810_201909_BeforeGraduateStudent\\python_programming\\Ondemand_Charge\\result"

# 选择插入算法角度阈值设定
# cos90 = 0
# cos180 = -1
# cos90 > cosr > cos180
# 90 < r < 180
# 目前设定
MaxAngle = -1  # 对应最大角度为180°
MinAngle = 0   # 对应最小角度为90° 
# 以下为数据初始化
# 节点数目 从 50 到 200 变化，将100节点的实验先 做全# 假设我有N辆电单车  会影响程序运行的时间
NodeNum = 6   
# 修改MCV的变量 每次 只修改一个
# 1J=1s*1W
# 1KJ=1000J
# Em  数值从150kj 到400kj变化，间隔50kj变化# MC的总能量 j
Em = 150000.0   
# 测试过程中从4m/s 到10m/s 变化# MC的移动速度 m/s
Vm = 4.0  
# Mc移动功耗为 J/m
Qm = 55.0  
# MCV充电传输速率 W
Qc = 40.0  
# 阈值上限当电车剩余能量小于1000.0j时，电车将发送request给MCV
Et = 890
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
# 创建txt文件
MCV_Tour_txt = os.path.join(childern_result_name, 'MCV_Tour_Set.txt')
result_txt = os.path.join(childern_result_name, 'MCV_Tour_Information.txt')
Obstacle_information_data_txt = os.path.join(result_name, 'Obstacle_information_data.txt')
Node_information_data_txt = os.path.join(result_name, 'Node_information_data.txt')


if __name__ == "__main__":
    print "Programming is Begin……"
    ProgrammingStartTime = time.time()
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
    
     # 程序开始运行，计时开始
    start_time = time.time()
    time.sleep(1.5)
    end_time = time.time()
    t_sum = (end_time - start_time)
    # 充电回路汇总
    R_Sum = []
    # 充电回路的命名
    MCV_Num = 0
    # 每个充电回路的信息的汇总
    DataStore_list_sum = []
    # 需要对每个节点进行操作，直到剩余服务站S的标号
    while 1 != len(NodeList):
        t_sum = 50
        t = 50
        print "每次运行花费 t =", t, 's'
        # 每次操作前先把能量进行备份，一边后续操作所需
        NodeEsBackup = []
        for i in range(0, len(NodeList)):
            NodeEsBackup.append(NodeEs[0][NodeList[i]])
        # 更新节点剩余的能量  # 不仅仅是考虑边界，还得考虑运动过程中，遇到障碍应该避开
        # 只取NodeList中第一个节点，直到这个节点发出Request并且被满足可调度性条件
        i = 1
        # 修改节点能量
        NodeEs[0][NodeList[i]] = round((NodeEs[0][NodeList[i]] - NodeP[0][NodeList[i]]*t), 2)
        
        print "预计消耗的能量 NodeP[0][NodeList[i]]*t =", NodeP[0][NodeList[i]]*t_sum, 'j'
        print "能量阈值上限 Et =", Et, 'j'
        
        # 能量将低于阈值下限
        if NodeEs[0][NodeList[i]] <= Et:
            if NodeEsBackup[i] <= Et:
                print "case 1"
                print "之前该节点已经失效，不再参与计算"
                # 剩余能量还是上次备份的能量
                NodeEs[0][NodeList[i]] = NodeEsBackup[i]
            else:
                print "case 2"
                print "NodeEs[0][NodeList[i]]  =", NodeEs[0][NodeList[i]]
                print "节点实际消耗的能量为", NodeEsBackup[i] - Et, 'j'
                # 修改节点剩余能量,该点变为静态点了
                NodeEs[0][NodeList[i]] = Et
                print "节点实际运动的时间为", (NodeEsBackup[i] - Et)/NodeP[0][NodeList[i]], 's'
                NodeMoveTime[0][NodeList[i]] = (NodeEsBackup[i] - Et)/NodeP[0][NodeList[i]]
        # 能量不会低于阈值下限
        if NodeEs[0][NodeList[i]] > Et:
            print "case 3"
            print "NodeEs[0][NodeList[i]]  =", NodeEs[0][NodeList[i]]
            print "节点实际消耗的能量为", NodeP[0][NodeList[i]]*t_sum, 'j'
            print "节点实际运动的时间为", t_sum, 's'
            NodeMoveTime[0][NodeList[i]] = t_sum
        
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
        # 测试数据用的 事后需要将其注释掉
        # NodeEs[0][1] = El
        # print "NodeEs[0] =", NodeEs[0]
        # 当电单车节点剩余的能量小于阈值上限时，节点向服务站S发送充电请求信号Request
        if NodeEs[0][NodeList[i]] <= Et:
            print "发送充电请求Request"
            print "将该节点添加进入充电回路"
            NodeRequest[0][NodeList[i]] = 1
            
            BestNode = NodeList[i]
            
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
                    print "DataStore_list =", DataStore_list
                    DataStore_list_sum.append(DataStore_list)
                else:
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
                   
            print "R_New =", R_New
            print "NodeList =", NodeList
            print "操作一次Request缓冲池需要的时间为 t =", t, 's'
        if len(NodeList) == 1:
            print "说明NodeList已经删除到剩余服务站节点"
            break
    print "充电回路汇总"
    print "R_Sum =\n", R_Sum
    f1 = open("MCV_Tour.txt", 'w')
    f1.write(str(R_Sum))
    f1.close()
    print "DataStore_list_sum =\n", DataStore_list_sum
    # 回路消耗等相关数据的保存路径
    np.savetxt("result.txt", DataStore_list_sum, fmt='%0.2f')
    print "需要操作的节点序列中，不存在未操作的节点"        