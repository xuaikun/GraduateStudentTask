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

# 数据初始化
N = 3   # 假设我有N辆电单车
edge_n = 500 # 假设定义的二维空间范围是 edge_n * edge_n

# 初始化电单车在二维空间中的坐标
N_x = np.empty([1, N + 1], float)
N_y = np.empty([1, N + 1], float)
# 重新编号后的节点的标号改变，对应的坐标的标号获得修改，相关值的大小未被修改
N_x_new = np.empty([1, N + 1], float)
N_y_new = np.empty([1, N + 1], float)

N_x_final = np.empty([1, N + 1], float)
N_y_final = np.empty([1, N + 1], float)

# 定义 每个节点消耗的功率
P_i = np.empty([1, N + 1], float)
# 将从小到大排序的节点消耗功率，保存到P_i_temp临时数组中，以便后期查询知道其时那个节点的功率
P_i_temp = np.empty([1, N + 1], float)
# 重新标号后的电单车的消耗功率
P_i_new = np.empty([1, N + 1], float)

# 每辆电单车的能量
Es_i = np.empty([1, N + 1], float)


# 用来保存充电回来的子集
R = np.empty([N + 1, 1], list)

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
def AllNodeLink(x, y):
    x_list = x
    y_list = y
    
    #创建图并命名
    plt.figure('Scatter fig')
    ax = plt.gca()
    #设置x轴、y轴名称
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    
    print "x_list =", x_list
    print "y_list =", y_list
    ax.scatter(x_list, y_list,color = 'red', marker = 'o')
    # 将所有节点一一连起来
    for i in range(0, len(x_list)):
        for j in range(0, len(y_list)):
            # 两两连接
            x_temp = [x_list[i], x_list[j]]
            y_temp = [y_list[i], y_list[j]]
            plt.plot(x_temp, y_temp)
            
    plt.xlim([0 - 1,edge_n + 1]) #设置绘图X边界                                                                                                   
    plt.ylim([0 - 1,edge_n + 1]) #设置绘图Y边界
    plt.show()
    return


# 创建邻接矩阵，对于重新排序的节点创建邻接矩阵
def CreateDistanceMatrix(N_x_x, N_y_y, n, N_i_new):
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
                result = A.a_star_algorithm(first_coordinate, second_coordinate)
                # print "构造回路的result = ", result
                # 电单车i到电单车j（或电单车j到电单车i）的实际距离
                # 距离取两位小数，就好
                N_distance_temp[i][j] = round(result[3],2)
                N_distance_temp[j][i] = round((N_distance_temp[i][j]), 2)
    return N_distance_temp

# 创建新邻接矩阵，对于过程中创建的邻接矩阵
def CreateDistanceNewMatrix(N_x_x, N_y_y, n, N_i_new):
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
                result = A.a_star_algorithm(first_coordinate, second_coordinate)
                # print "构造回路的result = ", result
                # 电单车i到电单车j（或电单车j到电单车i）的实际距离
                # 距离取两位小数，就好
                N_distance_temp[i][j] = round(result[3],2)
                N_distance_temp[j][i] = round((N_distance_temp[i][j]), 2)
    return N_distance_temp

# 将节点前后连接起来
def NodeToOtherNodeLink(x, y):
    print "x =", x
    print "y =", y
    #分别存放所有点的横坐标和纵坐标，一一对应
    x_list = x
    y_list = y
    
    #创建图并命名
    plt.figure('Scatter fig')
    ax = plt.gca()
    #设置x轴、y轴名称
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    
    print "x_list =", x_list
    print "y_list =", y_list
    ax.scatter(x_list, y_list,color = 'red', marker = 'o')
    # 将节点连接构成回路
    for i in range(0, N):
        # 前后连接
        x_temp = [x_list[i], x_list[i+1]]
        y_temp = [y_list[i], y_list[i+1]]
        plt.plot(x_temp, y_temp)
        if i == N - 1:
            print 'i =', i
            x_temp = [x_list[i + 1], x_list[0]]
            y_temp = [y_list[i + 1], y_list[0]]
            plt.plot(x_temp, y_temp)
    plt.xlim([0 - 1,edge_n + 1]) #设置绘图X边界                                                                                                   
    plt.ylim([0 - 1,edge_n + 1]) #设置绘图Y边界
    plt.show()
    return 

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
def ChangeChildrenTour(P_temp, N_distance_temp):
     # 初始化充电子回路的子集
     # 用来保存充电回来的子集
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
    while i <= N:
        for k in range(i, N + 1):
            # print "返回开始时i =", i
            # 记录是否重新创建回路的标志
            if R_list_flag is True:
                R_list = []
                R_list.append(0)
            # 将电单车Nk加入Rz里面
            # print "每次k都会+1 k =", k
            R_list.append(k)
            R_list_flag = False
            # 应用可调度性判定算法判定再加入Nk后Rz的可调度性
            # 如果Rz不可以可调度，则将Nk从回路中取出
            # 可调度性判断
            Em = 72000  # MC的总能量为72000 kj
            qm = 110  # Mc移动功耗为qm = 10 J/m
            qc = 1800  # qc*n 为能量传输率，qc= 4.45 W
            nl = 0.5  # 类似于效率一样，占比多少 n = 0.5
            T = 7200  # 充电周期需要知道10s
            vm = 20  # MC的移动速度0.3m/s
            print "加入了Nk节点的R_list =", R_list
            # 返回可调度性
            # result = True 可调度
            # result = False 不可调度
            result = B.judging_whether_scheduled(N, P_temp, Em, qc, qm, nl, R_list, vm, N_distance_temp, T)
            # 最后一组值的操作
            if success_flag is False and result is True and k == N:
                # print "最后一组结果可以在这保存"
                R_temp[z][count] = R_list
                i = k + 1
                break
            # print "可调度性判断result =", result
            if result is False:
                R_list.remove(k)
                # 说明只有S在里面
                # 下一次从k + 1遍历
                # print "不可以被调度时，i应该被修改了i = ", i, "并且k =", k
                print "删除了Nk节点的R_list = ", R_list
                # print "z =", z
                # print "count =", count
                # print "R[", z, "][0]=", R[z][count]
                print "对于不可调度有其他情况，每次先判断当前R_list的长度 len(R_list)=", len(R_list)
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
            print "表示所有的节点都可以加入，R里面"
            print "R[", z, "][", count, "]=", R_temp[z][count]
            i = N + 1
            break
    result = []
    result.append(R_temp)
    result.append(z)
    return result



# 程序从这里开始执行
if __name__ == "__main__":
    # 随机在1000*1000的空间内生成N辆电单车的坐标
    # 将生成的电单车坐标分别保存到x,y列表中
    # 保存节点序号的
    N_i = []
    N_i.append(0)
    x = []
    y = []
    v = 5   # 初始化电单车的运行速度为 5m/s
    for i in range(1, N+1):
        N_x[0][i] = round(random.uniform(1, edge_n), 2)
        N_y[0][i] = round(random.uniform(1, edge_n), 2)
        # 每辆点电单车的电量，初始化为840Kj
        Es_i[0][i] = 840000
        N_i.append(i)
        x.append(N_x[0][i])
        y.append(N_y[0][i])
    # 程序开始运行，计时开始
    start_time = time.time()
    print "x =", x
    print "y =", y
    print "N_i =", N_i
    # 将电单车之间两两连接起来画图
    AllNodeLink(x, y)
    
    # 随机生成每辆电单车的功率
    for i in range(1, N + 1):
        # 单位为 W
        P_i[0][i] = round(random.uniform(95, 115), 2)
        # P_i[0][i] = round(random.uniform(150, 180), 2)
        # P_i_temp[0][i] = P_i[0][i]
    
    for i in range(1, N + 1):
        print "P_i[0][", i, "]=", P_i[0][i]
    
    # 打印生成的N辆电单车的坐标
    for i in range(1, N + 1):
        print "坐标为：", (N_x[0][i], N_y[0][i])

    # 电单车停止时间
    # time.sleep(1)
    
    # 将节点坐标复制
    N_x_new = N_x
    N_y_new = N_y
    
    # 电单车能量到此开始计算减少了多少
    end_time = time.time()
    Es_temp = []
    for i in range(1 , N + 1):
        Es_temp.append(Es_i[0][i])    
    print "开始时Es_i =", Es_temp 
    # 准备充电之前都在运行
    for i in range(1, N + 1):
        Es_i[0][N_i[i]] = round((Es_i[0][N_i[i]] - P_i[0][N_i[i]]*(end_time - start_time)), 2)
    
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
    S_x = N_x_new[0][P_i_Max_Node] + 1
    S_y = N_y_new[0][P_i_Max_Node] + 1
    S.append(S_x)
    S.append(S_y)
    print "充电桩坐标为：", (S[0], S[1])
    # 表示充电桩S的坐标，加入了N中
    N_x_new[0][0] = S[0]
    N_y_new[0][0] = S[1]
    
    
   
    # 备份N_i
    N_i_temp = N_i
    # 最优的操作节点排序
    Node_optimal_sort = []
    # 先把S点放进去
    Node_optimal_sort.append(0)
    # 每个节点都需要操作
    # 直到 len(N_i) = 1 表明里面只剩下当前充电桩S的位置节点
    # 就退出while
    while 1 != len(N_i):
        # S统计到各个节点的距离
        # 统计电单车剩余的能量
        print "len(N_i) =", len(N_i)
        print "N_i =", N_i
        N_distance_temp = []
        Es_temp = []
        # 创建一个N*N的空降，生成邻接矩阵，构造无向图G(V,E)
        # N_distance[Ni][Nj] = x 表示Ni到Nj（或Nj到Ni）的距离为x
        N_distance = np.empty([len(N_i), len(N_i)], float)
        for i in range(0, len(N_i)):
            for j in range(0, len(N_i)):
                N_distance[i][j] = 0.0
        # print "邻接矩阵矩阵初始化"
        # PrintNewMatrix(N_distance, len(N_i))
        # 加入S点后，邻接矩阵每次都会更新，之后每次都会减少节点
        # print "N_x_new = ",N_x_new
        # print "N_y_new = ",N_y_new
        # 为什么使用len(N_i_temp)
        N_distance = CreateDistanceNewMatrix(N_x_new, N_y_new, len(N_i), N_i)
        # 重新打印邻接矩阵
        PrintNewMatrix(N_distance, len(N_i))
        # 只考虑当前所剩下的节点，不包括S点（N_i[0]）
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
        # np.delete(N_distance, N_distance)
    
    
    #  最优的节点序列已经找出
    
    print "最优的节点序列Node_optimal_sort =", Node_optimal_sort
    # Node_optimal_sort = [0, 1, 3, 2]
    
    # 对原来的排序进行改变     
    for i in range(0, len(Node_optimal_sort)):
        N_x_final[0][i] = N_x_new[0][Node_optimal_sort[i]]
        N_y_final[0][i] = N_y_new[0][Node_optimal_sort[i]]
        
        P_i[0][Node_optimal_sort[i]] = round(P_i[0][Node_optimal_sort[i]],2)
        P_i_new[0][i] = round(P_i[0][Node_optimal_sort[i]],2)
    # 排序完成 
    x = N_x_final[0]
    y = N_y_final[0]
    
    
    print "N_x_new = ",N_x_new
    print "N_y_new = ",N_y_new
    
    print "N_x_final = ",N_x_final
    print "N_y_final = ",N_y_final
    
    print "P_i =", P_i
    print "P_i_new =", P_i_new
    
    # print "len(Node_optimal_sort) =", len(Node_optimal_sort)
    N_distance = np.empty([len(Node_optimal_sort), len(Node_optimal_sort)], float)
    for i in range(0, len(Node_optimal_sort)):
        for j in range(0, len(Node_optimal_sort)):
            N_distance[i][j] = 0.0
    # print "邻接矩阵矩阵初始化"
    # PrintNewMatrix(N_distance, len(N_i))
    # 加入S点后，邻接矩阵每次都会更新，之后每次都会减少节点
    # 延时1s，应该是电脑反应不过来，出现错误
    time.sleep(1)
    # N_x_final = N_x_final
    # N_y_final = N_y_final
    # print "N_x_final =", N_x_final
    # print "N_y_final =", N_y_final
    
    # 与上面的索引生成邻接矩阵不一样，这里是直接产生邻接矩阵
    # 这里是已经排好序的列表
    N_distance = CreateDistanceMatrix(N_x_final, N_y_final, len(Node_optimal_sort), Node_optimal_sort)
    # 重新打印邻接矩阵
    PrintNewMatrix(N_distance, len(Node_optimal_sort))
    
    
    # 加入S点后，将电单车前后连接起来
    NodeToOtherNodeLink(x, y)
   
    print "开始构造子回路"
    
    # 构造充电子回路
    # P_i_new, N_distance 可以代表重新节点排序
    result = ChangeChildrenTour(P_i_new, N_distance)
    R = result[0]
    z = result[1]
    
    print "result = ", result
    
    print "R =", R
    print "z =", z
    
    print "\n"
    # 打印检查产生的充电子回路有多少条，每条子回路由那些节点组成
    for i in range(0, z + 1):
        print R[i]

