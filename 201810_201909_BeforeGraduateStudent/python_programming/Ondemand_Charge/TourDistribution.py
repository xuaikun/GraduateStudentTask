# encoding: utf-8
# 本程序主要实现，充电回路的构造

# 算法功能：充电回路的构造
# 算法输入：电单车在二维空间中的坐标以及功率
# 算法输出： 充电回路子回路结果
import os
import numpy as np

# 调试程序的标志，DebugFlag = True 为调试
# DebugFlag = False 不调试
DebugFlag = False

# 输入要操作的文件的路径【一般文件是txt文件】
# 函数功能：将输入的充电回路，分配给最少的MCV
# 函数输入：保存着充电回路数据的txt文档
# 函数输出：MCV数量，已经每个MCV为哪些充电回路工作的详细信息
def TourDistributionProgramming(MCV_Tour_Information_txt):
    # 充电回路周期T = MCV给服务点的充电时间 + MCV移动到服务点的时间
    # 充电时间Tj-working: MCV给服务点的充电时间
    # 导入测试数据
    TourSum = np.loadtxt(MCV_Tour_Information_txt, dtype = np.float)
    # 初始化充电回路分配给MCV的集合
    MCVSum = []
    # if DebugFlag is True:
    print "2 D =", TourSum.ndim
    print "TourSum =\n", TourSum
    # 需要判断充电回路集合，是否只有一个回路，只有一个充电回路则为一维数据，否则为二维数据
    if TourSum.ndim == 1:
        MCVSum.append(TourSum)
    elif TourSum.ndim != 1:
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
    print "len(MCVSum) =", len(MCVSum)
    print "before MCVSum =", MCVSum
    MCVResult = []
    MCVResult.append(len(MCVSum))
    MCVResult.append(MCVSum)
    # 将产生的回路分配结果保存的新建的txt文档中，以便后期观察数据
    f1 = open(filePath, 'w')
    f1.write(str(MCVResult))
    f1.close()
    print "len(MCVSum) =", len(MCVSum)
    print "after MCVSum =", MCVSum
    # 返回充电回路分配给MCV的详细信息
    return MCVSum 
         