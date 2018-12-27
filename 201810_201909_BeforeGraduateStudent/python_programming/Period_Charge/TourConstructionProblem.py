# encoding: utf-8
import numpy as np
import random
import A_Star_Algorithm as A
import JudgingWhetherScheduled as B


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


N = 5    # 假设我有N辆电单车
# 初始化电单车在二维空间中的坐标
N_x = np.empty([1, N + 1], int)
N_y = np.empty([1, N + 1], int)
# 重新编号后的节点的标号改变，对应的坐标的标号获得修改，相关值的大小未被修改
N_x_new = np.empty([1, N + 1], int)
N_y_new = np.empty([1, N + 1], int)
# 随机在1000*1000的空间内生成N辆电单车的坐标
for i in range(1, N+1):
    N_x[0][i] = random.randint(1, 6)
    N_y[0][i] = random.randint(1, 6)

# 定义 每个节点消耗的功率
P_i = np.empty([1, N + 1], int)
# 将从小到大排序的节点消耗功率，保存到P_i_temp临时数组中，以便后期查询知道其时那个节点的功率
P_i_temp = np.empty([1, N + 1], int)
# 重新标号后的电单车的消耗功率
P_i_new = np.empty([1, N + 1], int)
# 随机生成每辆电单车的功率
for i in range(1, N + 1):
    P_i[0][i] = random.randint(1, 10)
    P_i_temp[0][i] = P_i[0][i]

for i in range(1, N + 1):
    print "P_i[0][", i, "]=", P_i[0][i]


# 打印生成的N辆电单车的坐标
for i in range(1, N + 1):
    print "坐标为：", (N_x[0][i], N_y[0][i])
# 创建一个N*N的空降，生成邻接矩阵，构造无向图G(V,E)
# N_distance[Ni][Nj] = x 表示Ni到Nj（或Nj到Ni）的距离为x
N_distance = np.empty([N + 1, N + 1], int)
# i和j表示第n个节点
for i in range(1, N + 1):
    for j in range(i, N + 1):
        if i == j:
            # 自己到自己的距离为0
            N_distance[i][j] = 0
        else:
            # i到j的距离，i<j,因为为无向图，可以使用N_distance[j][i]= N_distance[i][j] 将对称的位置赋相同的值
            # 将当前两个点的坐标提取出来
            x1 = N_x[0][i]
            y1 = N_y[0][i]
            first_coordinate = []
            first_coordinate.append(x1)
            first_coordinate.append(y1)
            x2 = N_x[0][j]
            y2 = N_y[0][j]
            second_coordinate = []
            second_coordinate.append(x2)
            second_coordinate.append(y2)
            # print "第一个坐标：", first_coordinate
            # print "第二个坐标：", second_coordinate
            result = A.a_star_algorithm(first_coordinate, second_coordinate)
            # print "构造回路的result = ", result
            # 电单车i到电单车j（或电单车j到电单车i）的实际距离
            N_distance[i][j] = result[3]
            N_distance[j][i] = N_distance[i][j]
print "构造无向图："
for i in range(1, N + 1):
    N_distance_list = []
    for j in range(1, N + 1):
        N_distance_list.append(N_distance[i][j])
    # 将无向图打印出来看看
    print N_distance_list

# for i in range(1, N + 1):
#    print "P_i_temp[0][", i, "]=", P_i_temp[0][i]
select_sort(P_i_temp)
# for i in range(1, N + 1):
#     print "P_i_temp[0][", i, "]=", P_i_temp[0][i]

for i in range(1, N + 1):
    for j in range(1, N + 1):
        # 只要从小到大遍历，那么对于相同大小值之前的位置不变
        if P_i_temp[0][i] == P_i[0][j]:
            # j 为之前的电单车的标号
            P_i_new[0][i] = P_i[0][j]
            N_x_new[0][i] = N_x[0][j]
            N_y_new[0][i] = N_y[0][j]
            P_i[0][j] = 1000

            # 可能有很多组这样的值，但只有第一组符合
            break

for i in range(1, N + 1):
    print "P_i_new[0][", i, "]=", P_i_new[0][i]
for i in range(1, N + 1):
    print "坐标为：", (N_x_new[0][i], N_y_new[0][i])

# 表示充电桩的位置，暂时定义在这
S = [0, 0]
# 表示充电桩S的坐标，加入了N中
N_x_new[0][0] = S[0]
N_y_new[0][0] = S[1]

for i in range(0, N + 1):
    for j in range(i, N + 1):
        if i == j:
            N_distance[i][j] = 0
        else:
            # 重新
            x1 = N_x_new[0][i]
            y1 = N_y_new[0][i]
            x2 = N_x_new[0][j]
            y2 = N_y_new[0][j]
            first_coordinate = []
            second_coordinate = []
            first_coordinate.append(x1)
            first_coordinate.append(y1)
            second_coordinate.append(x2)
            second_coordinate.append(y2)
            # print "第一个坐标：", first_coordinate
            # print "第二个坐标：", second_coordinate
            result = A.a_star_algorithm(first_coordinate, second_coordinate)
            # print "构造回路的result = ", result
            N_distance[i][j] = result[3]
            N_distance[j][i] = N_distance[i][j]
print "新的无向图："
for i in range(0, N + 1):
    N_distance_list = []
    for j in range(0, N + 1):
        N_distance_list.append(N_distance[i][j])
    print N_distance_list


print "开始构造子回路"


R = np.empty([N + 1, 1], list)
for i in range(0, N + 1):
        R[i][0] = ' '
i = 1
z = 0
count = 0
while i <= N:
    print "while之后的开始时，i =", i
    R_list = []
    R_list.append(0)
    # 可以添加回路成功的标志
    success_flag = True
    if z == N + 1:
        print "程序结束z = ", z
    break
    for k in range(i, N + 1):
        # 将电单车Nk加入Rz里面
        print "每次k都会+1 k =", k
        R_list.append(k)
        # 应用可调度性判定算法判定再加入Nk后Rz的可调度性
        # 如果Rz不可以可调度，则将Nk从回路中取出
        # 可调度性判断
        Em = 50000  # MC的总能量为65 kj
        qm = 8  # Mc移动功耗为qm = 8 J/m
        qc = 100  # qc*n 为能量传输率，qc= 4.45 W
        nl = 0.5  # 类似于效率一样，占比多少 n = 0.5
        T = 2000  # 充电周期需要知道10s
        vm = 0.3  # MC的移动速度0.3m/s
        print "加入了Nk节点的R_list =", R_list
        result = B.judging_whether_scheduled(N, P_i_new, Em, qc, qm, nl, R_list, vm, N_distance, T)
        print "可调度性判断result =", result
        if result is False:
            R_list.remove(k)
            i = k
            z = z + 1
            print "不可以被调度时，i应该被修改了i = ", i, "并且k =", k
            print "删除了Nk节点的R_list = ", R_list
            print "z =", z
            print "count =", count
            R[z][count] = R_list
            print "R[", z, "][0]=", R[z][count]
            # 表明只能有部分节点能加入同一个回路
            success_flag = False
            print "退出下一轮遍历,并且k = k + 1"
            break

    # 表示所有的节点都可以放入同一个充电回路
    if success_flag is True:
        R[z][count] = R_list
        print "表示所有的节点都可以加入，R里面"
        print "R[", z, "][", count, "]=", R[z][count]
        i = N + 1
        break

print "\n"
for q in range(0, N + 1):
    print R[q][0]

