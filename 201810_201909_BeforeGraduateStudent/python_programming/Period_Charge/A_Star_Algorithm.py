# encoding: utf-8

# A*算法
# 算法功能：用于计算两点之间的距离，可避开障碍物
# 算法输入：起点坐标，终点坐标
# 算法输出：两点之间的距离


import numpy as np
import math
import time
# debug_flag 为调试标志，为True是开启调试模式， 为False时关闭调试模式
debug_flag = False
# 本程序未考虑负的坐标系
# create_2D_space[][] = 2 时为障碍
# create_2D_space[][] = 0 时为空地
# create_2D_space[][] = 1 时为起点和中点
n = 2000          # 定义二维平面大小
# 定义 每个位置的g值
g = np.empty([n + 1, n + 1], float)
# 定义 每个位置的h值
h = np.empty([n + 1, n + 1], float)
for i in range(0, n + 1):
    for j in range(0, n + 1):
        g[i][j] = 0.0
        h[i][j] = 0 
# 构造二维平面
create_2D_space = np.empty([n + 2, n + 2], int)
# 开放列表open_list_S,用于存放当前操作点周围的点，或之前操作的点周围的点，不能是障碍点，它的值为0或1
open_list_S = np.empty([n + 1, n + 1], int)
# 封闭列表close_list_C，用于存放已被操作过的点，不能再被操作，该点的值应该为1
# 并且create_2D_space[][] = 2赋值，表示该点成为障碍
close_list_C = np.empty([n + 1, n + 1], int)

# 用全局列表保存的话，更方便操作
# 保存临时障碍坐标
temp_obstacle_x = []
temp_obstacle_y = []

# 保存永久障碍坐标
perpetual_obstacle_x = []
perpetual_obstacle_y = []

def init(start_position, end_position):
    # 初始化二维平面
    # 当二维平面中位置的值不为0，1时，说明这里是障碍，就不能越过，得绕过
    # 其中为0的点为空旷区域，可以随意通过
    # 其中为1的点为起始点或者目标点
    # 已经走过的地方，可以用2表示，不能再被再一次访问，也表示障碍
    for i in range(1, n + 1):
        for j in range(i, n + 1):
            create_2D_space[i][j] = 0
            create_2D_space[j][i] = create_2D_space[i][j]
    # 外围一层变成障碍
    for i in range(0, n + 2):
        create_2D_space[0][i] = 2
        create_2D_space[i][0] = create_2D_space[0][i]
        create_2D_space[n + 1][i] = 2
        create_2D_space[i][n + 1] = create_2D_space[n + 1][i]

    # 将起始坐标和目的坐标加入二维平面中
    create_2D_space[start_position[0]][start_position[1]] = 1
    create_2D_space[end_position[0]][end_position[1]] = 1

    # 检查二维平面
    '''
    for i in range(1, n + 1):
        # 将平面按行打印，提前保存到列表里面
        space_list = []
        for j in range(1, n + 1):
            space_list.append(create_2D_space[i][j])
            if j == n:
                print space_list
    '''
    # 若open_list_S[i][j] = 0，表示没有保存数据，表示点(i,j)不能被操作
    # 若open_list_S[1][1] = 1，表示坐标为（1,1）的点在open_list_S中可以被操作
    for i in range(1, n + 1):
        for j in range(1, n + 1):
            open_list_S[i][j] = 0

    # 若close_list_C[i][j] = 0，表示没有点（i,j）未保存保存到close_list_C中，任然可以被考察
    # 若close_list_C[1][1] = 1，表示坐标为（1,1）的点在close_list_C中，暂时不能被考察，
    # 若同时满足create_2D_space[i][j] = 2（已被操作过的点，将其视为障碍）, 则表示点（i,j）不再被考察
    for i in range(1, n + 1):
        for j in range(1, n + 1):
            close_list_C[i][j] = 1

    # 将起始点加入开放列表中
    open_list_S[start_position[0]][start_position[1]] = 1
    close_list_C[start_position[0]][start_position[1]] = 0


def set_obstacle(obstacle_coordinate, obstacle_num):
    # 此函数用于设置障碍的
    # create_2D_space[2][5] = 2
    # create_2D_space[3][4] = 2
    # create_2D_space[4][3] = 2
    # create_2D_space[5][2] = 2
    # create_2D_space[4][4] = 2
    # 当前点(8,7)为障碍
    # create_2D_space[8][7] = 2
    x_down_list = obstacle_coordinate[0]
    x_up_list  = obstacle_coordinate[1]

    # 障碍纵坐标y的范围
    y_down_list = obstacle_coordinate[2]
    y_up_list = obstacle_coordinate[3]
    for i in range(0, obstacle_num):
        x_down_value= x_down_list[0][i]
        x_up_value= x_up_list[0][i]
        y_down_value= y_down_list[0][i]
        y_up_value= y_up_list[0][i]
        # 遍历横坐标
        # print "x_down_value =", x_down_value
        # print "x_up_value =", x_up_value
        # print "y_down_value =", y_down_value
        # print "y_up_value =", y_up_value
        for j in range(x_down_value,x_up_value + 1):
            # 遍历纵坐标
            for k in range(y_down_value, y_up_value + 1):
                # （x_down_value+x_up_value）*(y_down_value + y_up_value)区域都是障碍
                create_2D_space[j][k] = 2
                # print "create_2D_space[", j, "][", k, "] =", create_2D_space[j][k] 
    return


def possibilities_position(parent_position, goal_position):
    
    # 将当前操作的点置为临时障碍
    temp_obstacle_x.append(parent_position[0])
    temp_obstacle_y.append(parent_position[1])
    create_2D_space[parent_position[0]][parent_position[1]] = 2  # 临时障碍，需要时可以置为空间
    
    # 起始坐标
    start_point = parent_position 
    # 左边
    left_position_x = parent_position[0] - 1
    left_position_y = parent_position[1]
    
     # 上面
    up_position_x = parent_position[0]
    up_position_y = parent_position[1] + 1
    
    # 右边
    right_position_x = parent_position[0] + 1
    right_position_y = parent_position[1]
    
     # 下
    down_position_x = parent_position[0]
    down_position_y = parent_position[1] - 1
    around_obstacle_num = 0
    # 检查当前点正反向四周障碍的数量
    if create_2D_space[left_position_x][left_position_y] == 2:
        around_obstacle_num = around_obstacle_num + 1
    if create_2D_space[up_position_x][up_position_y] == 2:
        around_obstacle_num = around_obstacle_num + 1
    if create_2D_space[right_position_x][right_position_y] == 2:
        around_obstacle_num = around_obstacle_num + 1
    if create_2D_space[down_position_x][down_position_y] == 2:
        around_obstacle_num = around_obstacle_num + 1
    # 如果当前节点周围周围，存在至少3面有障碍，则当前点置为障碍
    # 针对数值小的平面进行操作
    # print "around_obstacle_num = ", around_obstacle_num
    # print "temp_obstacle_x =", temp_obstacle_x
    # print "temp_obstacle_y =", temp_obstacle_y
    # print "perpetual_obstacle_x =", perpetual_obstacle_x
    # print "perpetual_obstacle_y =", perpetual_obstacle_y
    if around_obstacle_num > 3:
        print "当前节点周围存在四面障碍，将其置为永久障碍"
        print "临时障碍释放为可以遍历的空间"
        for i in range(0, len(temp_obstacle_x)):
            # 将除了当前的节点外，暂时的障碍全部释放为可以遍历的空间
            # print "清空temp_obstacle"
            # if temp_obstacle_x[0] != parent_position[0] and temp_obstacle_y[0] != parent_position[1]:
            create_2D_space[temp_obstacle_x[0]][temp_obstacle_y[0]] = 0
            # print "create_2D_space[",temp_obstacle_x[0],"][",temp_obstacle_y[0],"]=",create_2D_space[temp_obstacle_x[0]][temp_obstacle_y[0]]
            temp_obstacle_x.remove(temp_obstacle_x[0])
            temp_obstacle_y.remove(temp_obstacle_y[0])
            # print "temp_obstacle_x =", temp_obstacle_x
            # print "temp_obstacle_y =", temp_obstacle_y
            # print "perpetual_obstacle_x =", perpetual_obstacle_x
            # print "perpetual_obstacle_y =", perpetual_obstacle_y
        # 把这个点置为永久障碍点，以后都不会在进来这个区域
        perpetual_obstacle_x.append(parent_position[0])
        perpetual_obstacle_y.append(parent_position[1])
        # 将当前四面环绕障碍的点设置为永久障碍
        create_2D_space[parent_position[0]][parent_position[1]] = 2
        # print "create_2D_space[", parent_position[0],"][",parent_position[1
        #                      ],"=",create_2D_space[parent_position[0]][parent_position[1]]       
        
    end_position = goal_position
    result_list = []
    # 首先判断要到达的终点是不是在障碍区域内
    if create_2D_space[end_position[0]][end_position[1]]==2 :
        print "终点在障碍区域内,退出A*算法程序"
        result_list.append(False)
        result_list.append(end_position[0])
        result_list.append(end_position[1])
        result_list.append(10000000)
   
    else:
        # print "终点不在障碍区域内，继续执行"
        # 观察邻近的可行方块： parent_position 表示当前位置
        # 保证被操作的点不越界、不为障碍
        # 保存位置数据的两个数组初始化
        f_x = np.empty([1, 10], int)
        f_y = np.empty([1, 10], int)
        # 初始化f_x[][]和f_y[][]的值，避免出现随机值
        # （f_x[0][q], f_y[0][q]）==（0，0）点不属于我们的2D空间，可用于判断我们操作的点有没有越界
        # 我们的空间的坐标从1开始
        for q in range(1, 10):
            f_x[0][q] = 0
            f_y[0][q] = 0
        # 运行结束产生的标志
        # 运行得到结果返回True
        success_flag = False
        # 上下左右出现障碍的标志
        left_flag = False
        up_flag = False
        right_flag = False
        down_flag = False
        
    
        # 重新定义 8个位置的坐标（x,y）
        left_position_x = start_point[0] - 1
        left_position_y = start_point[1]
        if debug_flag is True:
            print "left_position_x = ", left_position_x
            print "left_position_y = ", left_position_y
            print "create_2D_space[left_position_x][left_position_y] =", (
                create_2D_space[left_position_x][left_position_y])
        # 左边
        # 我们的空间坐标最小为（1,1）,最大为（n,n）
        if left_position_x >= 1:  # 选中的方块不能超界
            if create_2D_space[left_position_x][left_position_y] != 2:  # 选中的方块不是障碍
                # h值的计算方法
                left_h_value = (abs(end_position[0] - left_position_x) + abs(
                    end_position[1] - left_position_y)) * 1
                h[left_position_x][left_position_y] = left_h_value
    
                if open_list_S[left_position_x][left_position_y] != 1:  # 保证它不在open_list_S里面
                    open_list_S[left_position_x][left_position_y] = 1
                    close_list_C[left_position_x][left_position_y] = 0
                    # 目前该位置的g的计算
                    # 现在的g值 = 当前位置的g值加上移动所加的值
                    now_g = g[parent_position[0]][parent_position[1]] + 1
                    g[left_position_x][left_position_y] = now_g
    
                else:  # 该点在open_list_S里面，则应该判断上次的g值与本次的g值的大小，只保留小的g值
                    # 原来的g值是这个
                    origin_g = g[left_position_x][left_position_y]
                    # 现在的g值 = 当前位置的g值加上移动所加的值
                    now_g = g[parent_position[0]][parent_position[1]] + 1
                    # 如果现在的g值比以前的g值小了，则修改g值，否则保持原来的g值
                    if now_g < origin_g:
                        g[left_position_x][left_position_y] = now_g
    
                if left_position_x == end_position[0] and left_position_y == end_position[1]:
                    success_flag = True
                # 保存当前位置左上角的区域的坐标
                f_x[0][1] = left_position_x
                f_y[0][1] = left_position_y
                if debug_flag is True:
                    print "f_x[0][1]  = ", f_x[0][1]
                    print "f_y[0][1]  = ", f_y[0][1]
                    print "g[left_position_x][left_position_y]", g[f_x[0][1]][f_y[0][1]]
                    print "h[left_position_x][left_position_y]", h[f_x[0][1]][f_y[0][1]]
            else:  # 如果左边的是障碍，则左下角和左上角都不能通过
                left_flag = True
    
        # 上面
        up_position_x = start_point[0]
        up_position_y = start_point[1] + 1
        if debug_flag is True:
            print "up_position_x  = ", up_position_x
            print "up_position_y  = ", up_position_y
            print "create_2D_space[up_position_x][up_position_y] =", (
                create_2D_space[up_position_x][up_position_y])
    
        if up_position_y <= n:
            if create_2D_space[up_position_x][up_position_y] != 2:  # 选中的方块不是障碍
                # h值计算
                up_h_value = (abs(end_position[0] - up_position_x) + abs(
                    end_position[1] - up_position_y)) * 1
                h[up_position_x][up_position_y] = up_h_value
    
                if open_list_S[up_position_x][up_position_y] != 1:  # 保证它不在open_list_S里面
                    open_list_S[up_position_x][up_position_y] = 1
                    close_list_C[up_position_x][up_position_y] = 0
                    # 目前该位置的g值的计算
                    # 现在的g值 = 当前位置的g值加上移动所加的值
                    now_g = g[parent_position[0]][parent_position[1]] + 1
                    g[up_position_x][up_position_y] = now_g
    
                else:  # 该点在open_list_S里面，则应该判断上次的g值与本次的g值的大小，只保留小的g值
                    # 原来的g值是这个
                    origin_g = g[up_position_x][up_position_y]
                    # 现在的g值 = 当前位置的g值加上移动所加的值
                    now_g = g[parent_position[0]][parent_position[1]] + 1
                    # 如果现在的g值比以前的g值小了，则修改g值，否则保持原来的g值
                    if now_g < origin_g:
                        g[up_position_x][up_position_y] = now_g
                if up_position_x == end_position[0] and up_position_y == end_position[1]:
                    success_flag = True
    
                # 保存当前位置左上角的区域的坐标
                f_x[0][3] = up_position_x
                f_y[0][3] = up_position_y
                if debug_flag is True:
                    print "f_x[0][3]  = ", f_x[0][3]
                    print "f_y[0][3]  = ", f_y[0][3]
                    print "g[up_position_x][up_position_y]", g[f_x[0][3]][f_y[0][3]]
                    print "h[up_position_x][up_position_y]", h[f_x[0][3]][f_y[0][3]]
            else:  # 如果正上方是障碍，则左上角和右上角不能通过
                up_flag = True
        # 右边
        right_position_x = start_point[0] + 1
        right_position_y = start_point[1]
        if debug_flag is True:
            print "right_position_x = ", right_position_x
            print "right_position_y = ", right_position_y
            print "create_2D_space[right_position_x][right_position_y] =", (
              create_2D_space[right_position_x][right_position_y])
    
        if right_position_x <= n:
            if create_2D_space[right_position_x][right_position_y] != 2:  # 选中的方块不是障碍
                # h值计算
                right_h_value = (abs(end_position[0] - right_position_x) + abs(
                    end_position[1] - right_position_y)) * 1
                h[right_position_x][right_position_y] = right_h_value
    
                if open_list_S[right_position_x][right_position_y] != 1:
                    open_list_S[right_position_x][right_position_y] = 1
                    close_list_C[right_position_x][right_position_y] = 0
                    # 目前该位置的g 和 h的值的计算
                    # 现在的g值 = 当前位置的g值加上移动所加的值
                    now_g = g[parent_position[0]][parent_position[1]] + 1
                    # 如果现在的g值比以前的g值小了，则修改g值，否则保持原来的g值
                    g[right_position_x][right_position_y] = now_g
    
                else:  # 该点在open_list_S里面，则应该判断上次的g值与本次的g值的大小，只保留小的g值
                    # 原来的g值是这个
                    origin_g = g[right_position_x][right_position_y]
                    # 现在的g值 = 当前位置的g值加上移动所加的值
                    now_g = g[parent_position[0]][parent_position[1]] + 1
                    # 如果现在的g值比以前的g值小了，则修改g值，否则保持原来的g值
                    if now_g < origin_g:
                        g[right_position_x][right_position_y] = now_g
    
                if right_position_x == end_position[0] and right_position_y == end_position[1]:
                    success_flag = True
                # 保存当前位置左上角的区域的坐标
                f_x[0][5] = right_position_x
                f_y[0][5] = right_position_y
                if debug_flag is True:
                    print "f_x[0][5]  = ", f_x[0][5]
                    print "f_y[0][5]  = ", f_y[0][5]
                    print "g[right_position_x][right_position_y]", g[f_x[0][5]][f_y[0][5]]
                    print "h[right_position_x][right_position_y]", h[f_x[0][5]][f_y[0][5]]
            else:  # 右边为障碍，则右上角和右下角不能通过
                right_flag = True
        # 下
        down_position_x = start_point[0]
        down_position_y = start_point[1] - 1
        if debug_flag is True:
            print "down_position_x = ", down_position_x
            print "down_position_y = ", down_position_y
            print "create_2D_space[down_position_x][down_position_y] =", (
               create_2D_space[down_position_x][down_position_y])
    
        if down_position_y >= 1:
            if create_2D_space[down_position_x][down_position_y] != 2:  # 选中的方块不是障碍
                # h值计算
                down_h_value = (abs(end_position[0] - down_position_x) + abs(
                    end_position[1] - down_position_y)) * 1
                h[down_position_x][down_position_y] = down_h_value
    
                if open_list_S[down_position_x][down_position_y] != 1:
                    open_list_S[down_position_x][down_position_y] = 1
                    close_list_C[down_position_x][down_position_y] = 0
                    # 目前该位置的g 值的计算
                    # 现在的g值 = 当前位置的g值加上移动所加的值
                    now_g = g[parent_position[0]][parent_position[1]] + 1
                    g[down_position_x][down_position_y] = now_g
    
                else:  # 该点在open_list_S里面，则应该判断上次的g值与本次的g值的大小，只保留小的g值
                    # 原来的g值是这个
                    origin_g = g[down_position_x][down_position_y]
                    # 现在的g值 = 当前位置的g值加上移动所加的值
                    now_g = g[parent_position[0]][parent_position[1]] + 1
                    # 如果现在的g值比以前的g值小了，则修改g值，否则保持原来的g值
                    if now_g < origin_g:
                        g[down_position_x][down_position_y] = now_g
    
                if down_position_x == end_position[0] and down_position_y == end_position[1]:
                    success_flag = True
    
                # 保存当前位置左上角的区域的坐标
                f_x[0][7] = down_position_x
                f_y[0][7] = down_position_y
                if debug_flag is True:
                    print "f_x[0][7]  = ", f_x[0][7]
                    print "f_y[0][7]  = ", f_y[0][7]
                    print "g[down_position_x][down_position_y]", g[f_x[0][7]][f_y[0][7]]
                    print "h[down_position_x][down_position_y]", h[f_x[0][7]][f_y[0][7]]
            else:  # 正下方为障碍，则右下角和左下角不能直接通过
                down_flag = True
        # 如果左边或者上边是障碍，左上角不能通过
        if left_flag is False and up_flag is False:
            # 左上
            left_up_position_x = start_point[0] - 1
            left_up_position_y = start_point[1] + 1
            if debug_flag is True:
                print "left_up_position_x = ", left_up_position_x
                print "left_up_position_x = ", left_up_position_y
                print "create_2D_space[left_up_position_x][left_up_position_y] =", (
                   create_2D_space[left_up_position_x][left_up_position_y])
    
            if left_up_position_x >= 1 and left_up_position_y <= n:
                if create_2D_space[left_up_position_x][left_up_position_y] != 2:  # 选中的方块不是障碍
                    # h值计算
                    left_up_h_value = (abs(end_position[0] - left_up_position_x) + abs(
                        end_position[1] - left_up_position_y)) * 1
                    h[left_up_position_x][left_up_position_y] = left_up_h_value
    
                    if open_list_S[left_up_position_x][left_up_position_y] != 1:
                        open_list_S[left_up_position_x][left_up_position_y] = 1
                        close_list_C[left_up_position_x][left_up_position_y] = 0
                        # 目前该位置的g 值的计算
                        # 现在的g值 = 当前位置的g值加上移动所加的值
                        now_g = g[parent_position[0]][parent_position[1]] + 1.4
                        g[left_up_position_x][left_up_position_y] = now_g
    
                    else:  # 该点在open_list_S里面，则应该判断上次的g值与本次的g值的大小，只保留小的g值
                        # 原来的g值是这个
                        origin_g = g[left_up_position_x][left_up_position_y]
                        # 现在的g值 = 当前位置的g值加上移动所加的值
                        now_g = g[parent_position[0]][parent_position[1]] + 1.4
                        # 如果现在的g值比以前的g值小了，则修改g值，否则保持原来的g值
                        if now_g < origin_g:
                            g[left_up_position_x][left_up_position_y] = now_g
                    if left_up_position_x == end_position[0] and left_up_position_y == end_position[1]:
                        success_flag = True
                    # 保存当前位置左上角的区域的坐标
                    f_x[0][2] = left_up_position_x
                    f_y[0][2] = left_up_position_y
                    if debug_flag is True:
                        print "f_x[0][2]  = ", f_x[0][2]
                        print "f_y[0][2]  = ", f_y[0][2]
                        print "g[left_up_position_x][left_up_position_y]", g[f_x[0][2]][f_y[0][2]]
                        print "h[left_up_position_x][left_up_position_y]", h[f_x[0][2]][f_y[0][2]]
        # 上面和右边都不是障碍，右上角才可以通过
        if up_flag is False and right_flag is False:
            # 右上
            right_up_position_x = start_point[0] + 1
            right_up_position_y = start_point[1] + 1
            if debug_flag is True:
                print "right_up_position_x = ", right_up_position_x
                print "right_up_position_x = ", right_up_position_y
                print "create_2D_space[right_up_position_x][right_up_position_y] =", (
                   create_2D_space[right_up_position_x][right_up_position_y])
    
            if right_up_position_x <= n and start_point[1] + 1 <= n:
                if create_2D_space[right_up_position_x][right_up_position_y] != 2:  # 选中的方块不是障碍
                    # h值计算
                    right_up_h_value = (abs(end_position[0] - right_up_position_x) + abs(
                        end_position[1] - right_up_position_y)) * 1
                    h[right_up_position_x][right_up_position_y] = right_up_h_value
    
                    if open_list_S[right_up_position_x][right_up_position_y] != 1:
                        open_list_S[right_up_position_x][right_up_position_y] = 1
                        close_list_C[right_up_position_x][right_up_position_y] = 0
                        # 目前该位置的g 和 h的值的计算
                        # 现在的g值 = 当前位置的g值加上移动所加的值
                        now_g = g[parent_position[0]][parent_position[1]] + 1.4
                        g[right_up_position_x][right_up_position_y] = now_g
    
                    else:  # 该点在open_list_S里面，则应该判断上次的g值与本次的g值的大小，只保留小的g值
                        # 原来的g值是这个
                        origin_g = g[right_up_position_x][right_up_position_y]
                        # 现在的g值 = 当前位置的g值加上移动所加的值
                        now_g = g[parent_position[0]][parent_position[1]] + 1.4
                        # 如果现在的g值比以前的g值小了，则修改g值，否则保持原来的g值
                        if now_g < origin_g:
                            g[right_up_position_x][right_up_position_y] = now_g
    
                    if right_up_position_x == end_position[0] and right_up_position_y == end_position[1]:
                        success_flag = True
                    # 保存当前位置左上角的区域的坐标
                    f_x[0][4] = right_up_position_x
                    f_y[0][4] = right_up_position_y
                    if debug_flag is True:
                        print "f_x[0][4]  = ", f_x[0][4]
                        print "f_y[0][4]  = ", f_y[0][4]
                        print "g[right_up_position_x][right_up_position_y]", g[f_x[0][4]][f_y[0][4]]
                        print "h[right_up_position_x][right_up_position_y]", h[f_x[0][4]][f_y[0][4]]
        # 如果右边或下边是障碍，则右下角不能通过
        if right_flag is False and down_flag is False:
            # 右下
            right_down_position_x = start_point[0] + 1
            right_down_position_y = start_point[1] - 1
            if debug_flag is True:
                print "right_down_position_x = ", right_down_position_x
                print "right_down_position_y = ", right_down_position_y
                print "create_2D_space[right_down_position_x][right_down_position_y] =", (
                    create_2D_space[right_down_position_x][right_down_position_y])
    
            if right_down_position_x <= n and start_point[1] - 1 >= 1:
                if create_2D_space[right_down_position_x][right_down_position_y] != 2:  # 选中的方块不是障碍
                    # h值计算
                    right_down_h_value = (abs(end_position[0] - right_down_position_x) + abs(
                        end_position[1] - right_down_position_y)) * 1
                    h[right_down_position_x][right_down_position_y] = right_down_h_value
    
                    if open_list_S[right_down_position_x][right_down_position_y] != 1:
                        open_list_S[right_down_position_x][right_down_position_y] = 1
                        close_list_C[right_down_position_x][right_down_position_y] = 0
                        # 目前该位置的g 和 h的值的计算
                        # 现在的g值 = 当前位置的g值加上移动所加的值
                        now_g = g[parent_position[0]][parent_position[1]] + 1.4
                        g[right_down_position_x][right_down_position_y] = now_g
    
                    else:  # 该点在open_list_S里面，则应该判断上次的g值与本次的g值的大小，只保留小的g值
                        # 原来的g值是这个
                        origin_g = g[right_down_position_x][right_down_position_y]
                        # 现在的g值 = 当前位置的g值加上移动所加的值
                        now_g = g[parent_position[0]][parent_position[1]] + 1.4
                        # 如果现在的g值比以前的g值小了，则修改g值，否则保持原来的g值
                        if now_g < origin_g:
                            g[right_down_position_x][right_down_position_y] = now_g
    
                    if right_down_position_x == end_position[0] and right_down_position_y == end_position[1]:
                        success_flag = True
                    # 保存当前位置左上角的区域的坐标
                    f_x[0][6] = right_down_position_x
                    f_y[0][6] = right_down_position_y
                    if debug_flag is True:
                        print "f_x[0][6]  = ", f_x[0][6]
                        print "f_y[0][6]  = ", f_y[0][6]
                        print "g[right_down_position_x][right_down_position_y]", g[f_x[0][6]][f_y[0][6]]
                        print "h[right_down_position_x][right_down_position_y]", h[f_x[0][6]][f_y[0][6]]
        # 如果下边或左边是障碍，则左下角不能通过
        if down_flag is False and left_flag is False:
            # 左下
            left_down_position_x = start_point[0] - 1
            left_down_position_y = start_point[1] - 1
            if debug_flag is True:
                print "left_down_position_x = ", left_down_position_x
                print "left_down_position_y = ", left_down_position_y
                print "create_2D_space[left_down_position_x][left_down_position_y] =", (
                  create_2D_space[left_down_position_x][left_down_position_y])
    
            if left_down_position_x >= 1 and left_down_position_y >= 1:
                if create_2D_space[left_down_position_x][left_down_position_y] != 2:
                    # h值计算
                    left_down_h_value = (abs(end_position[0] - left_down_position_x) + abs(
                        end_position[1] - left_down_position_y)) * 1
                    h[left_down_position_x][left_down_position_y] = left_down_h_value
    
                    if open_list_S[left_down_position_x][left_down_position_y] != 1:
                        open_list_S[left_down_position_x][left_down_position_y] = 1
                        close_list_C[left_down_position_x][left_down_position_y] = 0
                        # 目前该位置的g 和 h的值的计算
                        # 现在的g值 = 当前位置的g值加上移动所加的值
                        now_g = g[parent_position[0]][parent_position[1]] + 1.4
                        g[left_down_position_x][left_down_position_y] = now_g
    
                    else:  # 该点在open_list_S里面，则应该判断上次的g值与本次的g值的大小，只保留小的g值
                        # 原来的g值是这个
                        origin_g = g[left_down_position_x][left_down_position_y]
                        # 现在的g值 = 当前位置的g值加上移动所加的值
                        now_g = g[parent_position[0]][parent_position[1]] + 1.4
                        # 如果现在的g值比以前的g值小了，则修改g值，否则保持原来的g值
                        if now_g < origin_g:
                            g[left_down_position_x][left_down_position_y] = now_g
    
                    if left_down_position_x == end_position[0] and left_down_position_y == end_position[1]:
                        success_flag = True
                    # 保存当前位置左上角的区域的坐标
                    f_x[0][8] = left_down_position_x
                    f_y[0][8] = left_down_position_y
                    if debug_flag is True:
                        print "f_x[0][8]  = ", f_x[0][8]
                        print "f_y[0][8]  = ", f_y[0][8]
                        print "g[left_down_position_x][left_down_position_y]", g[f_x[0][8]][f_y[0][8]]
                        print "h[left_down_position_x][left_down_position_y]", h[f_x[0][8]][f_y[0][8]]
        # 设置当前最优距离,记得要点无所谓，空间为1000X1000的时候，f很可能超过100000
        new_f = 1000000
        # 初始最有距离的位置坐标
        goal_x = 0
        goal_y = 0
        # 找出当前最优距离
        for k in range(1, 9):
            # 出现障碍的点就不选了，只选正常的点
            if f_x[0][k] != 0 and f_y[0][k] != 0:
                if new_f > (g[f_x[0][k]][f_y[0][k]] + h[f_x[0][k]][f_y[0][k]]):
                    new_f = g[f_x[0][k]][f_y[0][k]] + h[f_x[0][k]][f_y[0][k]]
        # 重新获取最优距离对应的坐标
        for k in range(1, 9):
            # 出现障碍的点就不选了，只选正常的点
            if f_x[0][k] != 0 and f_y[0][k] != 0:
                if new_f == (g[f_x[0][k]][f_y[0][k]] + h[f_x[0][k]][f_y[0][k]]):
                    goal_x = f_x[0][k]
                    goal_y = f_y[0][k]
                    # 只需要找出一组当前最优解就行
                    break
        #  获取当前最优距离
        # print "g[", goal_x, "][", goal_y, "] =", g[goal_x][goal_y] 
        # print "h[", goal_x, "][", goal_y, "] =", h[goal_x][goal_y]
        
        f_star = g[goal_x][goal_y] + h[goal_x][goal_y]
        f_star = round(f_star, 2) 
        # print "f_star =", f_star
        # 用列表保存4个数据,运行结束的标志，当前操作点的两个坐标，到目前位置的最有距离
        
        result_list.append(success_flag)
        result_list.append(goal_x)
        result_list.append(goal_y)
        result_list.append(f_star)
        if goal_x == 0 and goal_y == 0:
            print "parent_position =", parent_position
            print "goal_position =", goal_position
            print "程序出错"
            exit
    return result_list


def a_star_algorithm(first_coordinate, second_coordinate, obstacle_coordinate, obstacle_num, obstacle_flag):
    # print "program begin……"
    # 坐标自己定
    # start_position = [1, 1]  # 起始坐标
    # end_position = [1000, 1000]  # 目标坐标
    if obstacle_flag is True:
        print "每次操作暂时障碍时，先初始化"
        for i in range(0, len(temp_obstacle_x)):
            
            # print "temp_obstacle_x =", temp_obstacle_x
            # print "temp_obstacle_y =", temp_obstacle_y
            # print "len(temp_obstacle_x) =", len(temp_obstacle_x)
            if len(temp_obstacle_x) != 0:
                temp_obstacle_x.remove(temp_obstacle_x[0])
                temp_obstacle_y.remove(temp_obstacle_y[0])
        print "每次操作永久障碍时，先初始化"
        for i in range(0, len(perpetual_obstacle_x)):
            if len(perpetual_obstacle_x) != 0:
                perpetual_obstacle_x.remove(perpetual_obstacle_x[0])
                perpetual_obstacle_y.remove(perpetual_obstacle_y[0])
    start_position = first_coordinate
    end_position = second_coordinate
    
    # 程序初始化
    start_position_new =start_position
    end_position_new = end_position
    # print "start_position =", start_position 
    # print "end_position =", end_position
    x_0 = start_position[0]
    y_0 = start_position[1]
    x_1 = end_position[0]
    y_1 = end_position[1]
    start_position_new[0] = int(start_position_new[0])
    start_position_new[1] = int(start_position_new[1])
    end_position_new[0] = int(end_position_new[0])
    end_position_new[1] = int(end_position_new[1])
    
    init(start_position_new, end_position_new)
    # 设置障碍
    # print "开始设置障碍"
    # 为了在运动时避开障碍，设置两种状态
    # （1）在无障碍情况下计算距离
    # （2）在有障碍的情况下再计算一次距离
    # 如果两次距离相等，则中途没有障碍，可以直接到达目的位置
    # 反之，不相等，则中途出现障碍，不能直接到大目的地，需要改变运动方向
    # obstacle_flag = True 表明空间存在障碍
    if obstacle_flag is True:
        print "设置障碍"
        set_obstacle(obstacle_coordinate, obstacle_num)
    # print "结束设置障碍"
    
    if abs(start_position_new[0] - end_position_new[0]) < 1 and abs(start_position_new[1] - end_position_new[1]) < 1:
        # print "两个点都在同一个单位区域内"
        # print "start_position =", start_position
        # print "end_position =", end_position
        result_new = []
        result_new.append(True)
        result_new.append(end_position[0])
        result_new.append(end_position[1])
        # 利用欧几里得距离公式计算
        Distance = math.sqrt(math.pow((x_0-x_1),2)+math.pow((y_0-y_1),2))
        # 取两位小数
        Distance = round(Distance, 2)
        # print "同一个区域的距离为Distance =", Distance 
        result_new.append(Distance)
        result = result_new
        return result
    else:
        # print "两个点不在同一个单位区域内"
        start_position[0] = int(start_position[0])
        start_position[1] = int(start_position[1])
        end_position[0] = int(end_position[0])
        end_position[1] = int(end_position[1])
        # print "start_position =", start_position
        # print "end_position =", end_position
        parent = start_position
       
        g[parent[0]][parent[1]] = 0  # 目标的横坐标的g值
        h[parent[0]][parent[1]] = 0  # 目标的纵坐标的h值
        # print "result = ", result
        # 初始化标志位，位置坐标，还有距离
        result = [False, parent[0], parent[1], 0]
        # 保留这个初始点的值，用于后期检验结果用
        new_x = start_position[0]
        new_y = start_position[1]
        # print "start_position =", start_position
        # print "new_x =", new_x
        # print "new_y =", new_y
        while end_position[0] != parent[0] or end_position[1] != parent[1]:
            # 从二维平面里面操作
            # print "while()"
            # print "new_x =", new_x
            # print "new_y =", new_y
            # 因为g+h= f f的值一直没变，可能会回到原来的地方，需要将临时障碍释放为可以遍历的空间
            if new_x == parent[0] and new_y == parent[1]:
                print "说明又重新回到起点了"
                # print "延时5s"
                # time.sleep(5)
                for i in range(0, len(temp_obstacle_x)):
                    create_2D_space[temp_obstacle_x[0]][temp_obstacle_y[0]] = 0
                    temp_obstacle_x.remove(temp_obstacle_x[0])
                    temp_obstacle_y.remove(temp_obstacle_y[0])
            
            # print "temp_obstacle_x =", temp_obstacle_x
            # print "temp_obstacle_y =", temp_obstacle_y
            result = possibilities_position(parent, end_position)
            # result 中包含4个内容，找到目标的标志Flag，目标横坐标x，目标纵坐标y，距离的值distance
            # print "result =", result
            # 找到目标点后，终止程序
            if end_position[0] == parent[0] and end_position[1] == parent[1]:
                break
            # 将已经操作过的点，放入close_list_C[][]中，不再被查询
            # open_list_S[parent[0]][parent[1]] = 0
            # close_list_C[parent[0]][parent[1]] = 1
            
            # 从当前找出的最短距离的点进行操作
            parent[0] = result[1]
            parent[1] = result[2]
            # 打印每一步的操作的节点
            # print "产生的parent =", parent
            # if parent[0] == 0 and parent[1] == 0:
            #     print "表示终点在障碍区域内"
            #     print "程序出错，退出A*算法"
            #     break
        # print "the final result is ", result
        return result





