# encoding: utf-8
import numpy as np
import random
import matplotlib.pyplot as plt
N = 5    # 假设我有N辆电单车
edge_n = 7
# 初始化电单车在二维空间中的坐标
N_x = np.empty([1, N], int)
N_y = np.empty([1, N], int)

# 随机在1000*1000的空间内生成N辆电单车的坐标
for i in range(0, N):
    N_x[0][i] = random.randint(1, edge_n)
    N_y[0][i] = random.randint(1, edge_n)

for i in range(0, N):
    print "坐标为：",(N_x[0][i],N_y[0][i])
x = N_x[0]
y = N_y[0]
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
# 将所有节点一一连起来
for i in range(0, N):
    for j in range(0, N):
        # 两两连接
        x_temp = [x_list[i], x_list[j]]
        y_temp = [y_list[i], y_list[j]]
        plt.plot(x_temp, y_temp)
plt.xlim([0 - 1,edge_n + 1]) #设置绘图X边界                                                                                                   
plt.ylim([0 - 1,edge_n + 1]) #设置绘图Y边界

plt.show()

ax.scatter(x_list, y_list,color = 'red', marker = 'o')
print 'x_list =', x_list
print 'y_list =', y_list
# 将节点连接构成回路
for i in range(0, N- 1):
    # 前后连接
    x_temp = [x_list[i], x_list[i+1]]
    y_temp = [y_list[i], y_list[i+1]]
    plt.plot(x_temp, y_temp)
    if i == N - 2:
        print 'i =', i
        x_temp = [x_list[i + 1], x_list[0]]
        y_temp = [y_list[i + 1], y_list[0]]
        plt.plot(x_temp, y_temp)
plt.xlim([0 - 1,edge_n + 1]) #设置绘图X边界                                                                                                   
plt.ylim([0 - 1,edge_n + 1]) #设置绘图Y边界
plt.show()

#画散点图，以x_list中的值为横坐标，以y_list中的值为纵坐标
#参数c指定点的颜色，s指定点的大小,alpha指定点的透明度

# for i in range(1, N + 1):
#     for j in range(1,  N + 1):
        # ax.scatter(x_list[i], y_list[j], c='r', s=20, alpha=0.5)

