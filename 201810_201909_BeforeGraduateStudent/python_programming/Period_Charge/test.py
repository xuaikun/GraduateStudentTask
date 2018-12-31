# encoding: utf-8
import numpy as np
import random
import matplotlib.pyplot as plt
N = 2    # 假设我有N辆电单车
# 初始化电单车在二维空间中的坐标
N_x = np.empty([1, N], int)
N_y = np.empty([1, N], int)

# 随机在1000*1000的空间内生成N辆电单车的坐标
for i in range(0, N):
    N_x[0][i] = random.randint(1, 6)
    N_y[0][i] = random.randint(1, 6)

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

print "x_list[0] =", x_list
print "y_list[0] =", y_list
ax.scatter(x_list, y_list)
plt.plot(x_list, y_list)
plt.show()

#画散点图，以x_list中的值为横坐标，以y_list中的值为纵坐标
#参数c指定点的颜色，s指定点的大小,alpha指定点的透明度

# for i in range(1, N + 1):
#     for j in range(1,  N + 1):
        # ax.scatter(x_list[i], y_list[j], c='r', s=20, alpha=0.5)

