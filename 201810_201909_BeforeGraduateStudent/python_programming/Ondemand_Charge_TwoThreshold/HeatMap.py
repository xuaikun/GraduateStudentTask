# encoding: utf-8  
import xlrd
import numpy as np
import numpy.random
import seaborn as sns
import matplotlib.pyplot as plt

def draw_heatmap(x, y):
	# 数据规整
	for i in range(0, len(x)):
		x[i] = int(x[i])
	for i in range(0, len(y)):
		y[i] = int(y[i])

	# 找出x或者y的最大值，作为热力图矩阵的边长
	lenght = max(max(x), max(y))
	# 初始化热力图矩阵
	z = np.zeros([lenght + 1, lenght + 1], dtype = np.int)
	# 将坐标下x,y的值对应的点的值进行相加，不存在的坐标对应值为0
	for i in range(0, len(x)):
		z[y[i]][x[i]] = z[y[i]][x[i]] + 1

	# 图片大小
	f, ax1 = plt.subplots(figsize=(10, 10))

	# 生成热力图
	sns.heatmap(z, ax=ax1, cmap='GnBu', square = True)
	# plt.imshow(z, cmap= 'GnBu', origin='low')
	# plt.colorbar()
	# 保存热力图
	plt.savefig("heatmap_ass_mx_test.png", dpi=600)
	plt.show()

def main():
	# 读取excel文档的数据,如果用的是其它文档可能得修改一下
	data = xlrd.open_workbook("data.xlsx")
	# 读取第一张表的数据
	table = data.sheets()[0]
	# 读取excel文档的第一列 X
	x = table.col_values(0)
	lenght = len(x)
	# 第一个值不用
	x = x[1:]

	# 读取excel文档的第二列 Y
	y = table.col_values(1)
	# 第一个值不用
	y = y[1:]
	draw_heatmap(x, y)

if __name__ == "__main__":
	main()
	
