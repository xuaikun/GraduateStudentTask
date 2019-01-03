# 保研后~研究生开学前的记录

### 2018年11月18日：
开始我的研究生学习，跟师姐做事，听老师的话，好好加油，要发厉害的文章，要拿奖学金，可以为家里省下很多。

### 2018年12月17日：
师姐发了个任务，好好写哦~

## 按周期进行充电

### 2018年12月27日：
实现了按周期充电的方式的构造充电回路
遇到了以前的问题：
while(1):
	x = 1
	print "x = ", x
	for i in range(x, 10):
		print "i =", i
		break
python中运行break后，i 一直= 1

while(1):
	x = 1
	print "x = ", x
	for i in range(x, 10):
		print "i =", i
		continue
continue 还在for内执行，break是跳出for并执行上方与for同级的程序
python中运行continue后，i 每次增加1

### 2019年1月2日：
今天主要完成将离散的点进行连接，并将排序后的点构成一个回路，本次工作主要将数据在二维空间中进行可视化

### 2019年1月3日：
今天主要完成数据范围的确定，不然数据比较乱，不确定，变化范围大。
将A_Star算法中的单位用cm表示，真正输入坐标为m，隐藏一个单位换算的情况。
start_position =[] 
    start_position.append(int(start_position_new[0]*100.0))
    start_position.append(int(start_position_new[1]*100.0))
单位由m转换为cm时，记得乘以100.0不能乘以100，不然数据有可能减小，隐藏着四舍五入的情况。