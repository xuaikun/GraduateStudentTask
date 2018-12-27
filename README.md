# 遇到的问题：
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

python中运行continue后，i 每次增加1
continue 还在for内执行，break是跳出for并执行上方与for同级的程序
