# encoding: utf-8
import math
Ax = -1.0/2.0
Ay = 1.0/2.0
Bx = 1
By = 0
Cx = 0
Cy = 0
dAB = math.sqrt(math.pow((Ax - Bx), 2) + math.pow((Ay - By), 2))
dAC = math.sqrt(math.pow((Ax - Cx), 2) + math.pow((Ay - Cy), 2))
dBC = math.sqrt(math.pow((Cx - Bx), 2) + math.pow((Cy - By), 2))
print "dAB =", dAB
print "dAC =", dAC
print "dBC =", dBC
# 选择插入算法
cosr = (math.pow(dAC, 2) + math.pow(dBC, 2) - math.pow(dAB, 2))/2*dAC*dBC
print "cosr =", round(cosr, 2)