# encoding: utf-8
import numpy as np

N_i = []
N_i.append(0)
for i in range(1, 11):
    N_i.append(i)
print "N_i =", N_i
while 1 != len(N_i):
    print "进入while()"
    print "删除前len(N_i) =", len(N_i)
    print "删除前N_i =", N_i
    print "N_i[1] =", N_i[1]
    N_i.remove(N_i[1])
    print "删除后N_i =", N_i
    print "删除后len(N_i) =", len(N_i)
print "退出while()"
