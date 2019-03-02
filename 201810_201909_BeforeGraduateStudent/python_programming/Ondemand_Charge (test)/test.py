# encoding: utf-8
i = 0
print "i =", i
i = 0
x = [3,2,3,4,5,6,1]
for i in range(1, 10):
    if x[i] == 3:
        q = x[0]
        x[0] = 0
        x.remove(3)
        x[0] = q
        break
print "over"