# encoding: utf-8
# 算法功能：构造1-覆盖传感器网络的可调度充电回路集合
# 输入变量：N = {N1,N2,……,Nn}, S
# 输出变量：P = {P1,P2,P3……Pz}
# 1.z = 1, i = 1
# 构造遍历整个网络的哈密尔顿回路H
# 2.在NUS上应用LKH算法得到遍历所有节点的哈密尔顿回路H
# 3.将所有及节点重新编号H=<N0, N1, ……， Nn>,并令N0 = S
# 4.while i <= n do
#   N0为所有充电回路的起点
# 5.    将N0加入充电回路Pz
#   根据VPGS条件，贪心的将H分解成z个子回路
# 6.    for j = i to n do
# 7.        将Nj加入充电回路Pz
# 8.        在Pz上应用充电回路可调度性判定算法
# 9.        if 算法返回失败状态 then
# 10.           将Nj从充电回路Pz中去除
# 11.           i = j, z++
# 12.           break
# 13.       endif
# 14.   endfor
# 15.endwhile
