clc
clear
close all
data = load('Edge.mat')
dlmwrite('what.txt', data.Edge);
type('what.txt')
