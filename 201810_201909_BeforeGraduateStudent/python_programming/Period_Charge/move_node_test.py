# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time
'''
fig, ax = plt.subplots()
xdata, ydata = [], []
ln, = ax.plot([], [], 'r-', animated=False)
#然后，构造开始帧函数init
def init():
    #我们的数据是一个0~2π内的正弦曲线
    # x的变化范围
    ax.set_xlim(300, 1000)
    # y的变化范围
    ax.set_ylim(300, 1000)
    return ln,
    
#接着，构造自定义动画函数animate，用来更新每一帧上各个x对应的y坐标值，参数表示第frame帧
def update(frame):
    xdata.append(frame)
    ydata.append(frame)
    # 更新当前位置数据
    ln.set_data(xdata, ydata)
    return ln,
#接下来，我们调用FuncAnimation函数生成动画。参数说明：
#fig 进行动画绘制的figure
#func 自定义动画函数，即传入刚定义的函数animate
#frames 动画长度，一次循环包含的帧数
#init_func 自定义开始帧，即传入刚定义的函数init
#interval 更新频率，以ms计
#blit 选择更新所有点，还是仅更新产生变化的点。应选择True，但mac用户请选择False，否则无法显示动画

# ani = FuncAnimation(fig, update, frames=np.linspace(0, 2*np.pi, 128),
#                     init_func=init, interval=100, blit=True)
ani = FuncAnimation(fig, update, frames=500,
                   init_func=init, interval=10, blit=True)
'''

#plt.grid()
# plt.plot()


#当然，你也可以将动画以mp4格式保存下来，但首先要保证你已经安装了ffmpeg 或者mencoder
# ani.save('basic_animation.mp4', fps=30, extra_args=['-vcodec', 'libx264'])
import matplotlib.pyplot as plt
plt.ion()    # 打开交互模式
# 同时打开两个窗口显示图片

plt.figure()
plt.plot()
print "延时5s"
time.sleep(5)
plt.figure()
plt.plot()
print "延时5s"
time.sleep(5)
# 显示前关掉交互模式
plt.ioff()
plt.show()

