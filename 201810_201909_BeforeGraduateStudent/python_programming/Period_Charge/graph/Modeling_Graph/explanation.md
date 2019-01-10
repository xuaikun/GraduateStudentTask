### 20190110A_Star_Algorithm_problem.jpg
这张图片表明了个人实现的A*算法的不足，当进入类似（1，5）这样的死胡同后，应该可以再出来，并不是死磕在里面：
解决思路：
设置临时障碍：temp_obstacle
设置永久障碍：perpetual_obstacle
当走过的地方都设置为temp_obstacle
当走到类似（1，5）这种地方的时候，表明4面都是障碍，则将临时障碍全部释放为可遍历空间，即：取消障碍，并将（1，5）设置为永久障碍，当前运行的点不能再进入（1，5）

### 20190110A_Star_Algorithm_coordinate_space_set.jpg
该图表示，在A*算法中设置坐标以及空间设置

### 20190110Node_Motion_diagram.jpg
解释一个节点的如何运动

### 20190110Node_Motio_flowchart.jpg
描述节点坐标的改变过程