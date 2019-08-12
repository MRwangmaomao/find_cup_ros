# find_cup_ros 说明
---
## 1.使用方法

### 1.1 安装
```
cd catkin_ws
catkin_make
```

## 2. 功能介绍

### 2.1 基于二维码的杯子定位与检测
[说明文档](doc/cup_find_apriltag.md)

### 2.2 已知杯子高度和桌子高度，基于单目的定位
[说明文档](doc/cup_find_general.md)

### 2.3 基于单目的视觉测量
[说明文档](doc/cup_find_measure.md)

### 2.4 基于双目的杯子定位与视觉测量
[说明文档](doc/cup_find_stero.md)

---
## 3. 椭圆检测原理和方法

参考博客：https://blog.csdn.net/txlqy9041/article/details/83240897

椭圆检测程序源代码：https://github.com/h3ct0r/fast_ellipse_detector