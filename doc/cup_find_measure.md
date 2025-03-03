# find_cup_measure 说明
---
## 1.使用方法

### 1.1 安装
```
cd catkin_ws
catkin_make
``` 

### 1.2 单个杯子定位以及直径测量 

#### 打开相机
```
roslaunch usb_cam usb_cam-test.launch （根据自己的相机驱动进行修改)
```

#### 启动find_and_measure_cup_usbcam.launch

```
roslaunch find_cup_ros find_and_measure_cup_usbcam.launch
```

#### 发布/camera2world消息（修改相机相对于机械臂基座坐标系的位姿关系，下面只是参考，具体使用需要根据相机在机械臂上的位置以及机械臂的姿态来确定）

```
rostopic pub /camera2world geometry_msgs/Pose "position:
  x: 0.0
  y: 0.0
  z: 0.0
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 0.0"
```
#### 发布/CupInfo消息（在线修改杯子的直径和高度）

```
rostopic pub /CupInfo ...
```

```
rostopic pub /camera2world geometry_msgs/Pose "position:
  x: 0.0
  y: 0.0
  z: 0.0
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 0.0"
```


### 1.3 多个杯子检测与定位
#### 运行

```
roslaunch usb_cam usb_cam-test.launch (根据相机修改)

roslaunch find_cup_ros find_measure_multi_cup.launch

rostopic pub /camera2world geometry_msgs/Pose "position:
  x: 0.12998
  y: 0.36881
  z: 0.36088
orientation:
  x: 0.91949
  y: -0.38776
  z: 0.033006
  w: -0.05565"

```

#### 运行效果

![multi_cup_find](img/multi_cup_find.png)
![multi_cup_find](img/multi_cup_find2.png)



#### 检测到多个杯子后输出的消息内容
```
header: 
  seq: 26
  stamp: 
    secs: 1565660793
    nsecs: 826471063
  frame_id: ''
detections: 
  - 
    size: []
    pose: 
      position: 
        x: 0.132457905733
        y: 0.376505383845
        z: 0.0736
      orientation: 
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
    center_point: [312, 296]
  - 
    size: []
    pose: 
      position: 
        x: 0.146763719348
        y: 0.508798735806
        z: 0.0736
      orientation: 
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
    center_point: [139, 85]
  - 
    size: []
    pose: 
      position: 
        x: 0.252507131881
        y: 0.378282139733
        z: 0.0736
      orientation: 
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
    center_point: [481, 117]
  - 
    size: []
    pose: 
      position: 
        x: 0.142525551278
        y: 0.30552966315
        z: 0.0736
      orientation: 
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
    center_point: [437, 388]
  - 
    size: []
    pose: 
      position: 
        x: 0.190660747487
        y: 0.343137736111
        z: 0.0736
      orientation: 
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
    center_point: [448, 257]
```

`

---
## 2. 原理

### 2.1 检测椭圆与三维定位

参考： [cup_find_general.md](cup_find_general.md)

### 2.2 根据图像检测到的像素直径计算实际物理直径
 
u和v是像素平面，f是相机光轴。
 
![ppt](img/ppt3.jpg)

 
f是相机的焦距（已知）

R是杯子的物理尺寸（已知）

b是杯子的像素尺寸（已知）

l是相机光心到杯子中心的重力方向投影的距离（已知）

n轴为上图的绿色直线方向，Z轴为相机的光轴方向

