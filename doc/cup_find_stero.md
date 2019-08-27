# find_cup_stero 说明
---
## 1.使用方法

### 1.1 安装

安装find_cup_ros包
```
cd catkin_ws
catkin_make
``` 

下载小觅相机驱动 https://github.com/slightech/MYNT-EYE-S-SDK 

安装教程： https://buildmedia.readthedocs.org/media/pdf/mynt-eye-s-sdk-docs-zh-cn/latest/mynt-eye-s-sdk-docs-zh-cn.pdf
### 1.2 运行 

#### 测试USB相机 

```
roslaunch find_cup_ros find_cup_mynt_stereo.launch
```

--- 

## 3. 双目相机原理介绍

小觅相机基线为12cm

根据d=x*b/f