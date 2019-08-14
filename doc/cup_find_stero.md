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
## 2. 原理

### 


## 3. 双目相机原理介绍

a. Camera.bf中的b指基线baseline（单位：米），
f是焦距fx（x轴和y轴差距不大），bf=b*f，
和ThDepth一起决定了深度点的范围：
bf * ThDepth / fx
即大致为b * ThDepth。
基线在双目视觉中出现的比较多，
如ORB-SLAM中的双目示例中的EuRoC.yaml中的bf为47.9，ThDepth为35，fx为435.2，
则有效深度为bf * ThDepth / fx = 47.9*35/435.3=3.85米；
KITTI.yaml中的bf为387.57，ThDepth为40，fx为721.54，
则有效深度为387.57*40/721.54=21.5米。
这里的xtion的IR基线（其实也可以不这么叫）bf为40，
ThDepth为50，fx为558.34，则有效深度为3.58米（官方为3.5米）。 