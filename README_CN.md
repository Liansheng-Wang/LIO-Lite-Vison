# LIO-Lite
一个可以用在无人机上的轻量级 LIO 系统：适配了Livox Mid-360雷达。  
* 整个项目是在[faster-lio](https://github.com/gaoxiang12/faster-lio.git)的基础上做适配.[faster-lio](https://github.com/gaoxiang12/faster-lio.git)纯度：98%
* 增加了场景中的重定位  

## 分支
分出了基于优化的分支和基于滤波的分支。  
基于优化的分支在X86架构上测试表现良好，但是在英伟达NX上帧率只有5hz不到。  
基于滤波的算法在英伟达NX上表现不错，运行起来CPU总占用在30%~40%。除去一些基础的占用，总占用应该更低。

## Illustrate
c++ == 17  

## Dependence
1. ROS (melodic or noetic)
2. glog: ```sudo apt-get install libgoogle-glog-dev```
3. eigen: ```sudo apt-get install libeigen3-dev```
4. pcl: ```sudo apt-get install libpcl-dev```
5. yaml-cpp: ```sudo apt-get install libyaml-cpp-dev```

## Build
```
  git clone https://github.com/Liansheng-Wang/LIO-Lite.git  
  cd LIO-Lite  
  catkin_make  
```

## Run
```
  source devel/setup.zsh
  roslaunch lvio_lite mapping_360.launch  
```
在完成建图之后，可以使用下面的命令开启重定位模式：
```
  roslaunch lvio_lite location_360.launch  
```
### note：
  重定位过程中如果不是在起点的时，需要在rviz中手动进行重定位  
  或者在 mid360_location.yaml 中参数进行修改：rpy
```
  init_trans: [0, 0, 0]
  init_rpy: [0, 0, 0]
```

## Reference
* [FAST-LIO2](https://github.com/hku-mars/FAST_LIO.git)
* [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM.git)
* [faster-lio](https://github.com/gaoxiang12/faster-lio.git)
* [slam_in_autonomous_driving](https://github.com/gaoxiang12/slam_in_autonomous_driving.git)

## UPDATE LOG:
### 2023-10-17 
在地图保存的部分增加了地图分割的部分。   
在重定位模式可以选择增量加载地图，在大场景下的轻量定位比较合适。   
可以根据场景的具体大小和计算平台的算力来调整分割地图的尺寸和重定位模式下的ivox的容量。  
在构建完地图的时候，可以使用以下的命令来手动测试地图的分割与加载过程。   
#### Test:
```
rosrun lvio_lite test_split
rosrun lvio_lite test_load 
```
![SplitMap](https://github.com/Liansheng-Wang/LIO-Lite/blob/eskf-base/doc/split_map.png)

### 2023-11-22
增加了在初始化阶段以重力为约束定义水平面.  

### 2023-11-23
测试了 HesaiXT-16 雷达.
![HesaiTX16](https://github.com/Liansheng-Wang/LIO-Lite/blob/eskf-base/doc/HesaiXT16.png)