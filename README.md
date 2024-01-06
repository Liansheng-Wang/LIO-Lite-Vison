# LIO-Lite
A lightweight LIO (Lidar Odometry) system was deployed on a UAV utilizing the Livox Mid-360 sensor.  
* The project incorporates incremental refinements to the existing [faster-lio](https://github.com/gaoxiang12/faster-lio.git) framework.  
* Add location node.

## classification
This project has several different branches. The optimize the based branch, using graph optimization. Testing on the X86 is OK, but the processing speed on the Nvidia NX is very low.  
ESKF based performs well on Nvidia NX.

## Illustrate
c++ == 17
The project is developed using C++17 standard.  

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
After rebuilding the map, you can follow the following command to load the map for relocation  
```
  roslaunch lvio_lite location_360.launch  
```

### note:
If it is not at the starting point during the repositioning process, it needs to be manually repositioned in rviz  
Or modify the parameters in in mid360_location.yaml:
```
  init_trans: [0, 0, 0]
  init_rqy: [0, 0, 0]
```

## Reference
* [FAST-LIO2](https://github.com/hku-mars/FAST_LIO.git)
* [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM.git)
* [faster-lio](https://github.com/gaoxiang12/faster-lio.git)
* [slam_in_autonomous_driving](https://github.com/gaoxiang12/slam_in_autonomous_driving.git)



## UPDATE LOG:
### 2023-10-17 
Added map segmentation to avoid memory overload in large scenes and improve efficiency.   
Incremental loading of the map can be selected in the case of relocation.   

#### Test:
```
rosrun lvio_lite test_split
rosrun lvio_lite test_load 
```

![SplitMap](https://github.com/Liansheng-Wang/LIO-Lite/blob/eskf-base/doc/split_map.png)

### 2023-11-22
Added the use of gravity as a constraint to define the horizontal plane during the initialization phase.

### 2023-11-23
Test HesaiXT-16 Lidar.
![HesaiTX16](https://github.com/Liansheng-Wang/LIO-Lite/blob/eskf-base/doc/HesaiXT16.png)