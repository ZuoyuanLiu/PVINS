# PVINS：Loosely Coupled PSD-Visual-Inertial Fusion for Drift-Correctable Positioning System
PVINS is an optimization-based multi-sensor state estimator that integrates a Position Sensitive Detector (PSD), stereo camera, and IMU. Experiments were conducted in real indoor environments to validate the effectiveness of our system and its drift-correctable capability.

![image](https://github.com/ZuoyuanLiu/PVINS/blob/master/output/fig6.tif)

**DemoShow**: [youtube]( https://youtu.be/Y1bcVBrt0Qk) or [bilibili]( https://www.bilibili.com/video/BV1F3WhegE9k/)

[![PL-VINS](https://img.youtube.com/vi/Y1bcVBrt0Qk/0.jpg)](https://youtu.be/Y1bcVBrt0Qk)

## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 20.04.
ROS  noetic. [ROS Installation](http://wiki.ros.org/ROS/Installation)


### 1.2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html).


## 2. Build PVINS
Clone the repository and catkin_make:
```
    cd ~/catkin_ws/src
    git clone https://github.com/ZuoyuanLiu/PVINS.git
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```
(if you fail in this step, try to find another computer with clean system or reinstall Ubuntu and ROS)

## 3. Run PVINS with D435i
```
    roslaunch realsense2_camera_stereo_camera.launch
    rosrun pubpsd_pkg pubpsd_node
    roslaunch vins_psd vins_psd_rviz.launch
    rosrun vins_psd vins_psd_node realsense_stereo_imu_config.yaml(cd config)
    rosrun global_fusion global_fusion_psd_node
```
## 4. Related Papers
- PVINS：Loosely Coupled PSD-Visual-Inertial Fusion for Drift-Correctable Positioning System.
This paper is developed based on VINS-Fusion [1].
```
[1] A General Optimization-based Framework for Local Odometry Estimation with Multiple Sensors
```
*If you find aforementioned works helpful for your research, please cite them.*

## 5. Acknowledgements

Thank Dr.Fengming Sun, Ruiyang Zhong, Zhenyu Zhu, and Dr.Xinyi Zhao; Dr.Tong Qin, Jie Pan, Shaozu Cao, and Shaojie Shen (VINS-Fusion) very much

## 6. License
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.

We are still working on improving the code reliability. For any technical issues, please contact Zuoyuan Liu <zuoyuanliu1999@163.com>.

For commercial inquiries, please contact Fengming Sun <fmsun@cauc.edu.cn>.
