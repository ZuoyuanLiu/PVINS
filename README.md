# Toward Large-Scale PSD-based Visible Light Positioning with Visual-Inertial Fusion


![image](https://github.com/ZuoyuanLiu/PVINS/blob/master/output/figure9.tif)


**DemoShow**: [youtube](https://youtu.be/7j6x5J23VpY) or [bilibili](https://www.bilibili.com/video/BV1KrmmYMEEs/)

[![PL-VINS](https://img.youtube.com/vi/7j6x5J23VpY/0.jpg)](https://youtu.be/7j6x5J23VpY)

## 1. Prerequisites
1.1 **Ubuntu** and **ROS**

Ubuntu 18.04. ROS Melodic, please google it.

1.2. **Dependency**

Eigen 3.3.4 + OpenCV 3.2+ Cere-solver: [Ceres Installation](http://ceres-solver.org/installation.html), remember to **sudo make install**.

## 2. Build PVINS on ROS
Clone the repository and catkin_make (# note that you will create a new workspace named *catkin_plvins*):
```
	mkdir -p ~/catkin_pvins/src    
	cd ~/catkin_pvins/
	catkin_make
	source devel/setup.bash
	echo $ROS_PACKAGE_PATH             # test you have created it successfully
	git clone https://github.com/ZuoyuanLiu/PVINS.git
```

```	
	catkin_make
	source devel/setup.bash
```

## 3. Run with D435i


run in the ~/catkin_pvins/
```
	roslaunch realsense2_camera rs_stereo_camera.launch   #open camera and imu
	roslaunch vins_psd vins_psd_rviz.launch
	rosrun vins_psd vins_psd_node realsense_stereo_inu_config.yaml
	rosrun global_fusion global_fusion_psd_node
```

Now you should be able to run PVINS in the ROS RViZ. 


This paper is developed based on VINS-Fusion[1].
```
[1] A General Optimization-based Framework for Local Odometry Estimation with Multiple Sensors

```

*If you find aforementioned works helpful for your research, please cite them.*

## 4. Acknowledgements

Thanks Ruiyang Zhong, Dr Zhenyu Zhu,Prof. Hongguang Liu and Dr Xinyi Zhao; Dr. Qin Tong and Prof. Shen (VINS-Fusion) very much.

## 5. Licence
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.
We are still working on improving the code reliability. For any technical issues, please contact Zuoyuan Liu.

For commercial inquiries, please contact Fengming Sun.
