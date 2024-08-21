#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <fstream>
#include <thread>
#include <queue>
#include <mutex>
#include <geometry_msgs/PointStamped.h>
#include "../include/read_serial_thread.h"


int main(int argc, char** argv)
{
    
    ros::init(argc, argv, "psd_data");
    /*创建句柄（虽然后面没用到这个句柄，但如果不创建，运行时进程会出错）*/
    ros::NodeHandle nh_main;
    testThread test;//定义线程，startWork开始执行
    test.startWork(3);//数字表示开启write_queue线程的数量，zry设置的5,用1不行，可能缓冲区的数据太多了，不更新位置信息

    // sleep(100);
    ros::Rate loop_rate0(100); // 控制消息发布频率为100Hz（如果解算的速率慢就发布重复的）。和sleep(x)作用一样
    while (ros::ok())
    {
    ros::spinOnce();
    loop_rate0.sleep();
    }
    return 0;       
    
}