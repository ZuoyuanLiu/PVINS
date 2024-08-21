/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include "ros/ros.h"
#include "globalOpt.h"
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <stdio.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <fstream>
#include <queue>
#include <mutex>
//新添加
#include <geometry_msgs/PointStamped.h>


GlobalOptimization globalEstimator;
ros::Publisher pub_global_odometry, pub_global_path, pub_car;
nav_msgs::Path *global_path;
double last_vio_t = -1;
std::queue<geometry_msgs::PointStamped> psdQueue;
std::mutex m_buf;

void publish_car_model(double t, Eigen::Vector3d t_w_car, Eigen::Quaterniond q_w_car)
{
    visualization_msgs::MarkerArray markerArray_msg;
    visualization_msgs::Marker car_mesh;
    car_mesh.header.stamp = ros::Time(t);
    car_mesh.header.frame_id = "world";
    car_mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
    car_mesh.action = visualization_msgs::Marker::ADD;
    car_mesh.id = 0;

    car_mesh.mesh_resource = "package://global_fusion/models/car.dae";

    Eigen::Matrix3d rot;
    rot << 0, 0, -1, 0, -1, 0, -1, 0, 0;
    
    Eigen::Quaterniond Q;
    Q = q_w_car * rot; 
    car_mesh.pose.position.x    = t_w_car.x();
    car_mesh.pose.position.y    = t_w_car.y();
    car_mesh.pose.position.z    = t_w_car.z();
    car_mesh.pose.orientation.w = Q.w();
    car_mesh.pose.orientation.x = Q.x();
    car_mesh.pose.orientation.y = Q.y();
    car_mesh.pose.orientation.z = Q.z();

    car_mesh.color.a = 1.0;
    car_mesh.color.r = 1.0;
    car_mesh.color.g = 0.0;
    car_mesh.color.b = 0.0;

    float major_scale = 0.05;//设置汽车模型的尺寸

    car_mesh.scale.x = major_scale;
    car_mesh.scale.y = major_scale;
    car_mesh.scale.z = major_scale;
    markerArray_msg.markers.push_back(car_mesh);
    pub_car.publish(markerArray_msg);
}
//估计这里都用不到，接收过来的信息就不是经纬度，直接就是xyz
void PSD_callback(const geometry_msgs::PointStamped &PSD_msg)
{
    //printf("PSD_callback! \n");
    m_buf.lock();
    psdQueue.push(PSD_msg);
    m_buf.unlock();
}

void vio_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    //printf("vio_callback! \n");
    double t = pose_msg->header.stamp.toSec();
    last_vio_t = t;
    // 获取VIO输出的位置(三维向量)，姿态(四元数)
    Eigen::Vector3d vio_t(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z);
    Eigen::Quaterniond vio_q;
    vio_q.w() = pose_msg->pose.pose.orientation.w;
    vio_q.x() = pose_msg->pose.pose.orientation.x;
    vio_q.y() = pose_msg->pose.pose.orientation.y;
    vio_q.z() = pose_msg->pose.pose.orientation.z;
    // 位姿传入global Estimator中
    globalEstimator.inputOdom(t, vio_t, vio_q);


    m_buf.lock();
    // 寻找与VIO时间戳相对应的GPS消息
    while(!psdQueue.empty())
    {
        // 获取最老的GPS数据和其时间
        geometry_msgs::PointStamped PSD_msg = psdQueue.front();
        double psd_t = PSD_msg.header.stamp.toSec();
        // 10ms sync tolerance
        // +- 10ms的时间偏差
        printf("vio t: %f, gps t: %f \n", t, psd_t);
        // TODO 10ms sync tolerance,PSD的更新速率更高，100HZ的话5ms的误差吧
        if(psd_t >= t - 0.01 && psd_t <= t + 0.01)
        {
            // gps的经纬度，海拔高度
            //printf("receive GPS with timestamp %f\n", PSD_msg->header.stamp.toSec());
            double psd_x = PSD_msg.point.x;
            double psd_y = PSD_msg.point.y;
            double psd_z = PSD_msg.point.z;
            // 向globalEstimator中输入GPS数据
            globalEstimator.inputPSD(t, psd_x, psd_y, psd_z);
            psdQueue.pop();
            // PSD的位姿写入文本文件
            std::ofstream foutP("/home/robot/catkin_PsdVinsFusion/src/VINS-Fusion/output/psd.csv", ios::app);//续写
            foutP.setf(ios::fixed, ios::floatfield);
            foutP.precision(0);
            foutP << pose_msg->header.stamp.toSec() * 1e9 << ",";
            foutP.precision(5);
            foutP << psd_x << ","<< psd_y << ","<< psd_z << endl;
            foutP.close();
            break;//此处break,意味只存储了一个GPS数据后就break了
        }
        else if(psd_t < t - 0.005)
            psdQueue.pop();
        else if(psd_t > t + 0.005)
            break;
    }
    m_buf.unlock();

    Eigen::Vector3d global_t;
    Eigen:: Quaterniond global_q;
    globalEstimator.getGlobalOdom(global_t, global_q);

    nav_msgs::Odometry odometry;
    odometry.header = pose_msg->header;
    odometry.header.frame_id = "world";
    odometry.child_frame_id = "world";
    odometry.pose.pose.position.x = global_t.x();
    odometry.pose.pose.position.y = global_t.y();
    odometry.pose.pose.position.z = global_t.z();
    odometry.pose.pose.orientation.x = global_q.x();
    odometry.pose.pose.orientation.y = global_q.y();
    odometry.pose.pose.orientation.z = global_q.z();
    odometry.pose.pose.orientation.w = global_q.w();
    // 广播小车，即融合后轨迹
    pub_global_odometry.publish(odometry);
    pub_global_path.publish(*global_path);
    publish_car_model(t, global_t, global_q);
    // 融合后的位姿写入文本文件
    std::ofstream foutC("/home/robot/catkin_PsdVinsFusion/src/VINS-Fusion/output/vio_global.csv", ios::app);
    foutC.setf(ios::fixed, ios::floatfield);
    foutC.precision(0);
    foutC << pose_msg->header.stamp.toSec() * 1e9 << ",";
    foutC.precision(5);
    foutC << global_t.x() << ","
            << global_t.y() << ","
            << global_t.z() << ","
            << global_q.w() << ","
            << global_q.x() << ","
            << global_q.y() << ","
            << global_q.z() << endl;
    foutC.close();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "globalEstimator");
    ros::NodeHandle n("~");

    global_path = &globalEstimator.global_path;

    ros::Subscriber sub_PSD = n.subscribe("/psdData", 100, PSD_callback);//pubpsd_node发布的psd消息话题
    ros::Subscriber sub_vio = n.subscribe("/vins_estimator/odometry", 100, vio_callback);
    pub_global_path = n.advertise<nav_msgs::Path>("global_path", 100);
    pub_global_odometry = n.advertise<nav_msgs::Odometry>("global_odometry", 100);
    pub_car = n.advertise<visualization_msgs::MarkerArray>("car_model", 1000);
    ros::spin();
    return 0;
}
