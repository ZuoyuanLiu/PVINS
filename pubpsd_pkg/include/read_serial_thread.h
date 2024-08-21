#pragma once
#ifndef TESTTHREAD_H
#define TESTTHREAD_H
// extern "C"
#include <ros/ros.h>
#include <pthread.h>
#include <queue>
#include <opencv2/core/core.hpp>

class testThread
{
public:
    //basic
    testThread();
    
    void startWork(int num);

    ~testThread();

private:

    std::queue<uint8_t> que;
    int NUM = 0;
    bool shutdown = 0;//1:close 0:open port

    /*光源信标坐标*/
    std::vector<cv::Point3f> pts_3d{{0.92,0.62,2.131},{0.54,0.25,2.131},{-0.09,0.76,2.131},{0.02,-0.31,2.131}};
    std::vector<cv::Point3f> ptsLed3d{{101.358,1142.72,2740.46},{95.3419,100.671,2896.53},{1142.35,104.612,2966.79},{1144.48,1148.1,2932.95}};
    
    
    pthread_t readID;
    pthread_t *threadIDs;
    pthread_t pnpID;


    pthread_mutex_t m_mutex;
    pthread_cond_t m_cond;//writeque使用的条件锁
    pthread_cond_t m_cond_pnp;//pnp使用的条件锁

    //function
    /*ctrl+c的退出机制是这样的，read_serial函数while的条件是ros::ok,ctrl+c后while结束运行，
    接下来就会执行ExitReadthread，并将串口关闭，shutdown置为true，其他线程会很据这个shutdown
    的状态，结束线程，所以当执行ctrl+c后首先read_serial进程退出，进而所有进程全部退出*/
    void ExitReadthread();//退出read_serial线程
    void ExitProcessThread();//退出dataProcess线程
    void ExitPnpThread();//退出opencv的PNP线程
    static void cam2pixel(const float px,const float py, const cv::Mat &K);//opencv的pnp需要用的函数
    void Matrix2Quaternion(const cv::Mat &matrix);//这个函数暂时用不到，实现了矩阵到四元数的转化


    //producer：从串口中读取数据
    static void* read_serial(void* arg);
    //consumer：将数据写入que解算光斑坐标并通过ceres优化获得PSD坐标
    static void* dataProcess(void* arg);
    //使用opencv带的pnp算法求解-没经过测试，能运行而已，想要使用记得在startWork函数中开启线程
    static void* solvePosition(void* arg);
    //ceres实现的坐标求解
    struct PnPCeres;
    //用于发布话题PSD的坐标信息
    ros::Publisher pub;
    //用于将坐标写入文件
};

#endif