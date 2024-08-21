#include <ros/ros.h>
#include <serial/serial.h>
#include <geometry_msgs/PointStamped.h>
#include <string.h>
#include <unistd.h>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "../include/read_serial_thread.h"
#include <ceres/ceres.h>
#include <chrono>
#include "/usr/local/include/ceres/rotation.h"
#include <iostream>
#include <fstream>
#include <random>
uint8_t tmp = 0;
/*相机PNP参数：*/
cv::Mat K_piexl = (cv::Mat_<float>(3, 3) << 344.30, 0, 369.22, 0, 212.8775, 211.7385, 0, 0, 1);
std::vector<cv::Point2f> ptsPiexl_2d;
cv::Point2f Position2d;
float dx = 0.029044;//转换图像尺寸为430×429
float dy = 0.027084;
float qw ,qx ,qy ,qz;

/*ceres优化参数：（已成功实现100HZ位姿更新速率）*/
//三自由度估计包括x、y以及绕z轴旋转，Z的位置为120-130之间的随机数
cv::Mat K_real = ( cv::Mat_<double> ( 3,3 ) << 5.4, 0, 0.2, 0, 5.4, -0.1, 0, 0, 1);//16point
double R_euroc3[1]={3.138};//绕z轴需要估计
double R_euroc[3]={-0.002,0.03,3.138};//绕x和y轴的欧拉角作为固定值，不再参与优化
double Psd[3]={20,20,125};//PSD的xy作为优化变量，z固定不变
double Psd_z[1]={125};
std::vector<cv::Point2f> ptsReal_2d;

testThread::testThread()
{
    std::cout<<"start testThread"<<std::endl;
}
testThread::~testThread()
{
    std::cout<<"end read_serial thread"<<std::endl;
    shutdown = true;
    pthread_join(readID, NULL);
    pthread_join(pnpID, NULL);
    for(int i = 0;i < NUM; i++){
        pthread_cond_signal(&m_cond);
    }
    // pthread_join(pnpID, NULL);

    if(threadIDs){
        delete[] threadIDs;
    }


    pthread_mutex_destroy(&m_mutex);
    pthread_cond_destroy(&m_cond);
    pthread_cond_destroy(&m_cond_pnp);
}
void testThread::startWork(int num)
{
    std::cout<<"start startWork()"<<std::endl;
    ros::NodeHandle nh;
    pub = nh.advertise<geometry_msgs::PointStamped>("psdData",10);//20表示的消息缓冲队列的消息数量

    // 确保 pub 初始化完成
    if (!pub) {
    ROS_ERROR("Publisher not initialized in main.cpp");
    return;
    }
    do
    {
        this->NUM = num;
        threadIDs = new pthread_t[num];
        if(threadIDs == nullptr){
            std::cout << "threadIDs fail" << std::endl;
            break;
        }
        memset(threadIDs, 0, sizeof(pthread_t) * num);
        if(pthread_mutex_init(&m_mutex,NULL) !=0 ||
        pthread_cond_init(&m_cond,NULL) != 0 ||
        pthread_cond_init(&m_cond_pnp,NULL) !=0){
            std::cout<<" mutex and cond fail"<<std::endl;
            break;
        }

        //三个线程锁对que这个公共队列中的数据进行处理
        pthread_create(&readID, nullptr ,read_serial,this);
        for(int i = 0 ; i < num; i++){
            pthread_create(&threadIDs[i],nullptr ,dataProcess,this);
        }
        // pthread_create(&pnpID, nullptr ,solvePosition,this);//solvePnp线程
        return;
    } while (0);

    if(threadIDs){
        delete[] threadIDs;
        threadIDs = nullptr;//重置指针以免重复释放
        }
}
struct testThread::PnPCeres
{
    // _uv是坐标点对，_xyz是世界坐标即第一帧相机坐标
    PnPCeres ( cv::Point2f uv,cv::Point3f xyz ) : _uv(uv),_xyz(xyz) {}//Psd光斑坐标 LED世界坐标
    // 残差的计算
    template <typename T>
    bool operator() (
            const T* const Psd,        // Psd世界坐标，3维
            const T* const R_euroc3,
            T* residual ) const     // 残差
    {
        //首先根据相机的外参将世界坐标系转换到相机坐标系并归一化
        //R_euroc[0,1,2] are the angle-axis rotation
        T tmp[3];
        T Led[3];
        T p[3];
        Led[0]=T(_xyz.x);
        Led[1]=T(_xyz.y);
        Led[2]=T(_xyz.z);
        tmp[0] = Led[0]-Psd[0];
        tmp[1] = Led[1]-Psd[1];
        // tmp[1] = Led[1]-Psd[2];
        tmp[2] = Led[2]-T(Psd_z[0]);
        T R_euroct[3];
        // R_euroct[0]=T(R_euroc[0]);R_euroct[1]=T(R_euroc[1]);R_euroct[2]=T(R_euroc3[0]);
        R_euroct[0]=T(R_euroc[0]);R_euroct[1]=T(R_euroc[1]);R_euroct[2]=T(R_euroc3[0]);
        ceres::AngleAxisRotatePoint(R_euroct, tmp, p);
        //计算归一化坐标，xp,yp是归一化坐标，深度为p[2]
        T xp = p[0]/p[2];
        T yp = p[1]/p[2];
        //畸变系数
        T u_= xp*K_real.at<double>(0,0)+K_real.at<double>(0,2);
        T v_= yp*K_real.at<double>(1,1)+K_real.at<double>(1,2);
        //残差
        residual[0] = T(_uv.x)-u_;
        residual[1] = T(_uv.y)-v_;
        return true;
    }
        static ceres::CostFunction* Create(const cv::Point2f uv,const cv::Point3f xyz) {
            //2是输出维度（残差），输入维度，分别为旋转和平移向量
            // return (new ceres::AutoDiffCostFunction<PnPCeres, 2, 3, 1>(
            return (new ceres::AutoDiffCostFunction<PnPCeres, 2, 2, 1>(
                    new PnPCeres(uv,xyz)));
        }
    const cv::Point2f _uv;
    const cv::Point3f _xyz;
};
void* testThread::read_serial(void* arg){
    testThread* pool = static_cast<testThread*> (arg);
    std::cout<<"start read_serial"<<std::endl;
    //创建一个serial对象
    serial::Serial sp;
    //创建timeout
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    sp.setPort("/dev/ttyUSB0");//机器人小车的串口每次会变
    sp.setBaudrate(57600);//波特率设置不对会导致Segmentation fault (core dumped)
    //串口设置timeout
    sp.setTimeout(to);
    try
    {sp.open();}
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return nullptr;
    }
    //判断串口是否打开成功
    if(sp.isOpen())
    {ROS_INFO_STREAM("/dev/ttyUSB1 is opened.");}
    else
    {return nullptr;} 
    ros::Rate loop_rate1(25);
    while(ros::ok())//ctrl+c后退出while，关闭串口
    {  
        pthread_mutex_lock(&pool->m_mutex);
        size_t n = sp.available();
        
        if (n > 0) {
            uint8_t buffer[512];
            n = sp.read(buffer, n); 
            // ROS_INFO_STREAM("store data:");           
            for(size_t i = 0; i < n; i++ ){                  
                pool->que.push(buffer[i]);                
            }
        }
        pthread_cond_signal(&pool->m_cond);//条件锁，数据存储完毕即可唤醒等待的线程     
        pthread_mutex_unlock(&pool->m_mutex);//互斥锁解锁
        loop_rate1.sleep();
    }
    // loop_rate1.sleep();
    ROS_INFO("Exiting read_serial loop, ros::ok() is false"); 
    pool->shutdown = true;
    pool->ExitReadthread();
    sp.close();
 
    return nullptr;

}
void* testThread::dataProcess(void* arg)
{
    testThread* pool = static_cast<testThread*> (arg);
    // ROS_INFO_STREAM("Data process:");
    // 控制位置解算频率,这个线程有x个，消息发布频率为x乘这个速率
    ros::Rate loop_rate2(50);
    while(ros::ok)
    {
        pthread_mutex_lock(&pool->m_mutex);
        while(pool->que.size() < 18 && !pool->shutdown )
        {
            pthread_cond_wait(&pool->m_cond, &pool->m_mutex);
        }
        if(pool->shutdown){//串口关闭，退出程序
           pthread_mutex_unlock(&pool->m_mutex);
           pool->ExitProcessThread();
           return nullptr;
        }
        //连续两个66作为开始的标志    
        tmp = pool->que.front();
        pool->que.pop();
        if (tmp == 66)
        {
            // ROS_INFO_STREAM("start deal data:");
            tmp = pool->que.front();
            pool->que.pop();
            if( tmp == 66 ){//4.5K
                tmp = pool->que.front();
                pool->que.pop();
                uint16_t X_6k = (pool->que.front() << 8)|(tmp);
                double PositionX45k = ((double)X_6k - (double)30000)/(double)6000;
                pool->que.pop();
                tmp = pool->que.front();
                pool->que.pop();
                uint16_t Y_6k = (pool->que.front() << 8)|(tmp);
                double PositionY45k = ((double)Y_6k - (double)30000)/(double)6000;
                pool->que.pop();
                cam2pixel(PositionX45k,PositionY45k,K_piexl);
                ptsPiexl_2d.clear();//将原数据清除
                ptsReal_2d.clear();
                ptsReal_2d.push_back(cv::Point2f(PositionX45k,PositionY45k));
                ptsPiexl_2d.push_back(Position2d);
                // std::cout <<"4.5kx: "<< PositionX45k << std::endl;
                // std::cout <<"4.5ky: "<< PositionY45k << std::endl;
            }
            else
            {
                pthread_cond_signal(&pool->m_cond_pnp);
                pthread_mutex_unlock(&pool->m_mutex);
            }
            tmp = pool->que.front();//6K
            pool->que.pop();
            uint16_t X_8k = (pool->que.front() << 8)|(tmp);
            double PositionX6k = ((double)X_8k - (double)30000)/(double)6000;
            pool->que.pop();
            tmp = pool->que.front();
            pool->que.pop();
            uint16_t Y_8k = (pool->que.front() << 8)|(tmp);
            double PositionY6k = ((double)Y_8k - (double)30000)/(double)6000;
            pool->que.pop();
            cam2pixel(PositionX6k,PositionY6k,K_piexl);
            ptsPiexl_2d.push_back(Position2d);
            ptsReal_2d.push_back(cv::Point2f(PositionX6k,PositionY6k));
            // std::cout << "6kx: "<< PositionX6k << std::endl;
            // std::cout << "6ky: "<< PositionY6k << std::endl;
            tmp = pool->que.front();//8K
            pool->que.pop();
            uint16_t X_10k = (pool->que.front() << 8)|(tmp);
            double PositionX8k = ((double)X_10k - (double)30000)/(double)6000;
            pool->que.pop();
            tmp = pool->que.front();
            pool->que.pop();
            uint16_t Y_10k = (pool->que.front() << 8)|(tmp);
            double PositionY8k = ((double)Y_10k - (double)30000)/(double)6000;
            cam2pixel(PositionX8k,PositionY8k,K_piexl);
            ptsReal_2d.push_back(cv::Point2f(PositionX8k,PositionY8k));
            ptsPiexl_2d.push_back(Position2d);
            pool->que.pop();
            // std::cout << "8kx: "<< PositionX8k << std::endl;
            // std::cout << "8ky: "<< PositionY8k << std::endl;
            tmp = pool->que.front();//10K
            pool->que.pop();
            uint16_t X_14k = (pool->que.front() << 8)|(tmp);
            double PositionX10k = ((double)X_14k - (double)30000)/(double)6000;
            pool->que.pop();
            tmp = pool->que.front();
            pool->que.pop();
            uint16_t Y_14k = (pool->que.front() << 8)|(tmp);
            double PositionY10k = ((double)Y_14k - (double)30000)/(double)6000;
            pool->que.pop();
            cam2pixel(PositionX10k,PositionY10k,K_piexl);
            ptsReal_2d.push_back(cv::Point2f(PositionX10k,PositionY10k));
            ptsPiexl_2d.push_back(Position2d);
            // std::cout << "10kx: "<< PositionX10k << std::endl;
            // std::cout << "10ky: "<< PositionY10k << std::endl;
            pthread_cond_signal(&pool->m_cond_pnp);
            pthread_mutex_unlock(&pool->m_mutex);//这个和下面PNP的解锁要保留一个
            ceres::Problem problemPnp;
            for (int i = 0 ; i < ptsReal_2d.size(); i++)
            {
                ceres::CostFunction* cost_function =PnPCeres::Create(ptsReal_2d[i],pool->ptsLed3d[i]);
                // problemPnp.AddResidualBlock(cost_function,NULL /* squared loss */, Psd, R_euroc3);
                problemPnp.AddResidualBlock(cost_function,NULL /* squared loss */, Psd, R_euroc3);
            }
            // problemPnp.SetParameterLowerBound(Psd, 2, 120);
            // problemPnp.SetParameterUpperBound(Psd, 2, 130); 
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_SCHUR;
            options.minimizer_progress_to_stdout = false;
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problemPnp, &summary);
            // std::cout << Psd[0] << " "<< Psd[1] << " " << Psd_z[0] << " " << R_euroc3[0] << std::endl;
            /*生成随机数，用于Z方向：*/
            std::random_device rd; // 用于生成种子
            std::mt19937 gen(rd()); // 使用Mersenne Twister引擎生成随机数
            double min = 120.0;
            double max = 130.0;
            std::uniform_real_distribution<> dis(min, max);
            double psd_z = dis(gen);

            if (Psd[0] > 100 && Psd[0] < 900 && Psd[1] > 150 && Psd[1] < 350)
            // if (Psd[0] > 100 && Psd[0] < 900 && Psd[1] > 100 && Psd[1] < 900)
            {
                // ROS_INFO("Psd signal publish:");
                geometry_msgs::PointStamped psd_msg;
                psd_msg.header.stamp = ros::Time::now(); // 时间戳
                //原PSD方向
                // psd_msg.point.x = Psd[0];
                // psd_msg.point.y = Psd[1];
                //与相机方向一致的xy
                psd_msg.point.x = Psd[1]/1000.0;                                                                                                                                           
                psd_msg.point.y = (1000-Psd[0])/1000.0;
                psd_msg.point.z = psd_z/1000.0;
                // psd_msg.point.z = psd_z/1000.0;

                if (pool->pub) // 确保发布者已被正确初始化
                {
                    // ROS_INFO_STREAM("Message publish :");
                    pool->pub.publish(psd_msg);
                    /*将位置信息写入文件：*/
                    // std::ofstream foutC("/home/robot/catkin_PSDPub/src/pubpsd_pkg/output/psd.csv", std::ios::app);
                    // foutC.setf(std::ios::fixed, std::ios::floatfield);
                    // foutC.precision(0);
                    // foutC << psd_msg.header.stamp.toSec() * 1e9 << ",";
                    // foutC.precision(5);
                    // foutC << psd_msg.point.x << ","<< psd_msg.point.y << ","<< psd_msg.point.z << std::endl;
                    // foutC.close();
                    } // 发布消息
                else
                {ROS_ERROR("Publisher not initialized in writeque");}
            }
            // else
            // {ROS_INFO("NO psd signal!");}

        }

        else
        {
            while (pool->que.front()!= 66)//不等于66就先把别的东西删除
            {
                pool->que.pop();
            }
            pthread_cond_signal(&pool->m_cond_pnp);
            pthread_mutex_unlock(&pool->m_mutex);
        }
        loop_rate2.sleep();
    }
    return nullptr;
}
void* testThread::solvePosition(void* arg)
{
    testThread* pool = static_cast<testThread*> (arg);
    while(1){
        pthread_mutex_lock(&pool->m_mutex);
        while(ptsPiexl_2d.size()<4 && !pool->shutdown){
            ptsPiexl_2d.clear();
            pthread_cond_wait(&pool->m_cond_pnp,&pool->m_mutex);
        }
        if(pool->shutdown){//串口关闭，退出程序
            pthread_mutex_unlock(&pool->m_mutex);
            pool->ExitPnpThread();
            return nullptr;
        }
        ROS_INFO_STREAM("start pnp:");
        cv::Mat r, t;
        cv::solvePnP(pool->pts_3d, ptsPiexl_2d, K_piexl, cv::Mat(), r, t, 0);
        cv::Mat R_1, R;
        cv::Mat t_1;
        cv::Rodrigues(r, R);
        cv::invert(R, R_1, 0);
        t_1 = -(R_1 * t);

        Eigen::Matrix3f eigenRotation;        
        cv::cv2eigen(R_1,eigenRotation);
        Eigen::Quaternionf quater(eigenRotation);
        // pool->f.open("/home/robot/catkin_PSDPub/src/data.txt",std::ios::out|std::ios::app);
        
        // pool->f<<ros::Time::now()<<" "<<t_1.at<double>(0,0) <<" "<< t_1.at<double>(1,0)<<" "<<  t_1.at<double>(2,0) <<" "<< 
        //                                 quater.x()<<" "<< quater.y()<<" "<< quater.z()<<" "<< quater.w()<<std::endl;
        // pool->f.close();         
        ptsPiexl_2d.clear();
        pthread_mutex_unlock(&pool->m_mutex);
        std::cout << " R = " << std::endl << R << std::endl;
        std::cout << " t = " << std::endl << t << std::endl; 
    }
    return nullptr;
}
void testThread::ExitReadthread()
{
    pthread_t tid = pthread_self();
    if(readID == tid){
        readID = 0;
        std::cout<<"pthread exit read = "<<std::to_string(pthread_self())<<std::endl;
    }
    pthread_exit(NULL);
    return ;
}
void testThread::ExitPnpThread()
{
    pthread_t tid = pthread_self();
    if(pnpID == tid){
        pnpID = 0;
        std::cout<<"pthread exit pnp = "<<std::to_string(pthread_self())<<std::endl;
    }
    pthread_exit(NULL);
    return ;
}
void testThread::ExitProcessThread()
{
    pthread_t tid = pthread_self();
    for(int i = 0; i < NUM; i++){
        if(threadIDs[i] == tid){
            threadIDs[i] = 0;
            std::cout<<"pthread exit = "<<std::to_string(pthread_self())<<std::endl;
        }
    }

    pthread_exit(NULL);
    return ;
}
void testThread::cam2pixel(const float px,const float py, const cv::Mat &K)
{
    float u = px/dx+K_piexl.at<float>(0, 2);
    float v = py/dy+K_piexl.at<float>(1, 2);

    Position2d.x = u;
    Position2d.y = v;
    return ;
}
void testThread::Matrix2Quaternion(const cv::Mat &matrix)
{
		float a[4][4] = {0};
		for(int i=0;i<4;i++)
			for(int j=0;j<4;j++)
			a[i][j]=matrix.at<float>(i,j);

		float trace = a[0][0] + a[1][1] + a[2][2]; 
		if( trace > 0 ) {
			// I changed M_EPSILON to 0
			float s = 0.5f / sqrtf(trace+ 1.0f);
			qw = 0.25f / s;
			qx = ( a[2][1] - a[1][2] ) * s;
			qy = ( a[0][2] - a[2][0] ) * s;
			qz = ( a[1][0] - a[0][1] ) * s;
		} else {
			if ( a[0][0] > a[1][1] && a[0][0] > a[2][2] ) {
			float s = 2.0f * sqrtf( 1.0f + a[0][0] - a[1][1] - a[2][2]);
			qw = (a[2][1] - a[1][2] ) / s;
			qx = 0.25f * s;
			qy = (a[0][1] + a[1][0] ) / s;
			qz = (a[0][2] + a[2][0] ) / s;
			} else if (a[1][1] > a[2][2]) {
			float s = 2.0f * sqrtf( 1.0f + a[1][1] - a[0][0] - a[2][2]);
			qw = (a[0][2] - a[2][0] ) / s;
			qx = (a[0][1] + a[1][0] ) / s;
			qy = 0.25f * s;
			qz = (a[1][2] + a[2][1] ) / s;
			} else {
			float s = 2.0f * sqrtf( 1.0f + a[2][2] - a[0][0] - a[1][1] );
			qw = (a[1][0] - a[0][1] ) / s;
			qx = (a[0][2] + a[2][0] ) / s;
			qy = (a[1][2] + a[2][1] ) / s;
			qz = 0.25f * s;
			}    
		}
        return ;
}
