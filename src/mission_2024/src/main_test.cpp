#include "camera.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "diansai_msgs/WayPoint.h"

// 目标位置
diansai_msgs::WayPoint target_waypoint;
// 摄像头数据对象
Camera_Data_t camera_data;

// 保存当前无人机位置
nav_msgs::Odometry current_odom;

// 回调函数，只接受信息
void odom_callback(nav_msgs::OdometryConstPtr pMsg){
    // 保存当前位置
    current_odom = *pMsg;
}

// 检查无人机是否到达目标航点
bool check_arrival(const nav_msgs::Odometry& odom, const diansai_msgs::WayPoint& waypoint) {
    // 计算距离
    double distance = sqrt(pow(odom.pose.pose.position.x - waypoint.x, 2) +
                            pow(odom.pose.pose.position.y - waypoint.y, 2) +
                            pow(odom.pose.pose.position.z - waypoint.z, 2));
    return distance < 0.1; // 阈值为0.1米
}

int main(int argc, char  *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"listener");
    ros::NodeHandle nh;
    
    ros::Publisher waypoint_pub = 
        nh.advertise<diansai_msgs::WayPoint>("/px4ctrl/custom_waypoint",10);
    ros::Subscriber odom_sub =
        nh.subscribe<nav_msgs::Odometry>("odom",
                                         100,
                                         odom_callback,
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().tcpNoDelay());

    diansai_msgs::WayPoint waypoints[1+6] = {
        {0.0, 0.0, 0.5, 0.0}, // x, y, z, yaw, 起飞悬停高度点
        {2.0, 4.0, 0.0, 0.0},
        {3.0, 6.0, 0.0, 0.0},
        {4.0, 8.0, 0.0, 0.0},
        {5.0, 8.0, 0.0, 0.0},
        {6.0, 8.0, 0.0, 0.0},
        {6.0, 8.0, 0.0, 0.0}
    };
    
    ros::Duration(0.1) du; // 等待系统稳定
    ros::Rate rate(10); // 10Hz

    // 检查是否到达起飞航点
    int cnt = 0;
    while(cnt <= 30)
    {
        ros::spinOnce();
        if (abs(current_odom.pose.pose.position.z - waypoints[0].z) < 0.1) 
        {
            cnt++;
        }
        du.sleep();
    }
    ROS_INFO("Reached takeoff waypoint, starting mission...");

    for(int i = 1; i <= 6; ++i)
    {
        // 设置目标航点
        target_waypoint = waypoints[i]; 

        // 发布目标航点
        waypoint_pub.publish(target_waypoint); 
        
        // 检查是否到达目标航点
        while(ros::ok())
        {
            ros::spinOnce();
            if (check_arrival(current_odom, target_waypoint)) break;
            rate.sleep();
        }

        // 调用摄像头，识别二维码
        camera_data.set_flag();
        while(ros::ok())
        {
            ros::spinOnce();
            if (camera_data.is_qrcode_succeed) 
            {
                camera_data.clear_flag();
                break;
            }
            rate.sleep();
        }
        // 调用激光笔

    }

    return 0;
}



