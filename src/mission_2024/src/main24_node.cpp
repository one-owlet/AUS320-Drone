#include "point.h"
#include "camera.h"
#include <std_msgs/String.h>
#include <quadrotor_msgs/TakeoffLand.h>

Point_Data_t point_data;
Camera_Data_t camera_data;

int main(int argc, char  *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"main24_node");
    ros::NodeHandle nh;
    
    ros::Publisher point_pub = 
        nh.advertise<diansai_msgs::WayPoint>("/px4ctrl/custom_waypoint", 10);

    ros::Publisher land_pub = 
        nh.advertise<quadrotor_msgs::TakeoffLand>("/px4ctrl/takeoff_land", 100);

    ros::Subscriber point_sub =
        nh.subscribe<nav_msgs::Odometry>("/odom_high_freq",
                                         100,
                                         boost::bind(&Point_Data_t::feed, &point_data, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().tcpNoDelay());
    
    ros::Subscriber camera_sub =
        nh.subscribe<sensor_msgs::Image>("/camera/color/image_raw",
                                         1,
                                         boost::bind(&Camera_Data_t::feed, &camera_data, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().tcpNoDelay());
    
    ros::Rate r(10); // 10Hz
    ros::Duration(3.0).sleep(); // 等待3秒钟
    point_data.allow_judge_arrival = true; // 允许point_data.current_point获得里程计数据

    // 记录零偏
    ros::spinOnce();
    point_data.bias_point.x = point_data.current_point.pose.pose.position.x;
    point_data.bias_point.y = point_data.current_point.pose.pose.position.y;
    point_data.bias_point.z = point_data.current_point.pose.pose.position.z;
    point_data.bias_point.yaw = 0.0;
    ROS_INFO("已记录零偏!");
    ROS_INFO("零偏: x=%.2f, y=%.2f, z=%.2f, yaw=%.2f",
             point_data.bias_point.x,
             point_data.bias_point.y,
             point_data.bias_point.z,
             point_data.bias_point.yaw);
 
    // 发布起飞点
    point_data.target_point.x = point_data.target_points[0][0] + point_data.bias_point.x;
    point_data.target_point.y = point_data.target_points[0][1] + point_data.bias_point.y;
    point_data.target_point.z = point_data.target_points[0][2] + point_data.bias_point.z;
    point_data.target_point.yaw = point_data.target_points[0][3] + point_data.bias_point.yaw;
    point_pub.publish(point_data.target_point);
    ROS_INFO("已发布起飞点!");
    
    // 检查是否到达
    while(ros::ok())
    {
        ros::spinOnce();
        if (point_data.check_arrival(point_data.current_point, point_data.target_point)) { point_data.goal_reached_cnt++; }
        if (point_data.goal_reached_cnt >= 30) 
        {
            point_data.goal_reached_cnt = 0;
            point_data.allow_judge_arrival = false; // 禁止point_data.current_point获得里程计数据
            ROS_INFO("已到达起飞点!");
            break;
        }
        r.sleep();
    }

    // 遍历除起飞点之外的巡航点
    for(int i = 1; i <= 6; ++i)
    {
        // 发布目标航点
        point_data.target_point.x = point_data.target_points[i][0] + point_data.bias_point.x;
        point_data.target_point.y = point_data.target_points[i][1] + point_data.bias_point.y;
        point_data.target_point.z = point_data.target_points[i][2] + point_data.bias_point.z;
        point_data.target_point.yaw = point_data.target_points[i][3] + point_data.bias_point.yaw;
        
        point_pub.publish(point_data.target_point); 
        point_data.allow_judge_arrival = true; // 允许point_data.current_point获得里程计数据

        // 检查是否到达
        while(ros::ok())
        {
            ros::spinOnce();
            if (point_data.check_arrival(point_data.current_point, point_data.target_point)) { point_data.goal_reached_cnt++; }
            if (point_data.goal_reached_cnt >= 30) 
            {
                point_data.goal_reached_cnt = 0;
                point_data.allow_judge_arrival = false; // 禁止point_data.current_point获得里程计数据
                camera_data.allow_qrcode_input = true; // 开启二维码识别模式
                ROS_INFO("已到达第%d个目标点!", i);
                break;
            }
            r.sleep();
        }

        // 二维码识别
        while(ros::ok())
        {
            ros::spinOnce();
            if (camera_data.is_qrcode_succeed) 
            {
                camera_data.is_qrcode_mode = false;
                camera_data.allow_qrcode_input = false;
                ROS_INFO("已识别第%d二维码!", i);
                break;
            }
            r.sleep();
        }
        
        // 激光笔指示

    }

    // 发布最终航点
    point_data.target_point.x = point_data.bias_point.x;
    point_data.target_point.y = point_data.bias_point.y;
    point_data.target_point.z = point_data.bias_point.z + 0.3;
    point_data.target_point.yaw = point_data.bias_point.yaw;
    
    point_pub.publish(point_data.target_point); 
    point_data.allow_judge_arrival = true; // 允许point_data.current_point获得里程计数据

    // 检查是否到达
    while(ros::ok())
    {
        ros::spinOnce();
        if (point_data.check_arrival(point_data.current_point, point_data.target_point)) { point_data.goal_reached_cnt++; }
        if (point_data.goal_reached_cnt >= 30) 
        {
            point_data.goal_reached_cnt = 0;
            point_data.allow_judge_arrival = false; // 禁止point_data.current_point获得里程计数据
            ROS_INFO("已到达降落点上方!");
            break;
        }
        r.sleep();
    }

    // 发布降落指令
    quadrotor_msgs::TakeoffLand land_msg;
    land_msg.takeoff_land_cmd = quadrotor_msgs::TakeoffLand::LAND;
    land_pub.publish(land_msg);
    ROS_INFO("已发布降落指令!");

    return 0;
}



