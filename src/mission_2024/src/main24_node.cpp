#include "point.h"
#include "camera.h"
#include "std_msgs/String.h"

Point_Data_t point_data;
Camera_Data_t camera_data;

int main(int argc, char  *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"main24_node");
    ros::NodeHandle nh(~);
    
    ros::Publisher point_pub = 
        nh.advertise<diansai_msgs::WayPoint>("/px4ctrl/custom_waypoint",10);

    ros::Subscriber point_sub =
        nh.subscribe<nav_msgs::Odometry>("/odom_high_freq",
                                         100,
                                         boost::bind(&Point_Data_t::feed, &point_data, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().tcpNoDelay());
    
    ros::Subscriber camera_sub =
        nh.subscribe<sensor_msgs::Image>("/camera/image_raw",
                                         1,
                                         boost::bind(&Camera_Data_t::feed, &camera_data, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().tcpNoDelay());
    
    ros::Rate r(10); // 10Hz
    
    // 遍历目标航点
    for(int i = 0; i <= 6; ++i)
    {
        // 发布目标航点
        point_data.target_point = point_data.target_points[i]; 
        point_pub.publish(point_data.target_point); 
        
        // 检查是否到达
        while(ros::ok())
        {
            ros::spinOnce();
            if (check_arrival(current_odom, target_waypoint)) { point_data.goal_reached_cnt++; }
            if (point_data.goal_reached_cnt >= 30) 
            {
                point_data.goal_reached_cnt = 0;
                break;
            }
            r.sleep();
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
            r.sleep();
        }
        // 调用激光笔

    }

    return 0;
}



