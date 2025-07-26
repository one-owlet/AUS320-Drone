#include "point.h"

void Point_Data_t::feed(nav_msgs::OdometryConstPtr pMsg)
{
    if(!allow_judge_arrival) return; // 如果已经到达目标点，则不再更新当前点
    current_point = *pMsg;
}

bool Point_Data_t::check_arrival(const nav_msgs::Odometry& point_now, const diansai_msgs::WayPoint& point_des)
{
    double distance = sqrt(pow(point_now.pose.pose.position.x - point_des.x, 2) +
                           pow(point_now.pose.pose.position.y - point_des.y, 2) +
                           pow(point_now.pose.pose.position.z - point_des.z, 2));
    return distance < 0.15; // 阈值为0.1米
}