#ifndef __WAYPOINT_H__
#define __WAYPOINT_H__

#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include "diansai_msgs/WayPoint.h"

class Point_Data_t
{
public:
    int goal_reached_cnt = 0;
    bool allow_judge_arrival = false;
    nav_msgs::Odometry current_point;
    diansai_msgs::WayPoint bias_point;
    diansai_msgs::WayPoint target_point;
    float target_points[1+6][4] = {
    {0.0,  0.0,   0.5,  0.0  }, // x, y, z, yaw, 起飞悬停高度点
    {1.15, 0.52,  1.27, 0.0  },
    {1.15, 0.0,   1.27, 0.0  },
    {1.15, -0.52, 1.27, 0.0  },
    {1.15, -0.52, 0.9,  0.0  },
    {1.15, 0.0,   0.9,  0.0  },
    {1.15, 0.52,  0.9,  0.0  }
};
    
    void feed(nav_msgs::OdometryConstPtr pMsg);
    bool check_arrival(const nav_msgs::Odometry& point_now, const diansai_msgs::WayPoint& point_des);
};

#endif // __WAYPOINT_H__