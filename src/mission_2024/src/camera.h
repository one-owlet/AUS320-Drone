#ifndef __CAMERA_H__
#define __CAMERA_H__

#include <set>
#include <zbar.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

class Camera_Data_t
{
public:
    is_qrcode_mode{false};
    is_qrcode_succeed{false};
    std::set<std::string> qrcode_results;

    Camera_Data_t();
    ~Camera_Data_t();

    void set_flag();
    void clear_flag();
    void save_qrcode(const std::string& result);
    void feed(sensor_msgs::Image::ConstPtr pmsg);
}

#endif