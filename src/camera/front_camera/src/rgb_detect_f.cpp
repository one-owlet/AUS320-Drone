#include <ros/ros.h>  // ROS核心功能头文件
#include <geometry_msgs/Point.h>  // 用于发布偏移消息
#include <sensor_msgs/Image.h>  // ROS图像消息类型
#include <cv_bridge/cv_bridge.h>  // 将ROS图像消息转为OpenCV图像
#include <image_transport/image_transport.h>  // 图像传输工具
#include <opencv2/opencv.hpp>  // OpenCV核心库

// 全局变量：用于发布偏移量
ros::Publisher offset_pub;

// 图像回调函数：处理来自 RealSense RGB 图像并检测黄色封闭物体
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;

    try {
        // 将ROS图像转换为OpenCV格式（BGR8）
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // 获取OpenCV图像
    cv::Mat frame = cv_ptr->image;

    // 转换到HSV颜色空间便于颜色识别
    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    // 设置黄色HSV范围（可根据实际调整）
    cv::Scalar lower(20, 100, 100);
    cv::Scalar upper(30, 255, 255);
    cv::Mat mask;
    cv::inRange(hsv, lower, upper, mask);

    // 腐蚀+膨胀去噪
    cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

    // 查找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // 寻找最大轮廓
    double max_area = 0;
    std::vector<cv::Point> best;
    for (auto& c : contours) {
        double area = cv::contourArea(c);
        if (area > max_area) {
            max_area = area;
            best = c;
        }
    }

    // 创建偏移消息
    geometry_msgs::Point offset;
    offset.x = 0;
    offset.y = 0;

    if (max_area > 100) {
        cv::RotatedRect rect = cv::minAreaRect(best);
        cv::Point2f center = rect.center;

        // 图像中心
        int cx = frame.cols / 2;
        int cy = frame.rows / 2;

        offset.x = center.x - cx;
        offset.y = center.y - cy;

        // 画识别矩形
        cv::Point2f pts[4];
        rect.points(pts);
        for (int i = 0; i < 4; ++i)
            cv::line(frame, pts[i], pts[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);

        // 图像中心标记
        cv::circle(frame, cv::Point(cx, cy), 5, cv::Scalar(255, 0, 0), -1);

        // 显示偏移量文字
        std::ostringstream oss;
        oss << "Offset: (" << offset.x << ", " << offset.y << ")";
        cv::putText(frame, oss.str(), cv::Point(30, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
    }

    // 发布偏移
    offset_pub.publish(offset);

    // 显示图像窗口
    cv::imshow("RGB Object Detection", frame);
    cv::waitKey(1);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "rgb_detect_f");  // 节点名改为 rgbobj_detect
    ros::NodeHandle nh("~");

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(
        "/camera/color/image_raw", 1, imageCallback);

    offset_pub = nh.advertise<geometry_msgs::Point>("offset", 10);

    ROS_INFO("rgb_detect_f node started.");  // 日志输出节点名

    ros::spin();
    return 0;
}
