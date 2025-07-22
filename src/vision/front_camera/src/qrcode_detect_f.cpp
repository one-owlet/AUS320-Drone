#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <zbar.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <ros/package.h>
#include <set>  // 用于存储已经保存的二维码内容

using namespace zbar;
using namespace std;

// 全局变量，保存已经写入文件的二维码内容，防止重复写入
std::set<std::string> saved_results;

// 保存二维码结果函数（只保存之前没保存过的内容）
void saveQrCodeResultOnce(const std::string& result) {
    if (saved_results.find(result) == saved_results.end()) {  // 如果结果没保存过
        std::string path = ros::package::getPath("front_camera") + "/qrcode/result.txt";

        // 以追加模式打开文件，不存在自动创建
        std::ofstream fout(path, std::ios::app);
        if (!fout.is_open()) {
            ROS_ERROR("Failed to open file: %s", path.c_str());
            return;
        }

        fout << result << std::endl;  // 写入二维码结果并换行
        fout.close();

        ROS_INFO("Saved new QR code result: %s", result.c_str());

        saved_results.insert(result);  // 记录已保存内容
    }
    // 否则不打印也不保存，避免刷屏和重复写入
}

// 图像回调函数，接收图像并识别二维码
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        // 将 ROS 图像消息转换为 OpenCV 图像
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // 转换为灰度图，因为 ZBar 需要灰度图
    cv::Mat gray;
    cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);

    // 初始化 ZBar 扫描器
    ImageScanner scanner;
    scanner.set_config(ZBAR_QRCODE, ZBAR_CFG_ENABLE, 1);

    // 封装灰度图像到 ZBar 格式
    zbar::Image zbar_img(gray.cols, gray.rows, "Y800", gray.data, gray.cols * gray.rows);

    // 扫描二维码
    scanner.scan(zbar_img);

    // 遍历检测到的二维码符号
    for (auto symbol = zbar_img.symbol_begin(); symbol != zbar_img.symbol_end(); ++symbol) {
        std::string result = symbol->get_data();

        // 画二维码边框
        std::vector<cv::Point> points;
        for (int i = 0; i < symbol->get_location_size(); i++) {
            points.push_back(cv::Point(symbol->get_location_x(i), symbol->get_location_y(i)));
        }
        cv::polylines(cv_ptr->image, points, true, cv::Scalar(0, 255, 0), 2);
        cv::putText(cv_ptr->image, result, points[0], cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);

        // 保存二维码结果，确保同一内容只写一次，并只打印一次
        saveQrCodeResultOnce(result);
    }

    // 显示结果图像窗口
    cv::imshow("ZBar QR Detect", cv_ptr->image);
    cv::waitKey(1);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "qrcode_detect_f");  // 初始化 ROS 节点
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    // 订阅前视摄像头的图像话题
    image_transport::Subscriber sub = it.subscribe("/camera/color/image_raw", 1, imageCallback);

    ros::spin();  // 循环等待回调
    return 0;
}
