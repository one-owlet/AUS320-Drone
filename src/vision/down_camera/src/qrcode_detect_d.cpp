#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <zbar.h>
#include <fstream>
#include <set>
#include <ros/package.h>

using namespace zbar;
using namespace std;

// 记录已保存的二维码数据，防止重复写入和重复打印
std::set<std::string> saved_results;

// 保存识别结果（只写一次）并打印
void saveQrCodeResultOnce(const std::string& result) {
    if (saved_results.count(result) > 0) {
        return;
    }

    std::string path = ros::package::getPath("down_camera") + "/qrcode/result.txt";
    std::ofstream fout(path, std::ios::app);  // 追加写入
    if (!fout.is_open()) {
        ROS_ERROR("Failed to open file: %s", path.c_str());
        return;
    }

    fout << result << std::endl;
    fout.close();

    saved_results.insert(result);
    ROS_INFO("New QR code saved and printed: %s", result.c_str());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "qrcode_detect_d");
    ros::NodeHandle nh("~");

    // 打开摄像头
    cv::VideoCapture cap("/dev/video0");
    if (!cap.isOpened()) {
        ROS_ERROR("Cannot open camera!");
        return -1;
    }

    ros::Rate loop_rate(30);  // 30Hz

    while (ros::ok()) {
        cv::Mat frame, gray;
        cap >> frame;
        if (frame.empty()) continue;

        // 转灰度图
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // 初始化ZBar
        ImageScanner scanner;
        scanner.set_config(ZBAR_QRCODE, ZBAR_CFG_ENABLE, 1);
        zbar::Image zbar_img(gray.cols, gray.rows, "Y800", gray.data, gray.cols * gray.rows);

        int n = scanner.scan(zbar_img);

        for (auto symbol = zbar_img.symbol_begin(); symbol != zbar_img.symbol_end(); ++symbol) {
            std::string result = symbol->get_data();

            // 可视化边框
            std::vector<cv::Point> points;
            for (int i = 0; i < symbol->get_location_size(); i++) {
                points.push_back(cv::Point(symbol->get_location_x(i), symbol->get_location_y(i)));
            }
            cv::polylines(frame, points, true, cv::Scalar(0, 255, 0), 2);
            cv::putText(frame, result, points[0], cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);

            // 保存并只打印一次
            saveQrCodeResultOnce(result);
        }

        // 显示图像
        cv::imshow("RGB Camera QR Detect", frame);
        cv::waitKey(1);

        ros::spinOnce();
        loop_rate.sleep();
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
