#include "camera.h"

void Camera_Data_t::set_flag()
{
    is_qrcode_mode = true;
    is_qrcode_succeed = false;
}

void Camera_Data_t::save_qrcode(const std::string& result) 
{
    if (qrcode_results.find(result) == qrcode_results.end()) {  // 如果结果没保存过
        std::string path = ros::package::getPath("mission_2024") + "/qrcode/result.txt";

        // 以追加模式打开文件，不存在自动创建
        std::ofstream fout(path, std::ios::app);
        if (!fout.is_open()) {
            ROS_ERROR("Failed to open file: %s", path.c_str());
            return;
        }

        fout << result << std::endl;  // 写入二维码结果并换行
        fout.close();

        ROS_INFO("Saved new QR code result: %s", result.c_str());

        qrcode_results.insert(result);  // 记录已保存内容
    }
    // 否则不打印也不保存，避免刷屏和重复写入
}

void Camera_Data_t::feed(sensor_msgs::Image::ConstPtr pmsg)
{
    if (!is_qrcode_mode) return;

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(pmsg, "bgr8");
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat gray;
    cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);

    zbar::ImageScanner scanner;
    scanner.set_config(ZBAR_QRCODE, ZBAR_CFG_ENABLE, 1);
    zbar::Image zbar_img(gray.cols, gray.rows, "Y800", gray.data, gray.cols * gray.rows);
    scanner.scan(zbar_img);

    if (zbar_img.symbol_begin() == zbar_img.symbol_end()) return;

    // 图像中心
    cv::Point2f image_center(cv_ptr->image.cols / 2.0f, cv_ptr->image.rows / 2.0f);

    const zbar::Symbol* closest_symbol = nullptr;
    double min_distance = std::numeric_limits<double>::max();

    // 遍历所有二维码，找离中心最近的
    for (auto symbol = zbar_img.symbol_begin(); symbol != zbar_img.symbol_end(); ++symbol) {
        float avg_x = 0, avg_y = 0;
        int n = symbol->get_location_size();
        for (int i = 0; i < n; ++i) {
            avg_x += symbol->get_location_x(i);
            avg_y += symbol->get_location_y(i);
        }
        avg_x /= n;
        avg_y /= n;
        double dist = std::hypot(avg_x - image_center.x, avg_y - image_center.y);

        if (dist < min_distance) {
            min_distance = dist;
            closest_symbol = &(*symbol);
        }
    }

    if (closest_symbol) {
        std::string result = closest_symbol->get_data();

        // 绘制边框
        std::vector<cv::Point> points;
        for (int i = 0; i < closest_symbol->get_location_size(); i++) {
            points.emplace_back(
                closest_symbol->get_location_x(i),
                closest_symbol->get_location_y(i)
            );
        }
        cv::polylines(cv_ptr->image, points, true, cv::Scalar(0, 255, 0), 2);
        cv::putText(cv_ptr->image, result, points[0], cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);

        Camera_Data_t::save_qrcode(result);
        is_qrcode_succeed = true;
        ROS_INFO("QR code detected: %s", result.c_str());
    }
}

void Camera_Data_t::clear_flag()
{
    is_qrcode_mode = false;
    is_qrcode_succeed = false;
}