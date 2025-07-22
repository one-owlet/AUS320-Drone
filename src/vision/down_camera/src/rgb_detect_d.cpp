#include <ros/ros.h>  // ROS 的核心功能头文件
#include <geometry_msgs/Point.h>  // 用于发布偏移量消息
#include <opencv2/opencv.hpp>  // OpenCV 主头文件，图像处理所需

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rgb_detect_d");  // 初始化 ROS 节点，节点名为 obj_detection
    ros::NodeHandle nh("~"); // 私有

    // 创建一个 ROS 发布者，用于发布黄色图形相对于图像中心的偏移量
    ros::Publisher offset_pub = nh.advertise<geometry_msgs::Point>("offset", 10);

    // 打开摄像头设备(设备号为6)
    cv::VideoCapture cap("/dev/video0");
    if (!cap.isOpened()) {
        ROS_ERROR("Cannot open camera device!");  // 如果打开失败，报错并退出
        return -1;
    }

    // 声明图像帧容器
    cv::Mat frame;

    // 设置 ROS 循环频率为 30Hz（对应视频30帧每秒）
    ros::Rate r(30);

    // 主循环开始
    while (ros::ok()) {
        cap >> frame;  // 从摄像头获取一帧图像
        if (frame.empty()) continue;  // 如果帧为空则跳过

        // 将 BGR 图像转换为 HSV 色彩空间
        cv::Mat hsv;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        // 定义 HSV 中黄色的上下阈值
        cv::Scalar lower(20, 100, 100);
        cv::Scalar upper(30, 255, 255);

        // 根据阈值生成二值掩码，提取出黄色区域
        cv::Mat mask;
        cv::inRange(hsv, lower, upper, mask);

        // 腐蚀和膨胀操作，去除小噪声
        cv::erode(mask, mask, cv::Mat(), cv::Point(-1,-1), 2);
        cv::dilate(mask, mask, cv::Mat(), cv::Point(-1,-1), 2);

        // 查找所有外轮廓
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // 初始化最大面积和对应的轮廓
        double max_area = 0;
        std::vector<cv::Point> best;

        // 遍历所有轮廓，选出面积最大的一个（即最可能是黄色目标）
        for (auto& c : contours) {
            double area = cv::contourArea(c);
            if (area > max_area) {
                max_area = area;
                best = c;
            }
        }

        // 创建偏移消息，默认设为0
        geometry_msgs::Point offset;
        offset.x = 0;
        offset.y = 0;

        // 如果找到有效目标（面积足够大），进行绘制与偏移计算
        if (max_area > 100) {
            // 获取最小旋转矩形（可以适应椭圆、斜形等）
            cv::RotatedRect rect = cv::minAreaRect(best);
            cv::Point2f pts[4];
            rect.points(pts);  // 获取矩形四个角点

            // 绘制绿色矩形框
            for (int i = 0; i < 4; ++i)
                cv::line(frame, pts[i], pts[(i+1)%4], cv::Scalar(0,255,0), 2);

            // 图像中心坐标（原点）
            int cx = frame.cols / 2;
            int cy = frame.rows / 2;

            // 计算目标中心相对于图像中心的像素偏移
            float dx = rect.center.x - cx;  // 右为正，左为负
            float dy = rect.center.y - cy;  // 下为正，上为负

            // 填充 ROS 消息
            offset.x = dx;
            offset.y = dy;

            // 将偏移值以字符串形式写在图像上
            std::ostringstream oss;
            oss << "Offset(px): (" << dx << ", " << dy << ")";
            cv::putText(frame, oss.str(), cv::Point(30, 30),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,0,255), 2);

            // 在图像中心画一个蓝色圆点，表示坐标原点
            cv::circle(frame, cv::Point(cx, cy), 5, cv::Scalar(255, 0, 0), -1);
        }

        // 发布偏移消息
        offset_pub.publish(offset);

        // 显示带有识别结果的图像
        cv::imshow("camera tracking", frame);
        cv::waitKey(1);  // 更新窗口，等待1ms

        // 控制帧率
        r.sleep();
    }

    // 释放摄像头资源并关闭窗口
    cap.release();
    cv::destroyAllWindows();
    return 0;
}
