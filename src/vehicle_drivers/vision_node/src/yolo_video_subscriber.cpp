#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "opencv2/dnn.hpp"

class YoloVideoSubscriber : public rclcpp::Node
{
public:
    YoloVideoSubscriber()
        : Node("yolo_video_subscriber")
    {
        // 创建订阅者，订阅图像话题
        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            std::bind(&YoloVideoSubscriber::imageCallback, this, std::placeholders::_1));

        // 加载 YOLO 模型
        std::string model_path = "yolo11n.pt"; // 替换为实际模型路径
        net_ = cv::dnn::readNet(model_path);
        if (net_.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load YOLO model from %s", model_path.c_str());
        }
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            // 将 ROS 图像消息转换为 OpenCV 图像
            cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

            // 预处理图像以适配 YOLO 输入
            cv::Mat blob = cv::dnn::blobFromImage(frame, 1.0 / 255.0, cv::Size(640, 640), cv::Scalar(), true, false);
            net_.setInput(blob);

            // 推理
            std::vector<cv::Mat> outputs;
            net_.forward(outputs, net_.getUnconnectedOutLayersNames());

            // 解析 YOLO 输出并绘制检测框
            for (const auto &output : outputs)
            {
                auto *data = (float *)output.data;
                for (int i = 0; i < output.rows; ++i, data += output.cols)
                {
                    float confidence = data[4];
                    if (confidence > 0.5) // 置信度阈值
                    {
                        int x = static_cast<int>(data[0] * frame.cols);
                        int y = static_cast<int>(data[1] * frame.rows);
                        int width = static_cast<int>(data[2] * frame.cols);
                        int height = static_cast<int>(data[3] * frame.rows);

                        cv::rectangle(frame, cv::Rect(x, y, width, height), cv::Scalar(0, 255, 0), 2);
                    }
                }
            }

            // 显示图像
            cv::imshow("YOLO Detection", frame);
            cv::waitKey(1);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error processing image: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    cv::dnn::Net net_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<YoloVideoSubscriber>());
    rclcpp::shutdown();
    return 0;
}