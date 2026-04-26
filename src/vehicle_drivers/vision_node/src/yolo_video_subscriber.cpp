#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "opencv2/dnn.hpp"
#include "image_transport/image_transport.hpp"

class YoloVideoSubscriber : public rclcpp::Node
{
public:
    YoloVideoSubscriber()
        : Node("yolo_video_subscriber")
    {
        // 使用 image_transport 订阅，自动解压
        image_transport_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
        sub_ = image_transport_->subscribe(
            "/camera/image_raw", 10,
            std::bind(&YoloVideoSubscriber::imageCallback, this, std::placeholders::_1));

        // 加载 YOLO 模型（请确保模型文件放在工作目录或指定绝对路径）
        std::string model_path = "yolo11n.pt";
        net_ = cv::dnn::readNet(model_path);
        if (net_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load YOLO model from %s", model_path.c_str());
        }
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
    {
        try {
            // 解码后的图像已经是 bgr8
            cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

            // YOLO 推理
            cv::Mat blob = cv::dnn::blobFromImage(frame, 1.0/255.0, cv::Size(640,640), cv::Scalar(), true, false);
            net_.setInput(blob);

            std::vector<cv::Mat> outputs;
            net_.forward(outputs, net_.getUnconnectedOutLayersNames());

            for (const auto & output : outputs) {
                auto *data = (float *)output.data;
                for (int i = 0; i < output.rows; ++i, data += output.cols) {
                    float confidence = data[4];
                    if (confidence > 0.5) {
                        int x = static_cast<int>(data[0] * frame.cols);
                        int y = static_cast<int>(data[1] * frame.rows);
                        int w = static_cast<int>(data[2] * frame.cols);
                        int h = static_cast<int>(data[3] * frame.rows);
                        cv::rectangle(frame, cv::Rect(x, y, w, h), cv::Scalar(0,255,0), 2);
                    }
                }
            }

            cv::imshow("YOLO Detection", frame);
            cv::waitKey(1);
        } catch (const std::exception & e) {
            RCLCPP_ERROR(this->get_logger(), "Error processing image: %s", e.what());
        }
    }

    std::shared_ptr<image_transport::ImageTransport> image_transport_;
    image_transport::Subscriber sub_;
    cv::dnn::Net net_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<YoloVideoSubscriber>());
    rclcpp::shutdown();
    return 0;
}