#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cscore/cscore.h>
#include <cscore/cscore_cv.h>
#include <cscore/cscore_cpp.h>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>
namespace nfr
{
    class NFRCameraNode : public rclcpp::Node
    {
    private:
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSubscriber{nullptr};
        cs::MjpegServer server;
        cs::CvSource outputStream;
        int resolutionWidth, resolutionHeight;
    public:
        NFRCameraNode() : rclcpp::Node("nfr_camera_node")
        {
            std::string cameraName = declare_parameter("camera_name", "default camera");
            server = cs::MjpegServer(cameraName, declare_parameter("camera_port", 1181));
            resolutionWidth = declare_parameter("resolution_width", 640);
            resolutionHeight = declare_parameter("resolution_height", 480);
            int fps = declare_parameter("fps", 30);
            RCLCPP_INFO(get_logger(), "Opening outputStream for %d, %d, %d", resolutionWidth, resolutionHeight, fps);
            outputStream = cs::CvSource(cameraName, cs::VideoMode::kMJPEG, resolutionWidth, resolutionHeight, fps);
            RCLCPP_INFO(get_logger(), "Opened outputStream");
            RCLCPP_INFO(get_logger(), "Hello");
            server.SetSource(outputStream);
            imageSubscriber = create_subscription<sensor_msgs::msg::Image>("image", rclcpp::SensorDataQoS(), std::bind(&NFRCameraNode::cameraCallback, this, std::placeholders::_1));
        }
        void cameraCallback(const sensor_msgs::msg::Image msg)
        {
            cv::Mat mat = cv_bridge::toCvCopy(msg, "rgb8")->image;
            cv::Mat newMat;
            cv::resize(mat, newMat, {resolutionWidth, resolutionHeight});
            outputStream.PutFrame(newMat);
        }
    };
}
int main(int argc, const char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<nfr::NFRCameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}