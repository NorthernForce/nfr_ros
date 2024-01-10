#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <cscore/cscore.h>
#include <cscore/cscore_cv.h>
#include <networktables/NetworkTableInstance.h>
namespace nfr
{
    class NFRNoteDetectorNode : public rclcpp::Node
    {
    private:
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSubscriber{nullptr};
        cs::MjpegServer server;
        cs::CvSource outputStream;
        int resolutionWidth, resolutionHeight;
        cv::RNG rng{12345};
        nt::NetworkTableInstance instance;
        std::shared_ptr<nt::NetworkTable> table;
    public:
        NFRNoteDetectorNode() : rclcpp::Node("nfr_note_detector_node")
        {
            std::string cameraPath = declare_parameter("camera_path", "/realsense/image_raw");
            std::string cameraName = declare_parameter("camera_name", "realsense");
            resolutionWidth = declare_parameter("resolution_width", 640);
            resolutionHeight = declare_parameter("resolution_height", 480);
            int fps = declare_parameter("fps", 30);
            server = cs::MjpegServer(cameraName, declare_parameter("camera_port", 1182));
            outputStream = cs::CvSource(cameraName, cs::VideoMode::kMJPEG, resolutionWidth, resolutionHeight, fps);
            server.SetSource(outputStream);
            instance = nt::NetworkTableInstance::GetDefault();
            std::string tableName = declare_parameter("table_name", "xavier");
            table = instance.GetTable(tableName)->GetSubTable(cameraName);
            instance.SetServerTeam(declare_parameter("team_number", 172));
            imageSubscriber = create_subscription<sensor_msgs::msg::Image>(cameraPath, 10, [&](sensor_msgs::msg::Image msg) {
                cv::Mat mat = cv_bridge::toCvCopy(msg, "rgb8")->image;
                cv::Mat smallMat;
                cv::resize(mat, smallMat, {resolutionWidth, resolutionHeight});
                cv::Mat mask;
                cv::Scalar lower(0, 115, 215);
                cv::Scalar upper(255, 255, 255);
                cv::inRange(smallMat, lower, upper, mask);
                std::vector<std::vector<cv::Point>> contours;
                std::vector<cv::Vec4i> hierarchy;
                cv::findContours(mask, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
                std::vector<std::vector<cv::Point>> contours_poly(contours.size());
                std::vector<cv::Rect> boundRect(contours.size());
                for (size_t i = 0; i < contours.size(); i++)
                {
                    approxPolyDP(contours[i], contours_poly[i], 3, true);
                    boundRect[i] = boundingRect(contours_poly[i]);
                }
                if (contours.size() && instance.IsConnected())
                {
                    size_t maxIdx = 0;
                    for (size_t i = 0; i < boundRect.size(); i++)
                    {
                        if (boundRect[i].area() > boundRect[maxIdx].area())
                        {
                            maxIdx = i;
                        }
                    }
                    table->GetEntry("area").SetDouble((double)boundRect[maxIdx].area());
                    table->GetEntry("x").SetDouble((double)boundRect[maxIdx].x);
                    table->GetEntry("y").SetDouble((double)boundRect[maxIdx].y);
                    table->GetEntry("width").SetDouble((double)boundRect[maxIdx].width);
                    table->GetEntry("height").SetDouble((double)boundRect[maxIdx].height);
                    table->GetEntry("present").SetBoolean(true);
                    std::chrono::microseconds offset = (std::chrono::microseconds)instance.GetServerTimeOffset().value();
                    table->GetEntry("stamp").SetInteger(((std::chrono::nanoseconds)((std::chrono::nanoseconds)((rclcpp::Time)msg.header.stamp)
                        .nanoseconds() + offset)).count());
                }
                else if (instance.IsConnected())
                {
                    table->GetEntry("present").SetBoolean(false);
                }
                outputStream.PutFrame(mask);
            });
            instance.StartClient4("xavier-note_detector" + cameraName);
        }
    };
}
int main(int argc, const char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<nfr::NFRNoteDetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}