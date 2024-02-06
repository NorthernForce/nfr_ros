#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <nfr_msgs/msg/target_list.hpp>
#include <image_transport/camera_subscriber.hpp>
#include <image_transport/image_transport.hpp>
namespace nfr
{
    class NFRNoteDetectorNode : public rclcpp::Node
    {
    private:
        image_transport::CameraSubscriber cameraSubscription;
        rclcpp::Publisher<nfr_msgs::msg::TargetList>::SharedPtr targetPublisher{nullptr};
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imagePublisher{nullptr};
    public:
        NFRNoteDetectorNode() : rclcpp::Node("nfr_note_detector_node")
        {
            cameraSubscription = image_transport::create_camera_subscription(this, "image", std::bind(&NFRNoteDetectorNode::cameraCallback, this,
                std::placeholders::_1, std::placeholders::_2), "raw");
            targetPublisher = create_publisher<nfr_msgs::msg::TargetList>("targets", 10);
            imagePublisher = create_publisher<sensor_msgs::msg::Image>("processed_image", 10);
        }
        void cameraCallback(const sensor_msgs::msg::Image::ConstSharedPtr& image, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cameraInfo)
        {
            cv::Mat mat = cv_bridge::toCvCopy(image, "rgb8")->image;
            cv::Mat mask;
            cv::Scalar lower(0, 135, 225);
            cv::Scalar upper(255, 255, 255);
            cv::inRange(mat, lower, upper, mask);
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
            if (contours.size())
            {
                nfr_msgs::msg::TargetList targets;
                targets.header = image->header;
                for (size_t i = 0; i < boundRect.size(); i++)
                {
                    targets.targets.push_back(nfr_msgs::msg::Target());
                    auto& target = targets.targets[i];
                    target.area = boundRect[i].area();
                    target.center.x = boundRect[i].x + boundRect[i].width / 2;
                    target.center.x = boundRect[i].y + boundRect[i].height / 2;
                    target.fiducial_id = 0;
                    target.yaw = atan((target.center.x - cameraInfo->k[2]) / cameraInfo->k[0]);
                    target.pitch = atan((target.center.y - cameraInfo->k[4]) / (target.center.y / cameraInfo->k[5]));
                }
                targetPublisher->publish(targets);
                cv_bridge::CvImage newImage = cv_bridge::CvImage(image->header, "rgb8", mask);
                imagePublisher->publish(*newImage.toImageMsg());
            }
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