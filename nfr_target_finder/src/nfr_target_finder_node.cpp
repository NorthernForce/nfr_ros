#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nfr_msgs/msg/target_list.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <cv_bridge/cv_bridge.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
namespace nfr
{
    class NFRTargetFinderNode : public rclcpp::Node
    {
    private:
        bool useDepthSensor;
        std::shared_ptr<message_filters::Subscriber<apriltag_msgs::msg::AprilTagDetectionArray>> detectionSubscription;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>> cameraInfoSubscription;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depthSubscription;
        std::shared_ptr<message_filters::TimeSynchronizer<apriltag_msgs::msg::AprilTagDetectionArray, sensor_msgs::msg::CameraInfo,
            sensor_msgs::msg::Image>> depthSynchronizer;
        std::shared_ptr<message_filters::TimeSynchronizer<apriltag_msgs::msg::AprilTagDetectionArray, sensor_msgs::msg::CameraInfo>> synchronizer;
        rclcpp::Publisher<nfr_msgs::msg::TargetList>::SharedPtr targetPublisher;
    public:
        NFRTargetFinderNode(rclcpp::NodeOptions options) : rclcpp::Node("nfr_target_finder_node", options)
        {
            useDepthSensor = declare_parameter<bool>("use_depth_subscription", true);
            detectionSubscription = std::make_shared<message_filters::Subscriber<apriltag_msgs::msg::AprilTagDetectionArray>>(this, "tag_detections");
            cameraInfoSubscription = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>>(this, "camera_info");
            if (useDepthSensor)
            {
                depthSubscription = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "depth_raw");
                depthSynchronizer = std::make_shared<message_filters::TimeSynchronizer<apriltag_msgs::msg::AprilTagDetectionArray,
                    sensor_msgs::msg::CameraInfo, sensor_msgs::msg::Image>>
                    (*detectionSubscription, *cameraInfoSubscription, *depthSubscription, 10);
                depthSynchronizer->registerCallback(std::bind(&NFRTargetFinderNode::onDepthCamera, this, std::placeholders::_1, std::placeholders::_2,
                    std::placeholders::_3));
            }
            else
            {
                synchronizer = std::make_shared<message_filters::TimeSynchronizer<apriltag_msgs::msg::AprilTagDetectionArray,
                    sensor_msgs::msg::CameraInfo>>
                    (*detectionSubscription, *cameraInfoSubscription, 10);
                synchronizer->registerCallback(std::bind(&NFRTargetFinderNode::onCamera, this, std::placeholders::_1, std::placeholders::_2));
            }
            targetPublisher = create_publisher<nfr_msgs::msg::TargetList>("targets", 10);
        }
        void onDepthCamera(apriltag_msgs::msg::AprilTagDetectionArray::ConstSharedPtr detections, sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfo,
            sensor_msgs::msg::Image::ConstSharedPtr depthImage)
        {
            nfr_msgs::msg::TargetList targets;
            targets.header = detections->header;
            for (auto detection : detections->detections)
            {
                nfr_msgs::msg::Target target;
                target.area = 0;
                target.center.x = detection.centre.x;
                target.center.y = detection.centre.y;
                auto mat = cv_bridge::toCvCopy(*depthImage, sensor_msgs::image_encodings::TYPE_16UC1);
                target.depth = (double)mat->image.at<uint16_t>(target.center.y, target.center.x) / 1000;
                target.fiducial_id = detection.id;
                target.yaw = atan((target.center.x - cameraInfo->k[2]) / cameraInfo->k[0]);
                target.pitch = atan((target.center.y - cameraInfo->k[5]) / cameraInfo->k[4]);
                targets.targets.push_back(target);
            }
            targetPublisher->publish(targets);
        }
        void onCamera(apriltag_msgs::msg::AprilTagDetectionArray::ConstSharedPtr detections, sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfo)
        {
            nfr_msgs::msg::TargetList targets;
            targets.header = detections->header;
            for (auto detection : detections->detections)
            {
                nfr_msgs::msg::Target target;
                target.area = 0;
                target.center.x = detection.centre.x;
                target.center.y = detection.centre.y;
                target.depth = 0;
                target.fiducial_id = detection.id;
                target.yaw = atan((target.center.x - cameraInfo->k[2]) / cameraInfo->k[0]);
                target.pitch = atan((target.center.y - cameraInfo->k[5]) / cameraInfo->k[4]);
                targets.targets.push_back(target);
            }
            targetPublisher->publish(targets);
        }
    };
}
RCLCPP_COMPONENTS_REGISTER_NODE(nfr::NFRTargetFinderNode);