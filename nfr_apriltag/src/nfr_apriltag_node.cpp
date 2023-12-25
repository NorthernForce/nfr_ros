#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nlohmann/json.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <filesystem>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
using AprilTagDetectionArray = apriltag_msgs::msg::AprilTagDetectionArray;
using std::placeholders::_1;
namespace nfr
{
    class AprilTagLocalizationNode : public rclcpp::Node
    {
    private:
        rclcpp::Subscription<AprilTagDetectionArray>::SharedPtr subscription;
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher;
        std::unique_ptr<tf2_ros::Buffer> buffer;
        std::shared_ptr<tf2_ros::TransformListener> listener;
        std::map<int, tf2::Transform> tagPoses;
        std::string baseFrame;
    public:
        AprilTagLocalizationNode(const rclcpp::NodeOptions& options) : rclcpp::Node("nfr_apriltag_localization_node", options)
        {
            subscription = create_subscription<AprilTagDetectionArray>("tag_detections", rclcpp::SensorDataQoS(),
                std::bind(&AprilTagLocalizationNode::detectionCallback, this, _1));
            publisher = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose_estimations", rclcpp::SensorDataQoS());
            buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
            listener = std::make_shared<tf2_ros::TransformListener>(*buffer);
            std::filesystem::path defaultPath = (std::filesystem::path)ament_index_cpp::get_package_share_directory("nfr_charged_up") / "config"
                / "field.json";
            std::string fieldPath = declare_parameter<std::string>("field_path", defaultPath);
            std::ifstream fieldJson(fieldPath);
            nlohmann::json json = nlohmann::json::parse(fieldJson);
            for (auto tag : json["tags"])
            {
                tagPoses[tag["ID"]].setOrigin({tag["pose"]["translation"]["x"], tag["pose"]["translation"]["y"], tag["pose"]["translation"]["z"]});
                tagPoses[tag["ID"]].setRotation({tag["pose"]["rotation"]["quaternion"]["X"], tag["pose"]["rotation"]["quaternion"]["Y"],
                    tag["pose"]["rotation"]["quaternion"]["Z"], tag["pose"]["rotation"]["quaternion"]["W"]});
            }
            baseFrame = declare_parameter<std::string>("base_frame", "base_link");
            RCLCPP_INFO(get_logger(), "Loaded all tags from %s", get_parameter("field_path").as_string().c_str());
        }
        void detectionCallback(const AprilTagDetectionArray& detections)
        {
            for (auto detection : detections.detections)
            {
                RCLCPP_INFO(get_logger(), "Detected apriltag #%d.", detection.id);
                if (tagPoses.find(detection.id) == tagPoses.end())
                {
                    RCLCPP_WARN(get_logger(), "#%d is not on the field.", detection.id);
                }
                else
                {
                    if (!buffer->canTransform(baseFrame, detections.header.frame_id, tf2::TimePointZero))
                    {
                        RCLCPP_ERROR(get_logger(), "Could not transform %s to %s.", baseFrame.c_str(), detections.header.frame_id.c_str());
                        return;
                    }
                    if (!buffer->canTransform(detections.header.frame_id, detection.family + ":" + std::to_string(detection.id), tf2::TimePointZero))
                    {
                        RCLCPP_ERROR(get_logger(), "Could not transform %s to %s.", detections.header.frame_id.c_str(),
                            detection.family + ":" + std::to_string(detection.id));
                        return;
                    }
                    auto baseToCamera = buffer->lookupTransform(baseFrame, detections.header.frame_id, tf2::TimePointZero);
                    auto cameraToTag = buffer->lookupTransform(detections.header.frame_id, detection.family + ":" + std::to_string(detection.id),
                        tf2::TimePointZero);
                    tf2::Transform baseToCameraTransform;
                    tf2::fromMsg(baseToCamera, baseToCameraTransform);
                    tf2::Transform cameraToTagTransform;
                    tf2::fromMsg(cameraToTag, cameraToTagTransform);
                    auto worldToTag = tagPoses[detection.id];
                    geometry_msgs::msg::PoseWithCovarianceStamped pose;
                    pose.header.frame_id = "map";
                    pose.header.stamp = detections.header.stamp;
                    tf2::Transform worldToRobot = worldToTag * (baseToCameraTransform * cameraToTagTransform).inverse();
                    pose.pose.pose.orientation = tf2::toMsg(worldToRobot.getRotation());
                    pose.pose.pose.position.x = worldToRobot.getOrigin().getX();
                    pose.pose.pose.position.y = worldToRobot.getOrigin().getY();
                    pose.pose.pose.position.z = worldToRobot.getOrigin().getZ();
                    publisher->publish(pose);
                }
            }
        }
    };
}
RCLCPP_COMPONENTS_REGISTER_NODE(nfr::AprilTagLocalizationNode);