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
#include <nfr_msgs/msg/target_list.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
using AprilTagDetectionArray = apriltag_msgs::msg::AprilTagDetectionArray;
using std::placeholders::_1;
namespace nfr
{
    class AprilTagLocalizationNode : public rclcpp::Node
    {
    private:
        rclcpp::Subscription<AprilTagDetectionArray>::SharedPtr subscription;
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher;
        rclcpp::Publisher<nfr_msgs::msg::TargetList>::SharedPtr targetPublisher;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr infoSubscription;
        sensor_msgs::msg::CameraInfo info;
        std::unique_ptr<tf2_ros::Buffer> buffer;
        std::shared_ptr<tf2_ros::TransformListener> listener;
        std::map<int, tf2::Transform> tagPoses;
        std::string baseFrame;
    public:
        AprilTagLocalizationNode(const rclcpp::NodeOptions& options) : rclcpp::Node("nfr_apriltag_localization_node", options)
        {
            subscription = create_subscription<AprilTagDetectionArray>("tag_detections", rclcpp::SensorDataQoS(),
                std::bind(&AprilTagLocalizationNode::detectionCallback, this, _1));
            publisher = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose_estimations", 10);
            targetPublisher = create_publisher<nfr_msgs::msg::TargetList>("targets", 10);
            infoSubscription = create_subscription<sensor_msgs::msg::CameraInfo>("camera_info", 10, [&](const sensor_msgs::msg::CameraInfo& msg)
            {
                info = msg;
                infoSubscription.reset();
            });
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
            nfr_msgs::msg::TargetList targets;
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
                            (detection.family + ":" + std::to_string(detection.id)).c_str());
                        return;
                    }
                    auto baseToCamera = buffer->lookupTransform(baseFrame, detections.header.frame_id, tf2::TimePointZero);
                    auto cameraToTag = buffer->lookupTransform(detections.header.frame_id, detection.family + ":" + std::to_string(detection.id),
                        tf2::TimePointZero);
                    tf2::Transform baseToCameraTransform;
                    tf2::fromMsg(baseToCamera.transform, baseToCameraTransform);
                    tf2::Transform cameraToTagTransform;
                    tf2::fromMsg(cameraToTag.transform, cameraToTagTransform);
                    auto worldToTag = tagPoses[detection.id];
                    RCLCPP_INFO(get_logger(), "Tag estimated to be %f meters forward, %f meters tall, %f meters to the left",
                        cameraToTag.transform.translation.x, cameraToTag.transform.translation.y, cameraToTag.transform.translation.z);
                    geometry_msgs::msg::PoseWithCovarianceStamped pose;
                    pose.header.frame_id = "map";
                    pose.header.stamp = detections.header.stamp;
                    pose.pose.pose.position.x = worldToTag.getOrigin().getX();
                    pose.pose.pose.position.y = worldToTag.getOrigin().getY();
                    pose.pose.pose.position.z = worldToTag.getOrigin().getZ();
                    pose.pose.pose.position.x -= baseToCamera.transform.translation.x + cameraToTag.transform.translation.z;
                    pose.pose.pose.position.y -= baseToCamera.transform.translation.y + cameraToTag.transform.translation.x;
                    pose.pose.pose.position.z -= baseToCamera.transform.translation.z + cameraToTag.transform.translation.y;
                    tf2::Quaternion baseToCameraQuaternion;
                    tf2::fromMsg(baseToCamera.transform.rotation, baseToCameraQuaternion);
                    tf2::Quaternion cameraToTagQuaternion;
                    tf2::fromMsg(cameraToTag.transform.rotation, cameraToTagQuaternion);
                    tf2::Quaternion worldToRobot = worldToTag * cameraToTagQuaternion.inverse() * cameraToTagQuaternion.inverse();
                    pose.pose.pose.orientation = tf2::toMsg(worldToRobot);
                    publisher->publish(pose);
                    nfr_msgs::msg::Target target;
                    target.header = detections.header;
                    target.area = 0;
                    target.center.x = detection.centre.x;
                    target.center.x = detection.centre.y;
                    target.fiducial_id = detection.id;
                    target.yaw = atan((target.center.x - info.k[2]) / info.k[0]);
                    target.pitch = atan((target.center.y - info.k[4]) / (target.center.y / info.k[5]));
                    targets.targets.push_back(target);
                }
            }
            targetPublisher->publish(targets);
        }
    };
}
RCLCPP_COMPONENTS_REGISTER_NODE(nfr::AprilTagLocalizationNode);