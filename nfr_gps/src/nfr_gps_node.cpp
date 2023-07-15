#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <map>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
using namespace std;
using namespace rclcpp;
using namespace nlohmann;
using StaticTransformBroadcaster = tf2_ros::StaticTransformBroadcaster;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using AprilTagDetectionArray = apriltag_msgs::msg::AprilTagDetectionArray;
using TransformStamped = geometry_msgs::msg::TransformStamped;
using TransformBuffer = tf2_ros::Buffer;
using TransformListener = tf2_ros::TransformListener;
geometry_msgs::msg::Transform subtractTransforms(const geometry_msgs::msg::Transform& transform1,
    const geometry_msgs::msg::Transform& transform2)
{
    geometry_msgs::msg::Transform result;
    result.translation.x = transform1.translation.x - transform2.translation.x;
    result.translation.y = transform1.translation.y - transform2.translation.y;
    result.translation.z = transform1.translation.z - transform2.translation.z;
    tf2::Quaternion q1, q2, q_result;
    tf2::fromMsg(transform1.rotation, q1);
    tf2::fromMsg(transform2.rotation, q2);
    tf2::Matrix3x3 m1(q1);
    tf2::Matrix3x3 m2(q2);
    tf2::Matrix3x3 m_result = m1.inverse() * m2;
    m_result.getRotation(q_result);
    result.rotation = tf2::toMsg(q_result);
    return result;
}
geometry_msgs::msg::Transform addTransforms(const geometry_msgs::msg::Transform& transform1,
    const geometry_msgs::msg::Transform& transform2)
{
    geometry_msgs::msg::Transform result;
    result.translation.x = transform1.translation.x + transform2.translation.x;
    result.translation.y = transform1.translation.y + transform2.translation.y;
    result.translation.z = transform1.translation.z + transform2.translation.z;
    tf2::Quaternion q1, q2, q_result;
    tf2::fromMsg(transform1.rotation, q1);
    tf2::fromMsg(transform2.rotation, q2);
    tf2::Matrix3x3 m1(q1);
    tf2::Matrix3x3 m2(q2);
    tf2::Matrix3x3 m_result = m1 * m2;
    m_result.getRotation(q_result);
    result.rotation = tf2::toMsg(q_result);
    return result;
}
namespace nfr_ros
{
    class NFRGPSNode : public Node
    {
    private:
        shared_ptr<StaticTransformBroadcaster> broadcaster;
        shared_ptr<TransformBuffer> buffer;
        shared_ptr<TransformListener> listener;
        vector<shared_ptr<Subscription<AprilTagDetectionArray>>> apriltagSubscribers;
        shared_ptr<Publisher<PoseStamped>> posePublisher;
        vector<string> cameraNames, detectorTopics;
    public:
        NFRGPSNode() : Node("nfr_gps_node")
        {
            broadcaster = std::make_shared<StaticTransformBroadcaster>(*this);
            buffer = std::make_shared<TransformBuffer>(get_clock());
            listener = std::make_shared<TransformListener>(*buffer);
            posePublisher = create_publisher<PoseStamped>("apriltag_estimations", 10);
            string tagPosesPath = declare_parameter("tag_poses", "");
            detectorTopics = declare_parameter("detector_topics", vector<string>());
            cameraNames = declare_parameter("camera_names", vector<string>());
            ifstream tagPosesFile(tagPosesPath);
            const auto& tagData = json::parse(tagPosesFile);
            const auto& tags = tagData["tags"];
            for (const auto& tag : tags)
            {
                int id = tag["ID"];
                RCLCPP_INFO(get_logger(), "Loading information for tag %d", id);
                TransformStamped transform;
                transform.header.frame_id = "field_origin";
                transform.header.stamp = get_clock()->now();
                transform.child_frame_id = "tag16h5:" + to_string(id);
                transform.transform.translation.x = tag["pose"]["translation"]["x"];
                transform.transform.translation.y = tag["pose"]["translation"]["y"];
                transform.transform.translation.z = tag["pose"]["translation"]["z"];
                transform.transform.rotation.w = tag["pose"]["rotation"]["quaternion"]["W"];
                transform.transform.rotation.x = tag["pose"]["rotation"]["quaternion"]["X"];
                transform.transform.rotation.y = tag["pose"]["rotation"]["quaternion"]["Y"];
                transform.transform.rotation.z = tag["pose"]["rotation"]["quaternion"]["Z"];
                broadcaster->sendTransform(transform);
            }
            tagPosesFile.close();
            for (size_t i = 0; i < detectorTopics.size(); i++)
            {
                apriltagSubscribers.emplace_back(create_subscription<AprilTagDetectionArray>(detectorTopics[i], 10,
                [&](const AprilTagDetectionArray& detections) {
                    RCLCPP_INFO(get_logger(), "Recieved tag detections");
                    for (const auto& detection : detections.detections)
                    {
                        string tagFrame = "tag16h5:" + to_string(detection.id);
                        if (buffer->canTransform(tagFrame, "field_origin", tf2::TimePointZero) &&
                            buffer->canTransform(tagFrame, cameraNames[i], tf2::TimePointZero) &&
                            buffer->canTransform(cameraNames[i], "base_link", tf2::TimePointZero))
                        {
                            TransformStamped fieldToTag = buffer->lookupTransform(tagFrame, "field_origin", tf2::TimePointZero);
                            TransformStamped cameraToTag = buffer->lookupTransform(tagFrame, cameraNames[i], tf2::TimePointZero);
                            TransformStamped baseToCamera = buffer->lookupTransform(cameraNames[i], "base_link", tf2::TimePointZero);
                            PoseStamped poseEstimation;
                            poseEstimation.header.frame_id = "base_link";
                            poseEstimation.header.stamp = cameraToTag.header.stamp;
                            const auto& transform = subtractTransforms(fieldToTag.transform,
                                addTransforms(cameraToTag.transform, baseToCamera.transform));
                            poseEstimation.pose.position.x = transform.translation.x;
                            poseEstimation.pose.position.y = transform.translation.y;
                            poseEstimation.pose.position.z = transform.translation.z;
                            poseEstimation.pose.orientation = transform.rotation;
                            posePublisher->publish(poseEstimation);
                        }
                    }
                }));
            }
        }
    };
}
int main(int argc, char const* argv[])
{
    init(argc, argv);
    const auto& node = make_shared<nfr_ros::NFRGPSNode>();
    spin(node);
    shutdown();
    return 0;
}