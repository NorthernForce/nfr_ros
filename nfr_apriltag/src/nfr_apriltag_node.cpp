#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <map>
#include <nlohmann/json.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <fstream>
using namespace rclcpp;
using namespace std;
using Quaternion = tf2::Quaternion;
using TransformStamped = geometry_msgs::msg::TransformStamped;
using TransformBroadcaster = tf2_ros::TransformBroadcaster;
using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
using AprilTagDetectionArray = apriltag_msgs::msg::AprilTagDetectionArray;
using TransformBuffer = tf2_ros::Buffer;
using TransformListener = tf2_ros::TransformListener;
using TransformException = tf2::TransformException;
using Transform = geometry_msgs::msg::Transform;
using namespace ament_index_cpp;
using std::placeholders::_1;
Transform operator+(Transform a, Transform b)
{
    Transform c;
    c.translation.x = a.translation.x + b.translation.x;
    c.translation.y = a.translation.y + b.translation.y;
    c.translation.z = a.translation.z + b.translation.z;
    Quaternion qa;
    tf2::fromMsg(a.rotation, qa);
    Quaternion qb;
    tf2::fromMsg(b.rotation, qb);
    Quaternion qc = qa * qb;
    c.rotation = tf2::toMsg(qc);
    return c;
}
Transform operator-(Transform a, Transform b)
{
    Transform c;
    c.translation.x = a.translation.x - b.translation.x;
    c.translation.y = a.translation.y - b.translation.y;
    c.translation.z = a.translation.z - b.translation.z;
    Quaternion qa;
    tf2::fromMsg(a.rotation, qa);
    Quaternion qb;
    tf2::fromMsg(b.rotation, qb);
    Quaternion qc = qa * qb.inverse();
    c.rotation = tf2::toMsg(qc);
    return c;
}
class ApriltagNode : public Node
{
private:
    shared_ptr<TransformBroadcaster> broadcaster;
    shared_ptr<TransformListener> listener;
    shared_ptr<TransformBuffer> buffer;
    map<int, Transform> tagPoses;
    Publisher<PoseWithCovarianceStamped>::SharedPtr posePublisher;
    Subscription<AprilTagDetectionArray>::SharedPtr detectionsSubscription;
    string cameraFrame, baseFrame;
public:
    ApriltagNode() : Node("apriltag_node")
    {
        initializeTransforms();
        const auto& defaultFieldPath = filesystem::path(get_package_share_directory("nfr_charged_up"))
            / "config" / "field.json";
        string fieldPath = declare_parameter("field_path", defaultFieldPath);
        initializeFieldFromJson(fieldPath);
        cameraFrame = declare_parameter("camera_frame", "camera_link");
        baseFrame = declare_parameter("base_frame", "base_link");
        string poseTopic = declare_parameter("pose_topic", "pose_estimates");
        string detectionsTopic = declare_parameter("detections_topic", "detections");
        RCLCPP_INFO(get_logger(), "Initializing camera");
        initializeCamera(poseTopic, detectionsTopic);
    }
    void initializeTransforms()
    {
        broadcaster = std::make_shared<TransformBroadcaster>(*this);
        buffer = std::make_shared<TransformBuffer>(get_clock());
        listener = std::make_shared<TransformListener>(*buffer);
    }
    void initializeFieldFromJson(string fieldPath)
    {
        ifstream fieldFile(fieldPath);
        const auto& fieldRead = nlohmann::json::parse(fieldFile);
        for (const auto& tag : fieldRead["tags"])
        {
            Transform transform;
            transform.translation.x = tag["pose"]["translation"]["x"];
            transform.translation.y = tag["pose"]["translation"]["y"];
            transform.translation.z = tag["pose"]["translation"]["z"];
            transform.rotation.w = tag["pose"]["rotation"]["quaternion"]["W"];
            transform.rotation.x = tag["pose"]["rotation"]["quaternion"]["X"];
            transform.rotation.y = tag["pose"]["rotation"]["quaternion"]["Y"];
            transform.rotation.z = tag["pose"]["rotation"]["quaternion"]["Z"];
            tagPoses[tag["ID"]] = transform;
            RCLCPP_INFO(get_logger(), "Loaded tag %d at [%f, %f, %f]", (int)tag["ID"], transform.translation.x,
                transform.translation.y, transform.translation.z);
        }
    }
    void initializeCamera(string poseTopic, string detectionsTopic)
    {
        posePublisher = create_publisher<PoseWithCovarianceStamped>(poseTopic, 10);
        detectionsSubscription = create_subscription<AprilTagDetectionArray>(detectionsTopic, 10,
            bind(&ApriltagNode::cameraCallback, this, _1));
        RCLCPP_INFO(get_logger(), "Subscribed to apriltag detections");
    }
    void cameraCallback(const AprilTagDetectionArray& array)
    {
        const auto& timestamp = array.header.stamp;
        for (const auto& detection : array.detections)
        {
            if (tagPoses.find(detection.id) != tagPoses.end())
            {
                string tagFrame = "tag16h5:" + to_string(detection.id);
                TransformStamped cameraToTag;
                try
                {
                    cameraToTag = buffer->lookupTransform(tagFrame, cameraFrame, tf2::TimePointZero);
                }
                catch (const TransformException& e)
                {
                    RCLCPP_ERROR(get_logger(), "Caught transform exception when trying to lookup %s to %s: %s",
                        cameraFrame.c_str(), tagFrame.c_str(), e.what());
                    break;
                }
                TransformStamped baseToCamera;
                try
                {
                    baseToCamera = buffer->lookupTransform(cameraFrame, baseFrame, tf2::TimePointZero);
                }
                catch (const TransformException& e)
                {
                    RCLCPP_ERROR(get_logger(), "Caught transform exception when trying to lookup %s to %s: %s",
                        baseFrame.c_str(), cameraFrame.c_str(), e.what());
                    break;
                }
                Transform worldToTag = tagPoses[detection.id];
                Transform poseTransform = worldToTag - (cameraToTag.transform + baseToCamera.transform);
                PoseWithCovarianceStamped pose;
                pose.header.frame_id = baseFrame;
                pose.header.stamp = timestamp;
                pose.pose.pose.position.x = poseTransform.translation.x;
                pose.pose.pose.position.y = poseTransform.translation.y;
                pose.pose.pose.position.z = poseTransform.translation.z;
                pose.pose.pose.orientation = poseTransform.rotation;
                posePublisher->publish(pose);
            }
        }
    }
};
int main(int argc, char const *argv[])
{
    init(argc, argv);
    ApriltagNode::SharedPtr node = make_shared<ApriltagNode>();
    spin(node);
    shutdown();
    return 0;
}