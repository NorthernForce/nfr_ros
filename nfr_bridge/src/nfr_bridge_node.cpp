#include <rclcpp/rclcpp.hpp>
#include <ntcore/networktables/NetworkTableInstance.h>
#include <ntcore/networktables/NetworkTable.h>
#include <tinyxml2.h>
#include <cscore/cscore_oo.h>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
using namespace rclcpp;
using namespace std;
using namespace tinyxml2;
using Odometry = nav_msgs::msg::Odometry;
using Image = sensor_msgs::msg::Image;
using Quaternion = tf2::Quaternion;
using TransformStamped = geometry_msgs::msg::TransformStamped;
using TransformBroadcaster = tf2_ros::TransformBroadcaster;
using TransformListener = tf2_ros::TransformListener;
using TransformBuffer = tf2_ros::Buffer;
using AprilTagDetectionArray = apriltag_msgs::msg::AprilTagDetectionArray;
namespace nfr_ros
{
    class NFRBridgeNode : public Node
    {
    private:
        nt::NetworkTableInstance instance;
        shared_ptr<nt::NetworkTable> rosTable, odometryTable;
        shared_ptr<Publisher<Odometry>> odometryPublisher;
        shared_ptr<TransformBroadcaster> broadcaster;
        shared_ptr<TransformListener> listener;
        shared_ptr<TransformBuffer> buffer;
    public:
        NFRBridgeNode() : Node("nfr_bridge_node")
        {
            XMLDocument document;
            document.LoadFile(declare_parameter("topics_xml", "").c_str());
            if (document.Error())
            {
                RCLCPP_ERROR(get_logger(), "Could not load %s", get_parameter("topics_xml").get_value<string>());
            }
            else
            {
                instance = nt::NetworkTableInstance::Create();
                instance.SetServerTeam(declare_parameter("team", 172));
                instance.StartClient4(declare_parameter("client_name", "coprocessor"));
                rosTable = instance.GetTable(declare_parameter("ros_table", "ros"));
                broadcaster = std::make_shared<TransformBroadcaster>(*this);
                buffer = std::make_shared<TransformBuffer>(get_clock());
                listener = std::make_shared<TransformListener>(*buffer);
                const auto& bridge = document.FirstChildElement("bridge");
                const auto& imports = bridge->FirstChildElement("imports");
                const auto& exports = bridge->FirstChildElement("exports");
                for (auto import = imports->FirstChildElement(); import; import = import->NextSiblingElement())
                {
                    if (string(import->Name()) == "odometry")
                    {
                        odometryTable = rosTable->GetSubTable(import->FirstChildElement("source_topic")->Value());
                        odometryPublisher = create_publisher<Odometry>(
                            string(import->FirstChildElement("target_topic")->Value()),
                            10
                        );
                        create_wall_timer(10ms, [&]() {
                            double timestamp = odometryTable->GetEntry("timestamp").GetDouble(0);
                            Odometry odom;
                            odom.header.frame_id = "odom";
                            odom.child_frame_id = "base_link";
                            odom.header.stamp = Time(timestamp);
                            odom.pose.pose.position.x = odometryTable->GetEntry("x").GetDouble(0);
                            odom.pose.pose.position.y = odometryTable->GetEntry("y").GetDouble(0);
                            odom.pose.pose.position.z = 0;
                            Quaternion quaternion;
                            quaternion.setRPY(0, 0, odometryTable->GetEntry("theta").GetDouble(0));
                            tf2::convert(quaternion, odom.pose.pose.orientation);
                            odom.twist.twist.linear.x = odometryTable->GetEntry("vx").GetDouble(0);
                            odom.twist.twist.linear.y = odometryTable->GetEntry("vy").GetDouble(0);
                            odom.twist.twist.angular.z = odometryTable->GetEntry("vtheta").GetDouble(0);
                            odometryPublisher->publish(odom);
                            TransformStamped transform;
                            transform.header.frame_id = "odom";
                            transform.header.stamp = Time(timestamp);
                            transform.child_frame_id = "base_link";
                            transform.transform.translation.x = odom.pose.pose.position.x;
                            transform.transform.translation.y = odom.pose.pose.position.y;
                            transform.transform.translation.z = 0;
                            transform.transform.rotation = odom.pose.pose.orientation;
                            broadcaster->sendTransform(transform);
                        });
                    }
                }
                for (auto _export = exports->FirstChildElement(); _export; _export = _export->NextSiblingElement())
                {
                    if (string(_export->Name()) == "camera")
                    {
                        RCLCPP_WARN(get_logger(), "Cameras are not supported at the moment.");
                    }
                    else if (string(_export->Name()) == "apriltag_detector")
                    {
                        if (string(_export->Attribute("isaac_ros")) == "false")
                        {
                            string cameraName = _export->FirstChildElement("camera_name")->Value();
                            const auto& detectorTable = rosTable->GetSubTable(_export->FirstChildElement("target_topic")->Value());
                            const auto& subscriber = create_subscription<AprilTagDetectionArray>(
                                _export->FirstChildElement("source_topic")->Value(), 10, [&](const AprilTagDetectionArray& array) {
                                double timestamp = array.header.stamp.nanosec;
                                std::vector<long> ids;
                                for (const auto& detection : array.detections)
                                {
                                    ids.emplace_back(detection.id);
                                    try
                                    {
                                        TransformStamped transform = buffer->lookupTransform(
                                            detection.family + ":" + to_string(detection.id),
                                            cameraName,
                                            tf2::TimePointZero
                                        );
                                        const auto& detectionTable = detectorTable->GetSubTable(
                                            detection.family + ":" + to_string(detection.id));
                                        detectionTable->GetEntry("x").SetDouble(transform.transform.translation.x);
                                        detectionTable->GetEntry("y").SetDouble(transform.transform.translation.y);
                                        detectionTable->GetEntry("z").SetDouble(transform.transform.translation.z);
                                        detectionTable->GetEntry("rotation").SetDoubleArray(std::vector<double>{
                                            transform.transform.rotation.w,
                                            transform.transform.rotation.x,
                                            transform.transform.rotation.y,
                                            transform.transform.rotation.z
                                        });
                                    }
                                    catch (tf2::LookupException& e)
                                    {
                                        RCLCPP_ERROR(get_logger(), "Could not find transform from %s to %s: %s",
                                            cameraName, detection.family + ":" + to_string(detection.id), e.what());
                                    }
                                }
                                detectorTable->GetEntry("timestamp").SetDouble(timestamp);
                                detectorTable->GetEntry("ids").SetIntegerArray(ids);
                            });
                        }
                        else
                        {
                            RCLCPP_WARN(get_logger(), "Nvidia ISAAC ROS not supported at the moment.");
                        }
                    }
                    else if (string(_export->Name()) == "pose")
                    {
                        const auto& table = rosTable->GetSubTable(_export->FirstChildElement("target_topic")->Value());
                        string sourceFrame = _export->FirstChildElement("source_frame")->Value();
                        string targetFrame = _export->FirstChildElement("target_frame")->Value();
                        create_wall_timer(10ms, [&]() {
                            try
                            {
                                TransformStamped transform = buffer->lookupTransform(
                                    targetFrame,
                                    sourceFrame,
                                    tf2::TimePointZero
                                );
                                table->GetEntry("timestamp").SetDouble(transform.header.stamp.nanosec);
                                table->GetEntry("x").SetDouble(transform.transform.translation.x);
                                table->GetEntry("y").SetDouble(transform.transform.translation.y);
                                table->GetEntry("z").SetDouble(transform.transform.translation.z);
                                table->GetEntry("rotation").SetDoubleArray(std::vector<double>{
                                    transform.transform.rotation.w,
                                    transform.transform.rotation.x,
                                    transform.transform.rotation.y,
                                    transform.transform.rotation.z
                                });
                            }
                            catch (tf2::LookupException& e)
                            {
                                RCLCPP_ERROR(get_logger(), "Could not find transform from %s to %s: %s",
                                    sourceFrame, targetFrame, e.what());
                            }
                        });
                    }
                }
            }
        }
    };
}
int main(int argc, char const* argv[])
{
    init(argc, argv);
    const auto& node = make_shared<nfr_ros::NFRBridgeNode>();
    spin(node);
    shutdown();
    return 0;
}