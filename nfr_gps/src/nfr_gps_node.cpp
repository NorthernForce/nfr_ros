#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <geometry_msgs/msg/pose.hpp>
#include <map>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
using namespace std;
using namespace rclcpp;
using namespace nlohmann;
using StaticTransformBroadcaster = tf2_ros::StaticTransformBroadcaster;
using Pose = geometry_msgs::msg::Pose;
using AprilTagDetectionArray = apriltag_msgs::msg::AprilTagDetectionArray;
using TransformStamped = geometry_msgs::msg::TransformStamped;
namespace nfr_ros
{
    class NFRGPSNode : public Node
    {
    private:
        shared_ptr<StaticTransformBroadcaster> broadcaster;
        map<int, Pose> poses;
    public:
        NFRGPSNode() : Node("nfr_gps_node")
        {
            broadcaster = std::make_shared<StaticTransformBroadcaster>(*this);
            string tagPosesPath = declare_parameter("tag_poses", "");
            ifstream tagPosesFile(tagPosesPath);
            const auto& tagData = json::parse(tagPosesFile);
            const auto& tags = tagData["tags"];
            for (const auto& tag : tags)
            {
                Pose pose;
                int id = tag["ID"];
                RCLCPP_INFO(get_logger(), "Loading information for tag %d", id);
                pose.position.x = tag["pose"]["translation"]["x"];
                pose.position.y = tag["pose"]["translation"]["y"];
                pose.position.z = tag["pose"]["translation"]["z"];
                RCLCPP_INFO(get_logger(), "Tag located at %f, %f, %f", pose.position.x, pose.position.y, pose.position.z);
                pose.orientation.w = tag["pose"]["rotation"]["quaternion"]["W"];
                pose.orientation.x = tag["pose"]["rotation"]["quaternion"]["X"];
                pose.orientation.y = tag["pose"]["rotation"]["quaternion"]["Y"];
                pose.orientation.z = tag["pose"]["rotation"]["quaternion"]["Z"];
                poses[id] = pose;
                TransformStamped transform;
                transform.header.frame_id = "map";
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