#include <rclcpp/rclcpp.hpp>
#include <ntcore/networktables/NetworkTableInstance.h>
#include <ntcore/networktables/NetworkTable.h>
#include <ntcore/networktables/NetworkTableEntry.h>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
using namespace rclcpp;
using namespace std;
using NetworkTableInstance = nt::NetworkTableInstance;
using NetworkTable = nt::NetworkTable;
using NetworkTableEntry = nt::NetworkTableEntry;
using Odometry = nav_msgs::msg::Odometry;
using NetworkTableEventFlags = nt::EventFlags;
using NetworkTableEvent = nt::Event;
using Quaternion = tf2::Quaternion;
using TransformStamped = geometry_msgs::msg::TransformStamped;
using TransformBroadcaster = tf2_ros::TransformBroadcaster;
using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
class SquishyNode : public Node
{
private:
    NetworkTableInstance instance;
    shared_ptr<NetworkTable> odometryTable, setPoseTable;
    NetworkTableEntry odomX, odomY, odomTheta, odomDeltaX, odomDeltaY, odomDeltaTheta, odomTimestamp,
        setPoseX, setPoseY, setPoseTheta, setPoseDirty;
    Publisher<Odometry>::SharedPtr odometryPublisher;
    Publisher<PoseWithCovarianceStamped>::SharedPtr setPosePublisher;
    TimerBase::SharedPtr odometryTimer, setPoseTimer;
    shared_ptr<TransformBroadcaster> broadcaster;
public:
    SquishyNode() : Node("squishy_node")
    {
        instance = NetworkTableInstance::Create();
        instance.SetServerTeam(declare_parameter("team_number", 172));
        instance.StartClient4(declare_parameter("nt_name", "xavier"));
        RCLCPP_INFO(get_logger(), "Starting NetworkTable Client4 to connect to team %d as '%s'",
            get_parameter("team_number").as_int(), get_parameter("nt_name").as_string().c_str());
        odometryTable = instance.GetTable("/ros/odom");
        odomX = odometryTable->GetEntry("x");
        odomY = odometryTable->GetEntry("y");
        odomTheta = odometryTable->GetEntry("theta");
        odomDeltaX = odometryTable->GetEntry("vx");
        odomDeltaY = odometryTable->GetEntry("vy");
        odomDeltaTheta = odometryTable->GetEntry("vtheta");
        odomTimestamp = odometryTable->GetEntry("timestamp");
        RCLCPP_INFO(get_logger(), "Got entry handles for odometry");
        odometryPublisher = create_publisher<Odometry>("odom", 10);
        broadcaster = std::make_shared<TransformBroadcaster>(*this);
        RCLCPP_INFO(get_logger(), "Created publisher & broadcaster for odometry");
        odometryTimer = create_wall_timer(10ms, [&]() {
            double timestamp = odomTimestamp.GetDouble(0);
            double x = odomX.GetDouble(0);
            double y = odomY.GetDouble(0);
            double theta = odomTheta.GetDouble(0);
            double vx = odomDeltaX.GetDouble(0);
            double vy = odomDeltaY.GetDouble(0);
            double vtheta = odomDeltaTheta.GetDouble(0);
            Odometry odometry;
            odometry.child_frame_id = "base_link";
            odometry.header.frame_id = "odom";
            odometry.header.stamp = get_clock()->now();
            odometry.pose.pose.position.x = x;
            odometry.pose.pose.position.y = y;
            odometry.pose.pose.position.z = 0;
            Quaternion quaternion;
            quaternion.setRPY(0, 0, theta);
            odometry.pose.pose.orientation = tf2::toMsg(quaternion);
            odometry.twist.twist.linear.x = vx;
            odometry.twist.twist.linear.y = vy;
            odometry.twist.twist.linear.z = 0;
            odometry.twist.twist.angular.x = 0;
            odometry.twist.twist.angular.y = 0;
            odometry.twist.twist.angular.z = vtheta;
            odometryPublisher->publish(odometry);
            TransformStamped transform;
            transform.header = odometry.header;
            transform.child_frame_id = odometry.child_frame_id;
            transform.transform.translation.x = x;
            transform.transform.translation.y = y;
            transform.transform.translation.z = 0;
            transform.transform.rotation = odometry.pose.pose.orientation;
            broadcaster->sendTransform(transform);
        });
        RCLCPP_INFO(get_logger(), "Started odometry timer at 10ms interval");
        setPoseTable = instance.GetTable("/ros/set_pose");
        setPoseX = setPoseTable->GetEntry("x");
        setPoseY = setPoseTable->GetEntry("y");
        setPoseTheta = setPoseTable->GetEntry("theta");
        setPoseDirty = setPoseTable->GetEntry("dirty");
        setPosePublisher = create_publisher<PoseWithCovarianceStamped>("set_pose", 10);
        setPoseTimer = create_wall_timer(50ms, [&]()
        {
            if (setPoseDirty.GetBoolean(false))
            {
                setPoseDirty.SetBoolean(false);
                PoseWithCovarianceStamped pose;
                pose.header.stamp = get_clock()->now();
                pose.pose.pose.position.x = setPoseX.GetDouble(0);
                pose.pose.pose.position.y = setPoseY.GetDouble(0);
                pose.pose.pose.position.z = 0;
                Quaternion quaternion;
                quaternion.setRPY(0, 0, setPoseTheta.GetDouble(0));
                pose.pose.pose.orientation = tf2::toMsg(quaternion);
                setPosePublisher->publish(pose);
            }
        });
    }
};
int main(int argc, char const *argv[])
{
    init(argc, argv);
    SquishyNode::SharedPtr node = make_shared<SquishyNode>();
    spin(node);
    shutdown();
    return 0;
}