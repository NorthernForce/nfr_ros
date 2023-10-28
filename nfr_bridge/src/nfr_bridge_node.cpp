#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/MultiSubscriber.h>
#include <networktables/DoubleTopic.h>
#include <networktables/IntegerTopic.h>
#include <networktables/BooleanTopic.h>
namespace nfr
{
    class NFRBridgeNode : public rclcpp::Node
    {
    private:
        nt::NetworkTableInstance instance;
        std::shared_ptr<nt::NetworkTable> table, odometryTable, targetPoseTable, cmdVelTable, globalPoseTable;
        nt::DoubleSubscriber odometryX, odometryY, odometryTheta, odometryDeltaX, odometryDeltaY, odometryDeltaTheta, targetPoseX, targetPoseY,
            targetPoseTheta, globalPoseX, globalPoseY, globalPoseTheta;
        nt::DoublePublisher cmdVelX, cmdVelY, cmdVelTheta;
        nt::IntegerSubscriber odometryStamp, targetPoseStamp, globalPoseStamp;
        nt::BooleanEntry targetPoseCancel;
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr globalPosePublisher;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometryPublisher;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSusbscription;
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigateToPoseClient;
    public:
        NFRBridgeNode() : rclcpp::Node("nfr_bridge_node")
        {
            std::string tableName = declare_parameter("table_name", "xavier");
            odometryPublisher = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
            globalPosePublisher = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("global_set_pose", 10);
            instance = nt::NetworkTableInstance::GetDefault();
            table = instance.GetTable(tableName);
            odometryTable = table->GetSubTable("odometry");
            odometryX = odometryTable->GetDoubleTopic("x").Subscribe(0.0);
            odometryY = odometryTable->GetDoubleTopic("y").Subscribe(0.0);
            odometryTheta = odometryTable->GetDoubleTopic("theta").Subscribe(0.0);
            odometryDeltaX = odometryTable->GetDoubleTopic("vx").Subscribe(0.0);
            odometryDeltaX = odometryTable->GetDoubleTopic("vy").Subscribe(0.0);
            odometryDeltaX = odometryTable->GetDoubleTopic("vtheta").Subscribe(0.0);
            odometryStamp = odometryTable->GetIntegerTopic("stamp").Subscribe(0);
            instance.AddListener(odometryStamp, nt::EventFlags::kValueAll, std::bind(&NFRBridgeNode::receiveOdometry, this, std::placeholders::_1));
            targetPoseTable = table->GetSubTable("target_pose");
            targetPoseX = targetPoseTable->GetDoubleTopic("x").Subscribe(0.0);
            targetPoseY = targetPoseTable->GetDoubleTopic("y").Subscribe(0.0);
            targetPoseTheta = targetPoseTable->GetDoubleTopic("theta").Subscribe(0.0);
            targetPoseStamp = targetPoseTable->GetIntegerTopic("stamp").Subscribe(0);
            targetPoseCancel = targetPoseTable->GetBooleanTopic("cancel").GetEntry(false);
            navigateToPoseClient = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
            instance.AddListener(targetPoseStamp, nt::EventFlags::kValueAll,
                std::bind(&NFRBridgeNode::receiveTargetPose, this, std::placeholders::_1));
            globalPoseTable = table->GetSubTable("global_set_pose");
            globalPoseX = globalPoseTable->GetDoubleTopic("x").Subscribe(0.0);
            globalPoseY = globalPoseTable->GetDoubleTopic("y").Subscribe(0.0);
            globalPoseTheta = globalPoseTable->GetDoubleTopic("theta").Subscribe(0.0);
            globalPoseStamp = globalPoseTable->GetIntegerTopic("stamp").Subscribe(0);
            instance.AddListener(globalPoseStamp, nt::EventFlags::kValueAll, std::bind(&NFRBridgeNode::receiveGlobalPose, this, std::placeholders::_1));
            instance.AddListener(targetPoseCancel, nt::EventFlags::kValueAll, std::bind(&NFRBridgeNode::receiveCancel, this, std::placeholders::_1));
            cmdVelTable = table->GetSubTable("cmd_vel");
            cmdVelX = cmdVelTable->GetDoubleTopic("x").Publish();
            cmdVelY = cmdVelTable->GetDoubleTopic("y").Publish();
            cmdVelTheta = cmdVelTable->GetDoubleTopic("theta").Publish();
            cmdVelSusbscription = create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10,
                std::bind(&NFRBridgeNode::cmdVelCallback, this, std::placeholders::_1));
            instance.SetServer("localhost");
            instance.StartClient4("xavier");
            RCLCPP_INFO(get_logger(), "Started NT client4 as 'xavier'");
        }
        void receiveGlobalPose(const nt::Event& event)
        {
            geometry_msgs::msg::PoseWithCovarianceStamped pose;
            pose.header.frame_id = "map";
            std::chrono::nanoseconds timestamp = (std::chrono::nanoseconds)globalPoseStamp.Get();
            pose.header.stamp = rclcpp::Time((timestamp - (std::chrono::microseconds)instance.GetServerTimeOffset().value()).count());
            pose.pose.pose.position.x = globalPoseX.Get();
            pose.pose.pose.position.y = globalPoseY.Get();
            tf2::Quaternion quaternion;
            quaternion.setRPY(0, 0, globalPoseTheta.Get());
            pose.pose.pose.orientation = tf2::toMsg(quaternion);
            globalPosePublisher->publish(pose);
        }
        void receiveOdometry(const nt::Event& event)
        {
            nav_msgs::msg::Odometry odometry;
            odometry.child_frame_id = "base_link";
            odometry.header.frame_id = "odom";
            std::chrono::nanoseconds timestamp = (std::chrono::nanoseconds)odometryStamp.Get();
            odometry.header.stamp = rclcpp::Time((timestamp - (std::chrono::microseconds)instance.GetServerTimeOffset().value()).count());
            odometry.pose.pose.position.x = odometryX.Get();
            odometry.pose.pose.position.y = odometryY.Get();
            odometry.twist.twist.linear.x = odometryDeltaX.Get();
            odometry.twist.twist.linear.y = odometryDeltaY.Get();
            odometry.twist.twist.angular.z = odometryDeltaTheta.Get();
            tf2::Quaternion quaternion;
            quaternion.setRPY(0, 0, odometryTheta.Get());
            odometry.pose.pose.orientation = tf2::toMsg(quaternion);
            odometryPublisher->publish(odometry);
        }
        void receiveTargetPose(const nt::Event& event)
        {
            RCLCPP_INFO(get_logger(), "Setting target pose");
            nav2_msgs::action::NavigateToPose::Goal goal;
            goal.behavior_tree = "";
            goal.pose.header.frame_id = "map";
            std::chrono::nanoseconds timestamp = (std::chrono::nanoseconds)targetPoseStamp.Get();
            goal.pose.header.stamp = rclcpp::Time((timestamp - (std::chrono::microseconds)instance.GetServerTimeOffset().value()).count());
            goal.pose.pose.position.x = targetPoseX.Get();
            goal.pose.pose.position.y = targetPoseY.Get();
            tf2::Quaternion quaternion;
            quaternion.setRPY(0, 0, targetPoseTheta.Get());
            goal.pose.pose.orientation = tf2::toMsg(quaternion);
            navigateToPoseClient->async_cancel_all_goals();
            navigateToPoseClient->async_send_goal(goal);
        }
        void receiveCancel(const nt::Event& event)
        {
            if (targetPoseCancel.Get())
            {
                RCLCPP_INFO(get_logger(), "Cancelling goal");
                navigateToPoseClient->async_cancel_all_goals();
                targetPoseCancel.Set(false);
            }
        }
        void cmdVelCallback(const geometry_msgs::msg::Twist cmdVel)
        {
            cmdVelX.Set(cmdVel.linear.x);
            cmdVelY.Set(cmdVel.linear.y);
            cmdVelTheta.Set(cmdVel.angular.z);
        }
    };
}
int main(int argc, const char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<nfr::NFRBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}