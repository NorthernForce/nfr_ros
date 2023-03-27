#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/MultiSubscriber.h>
#include <networktables/DoubleTopic.h>
#include <networktables/IntegerTopic.h>
#include <networktables/BooleanTopic.h>
using namespace std::chrono_literals;
namespace nfr
{
    class NFRBridgeNode : public rclcpp::Node
    {
    private:
        nt::NetworkTableInstance instance;
        std::shared_ptr<nt::NetworkTable> table, odometryTable, targetPoseTable, cmdVelTable, globalSetPoseTable, poseTable;
        nt::DoubleSubscriber odometryX, odometryY, odometryTheta, odometryDeltaX, odometryDeltaY, odometryDeltaTheta, targetPoseX, targetPoseY,
            targetPoseTheta, globalSetPoseX, globalSetPoseY, globalSetPoseTheta;
        nt::DoublePublisher cmdVelX, cmdVelY, cmdVelTheta, poseX, poseY, poseTheta;
        nt::IntegerSubscriber odometryStamp, targetPoseStamp, globalSetPoseStamp;
        nt::IntegerPublisher poseStamp;
        nt::BooleanEntry targetPoseCancel;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometryPublisher;
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr globalSetPosePublisher;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSusbscription;
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigateToPoseClient;
        rclcpp::TimerBase::SharedPtr timer;
        std::shared_future<std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>>> future;
        std::unique_ptr<tf2_ros::Buffer> buffer{nullptr};
        std::shared_ptr<tf2_ros::TransformListener> listener;
    public:
        NFRBridgeNode() : rclcpp::Node("nfr_bridge_node")
        {
            std::string tableName = declare_parameter("table_name", "xavier");
            odometryPublisher = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
            globalSetPosePublisher = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("global_set_pose", 10);
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
            instance.AddListener(odometryStamp, nt::EventFlags::kValueAll, std::bind(&NFRBridgeNode::recieveOdometry, this, std::placeholders::_1));
            targetPoseTable = table->GetSubTable("target_pose");
            targetPoseX = targetPoseTable->GetDoubleTopic("x").Subscribe(0.0);
            targetPoseY = targetPoseTable->GetDoubleTopic("y").Subscribe(0.0);
            targetPoseTheta = targetPoseTable->GetDoubleTopic("theta").Subscribe(0.0);
            targetPoseStamp = targetPoseTable->GetIntegerTopic("stamp").Subscribe(0);
            targetPoseCancel = targetPoseTable->GetBooleanTopic("cancel").GetEntry(false);
            navigateToPoseClient = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
            instance.AddListener(targetPoseStamp, nt::EventFlags::kValueAll,
                std::bind(&NFRBridgeNode::recieveTargetPose, this, std::placeholders::_1));
            globalSetPoseTable = table->GetSubTable("global_set_pose");
            globalSetPoseX = globalSetPoseTable->GetDoubleTopic("x").Subscribe(0.0);
            globalSetPoseY = globalSetPoseTable->GetDoubleTopic("y").Subscribe(0.0);
            globalSetPoseTheta = globalSetPoseTable->GetDoubleTopic("theta").Subscribe(0.0);
            globalSetPoseStamp = globalSetPoseTable->GetIntegerTopic("stamp").Subscribe(0);
            instance.AddListener(globalSetPoseStamp, nt::EventFlags::kValueAll, std::bind(&NFRBridgeNode::recieveGlobalSetPose, this, std::placeholders::_1));
            instance.AddListener(targetPoseCancel, nt::EventFlags::kValueAll, std::bind(&NFRBridgeNode::recieveCancel, this, std::placeholders::_1));
            cmdVelTable = table->GetSubTable("cmd_vel");
            cmdVelX = cmdVelTable->GetDoubleTopic("x").Publish();
            cmdVelY = cmdVelTable->GetDoubleTopic("y").Publish();
            cmdVelTheta = cmdVelTable->GetDoubleTopic("theta").Publish();
            cmdVelSusbscription = create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10,
                std::bind(&NFRBridgeNode::cmdVelCallback, this, std::placeholders::_1));
            poseTable = table->GetSubTable("pose");
            poseX = poseTable->GetDoubleTopic("x").Publish();
            poseY = poseTable->GetDoubleTopic("y").Publish();
            poseTheta = poseTable->GetDoubleTopic("theta").Publish();
            poseStamp = poseTable->GetIntegerTopic("stamp").Publish();
            buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            listener = std::make_shared<tf2_ros::TransformListener>(*buffer);
            timer = create_wall_timer(50ms, [&]() {
                if (buffer->canTransform("base_link", "map", tf2::TimePointZero))
                {
                    auto transform = buffer->lookupTransform("base_link", "map", tf2::TimePointZero);
                    poseX.Set(transform.transform.translation.x);
                    poseY.Set(transform.transform.translation.y);
                    tf2::Quaternion quaternion;
                    tf2::fromMsg(transform.transform.rotation, quaternion);
                    double roll, pitch, yaw;
                    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
                    poseTheta.Set(yaw);
                    auto stamp = rclcpp::Time(transform.header.stamp);
                    std::chrono::microseconds offset = (std::chrono::microseconds)instance.GetServerTimeOffset().value();
                    poseStamp.Set(((std::chrono::nanoseconds)(stamp + offset)).count())
                }
            });
            instance.SetServerTeam(172);
            instance.StartClient4("xavier");
            RCLCPP_INFO(get_logger(), "Started NT client4 as 'xavier'");
        }
        void recieveOdometry(const nt::Event& event)
        {
            nav_msgs::msg::Odometry odometry;
            odometry.child_frame_id = "base_link";
            odometry.header.frame_id = "odom";
            std::chrono::nanoseconds timestamp = (std::chrono::nanoseconds)odometryStamp.Get();
            odometry.header.stamp = rclcpp::Time(((std::chrono::nanoseconds)(timestamp - (std::chrono::microseconds)instance.GetServerTimeOffset().value())).count());
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
        void recieveGlobalSetPose(const nt::Event& event)
        {
            geometry_msgs::msg::PoseWithCovarianceStamped pose;
            std::chrono::nanoseconds timestamp = (std::chrono::nanoseconds)globalSetPoseStamp.Get();
            pose.header.stamp = rclcpp::Time((timestamp - (std::chrono::microseconds)instance.GetServerTimeOffset().value()).count());
            pose.header.frame_id = "map";
            pose.pose.pose.position.x = globalSetPoseX.Get();
            pose.pose.pose.position.y = globalSetPoseY.Get();
            tf2::Quaternion quaternion;
            quaternion.setRPY(0, 0, globalSetPoseTheta.Get());
            pose.pose.pose.orientation = tf2::toMsg(quaternion);
            globalSetPosePublisher->publish(pose);
        }
        void recieveTargetPose(const nt::Event& event)
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
            future = navigateToPoseClient->async_send_goal(goal);
        }
        void recieveCancel(const nt::Event& event)
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