#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
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
        std::shared_ptr<nt::NetworkTable> table;
        struct
        {
            std::shared_ptr<nt::NetworkTable> table;
            nt::DoubleSubscriber deltaX, deltaY;
            nt::IntegerSubscriber stamp;
            rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher;
        } odometry;
        struct
        {
            std::shared_ptr<nt::NetworkTable> table;
            nt::DoubleSubscriber x, y, theta;
            nt::IntegerSubscriber stamp;
            nt::BooleanEntry cancel;
            rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client;
            std::shared_future<std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>>> future;
        } targetPose;
        struct
        {
            std::shared_ptr<nt::NetworkTable> table;
            nt::DoublePublisher x, y, theta;
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription;
        } cmdVel;
        struct
        {
            std::shared_ptr<nt::NetworkTable> table;
            nt::DoubleSubscriber x, y, theta;
            nt::IntegerSubscriber stamp;
            rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher;
        } globalSetPose;
        struct
        {
            std::shared_ptr<nt::NetworkTable> table;
            nt::DoublePublisher x, y, theta;
            nt::IntegerPublisher stamp;
            rclcpp::TimerBase::SharedPtr timer;
        } pose;
        struct Camera
        {
            std::shared_ptr<nt::NetworkTable> table;
            nt::DoublePublisher x, y, z, qx, qy, qz, qw;
            nt::IntegerPublisher stamp;
            rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription;
        };
        std::vector<Camera> cameras;
        struct
        {
            std::shared_ptr<nt::NetworkTable> table;
            nt::DoubleSubscriber theta;
            nt::IntegerSubscriber stamp;
            rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher;
        } imu;
        std::unique_ptr<tf2_ros::Buffer> buffer{nullptr};
        std::shared_ptr<tf2_ros::TransformListener> listener;
    public:
        NFRBridgeNode() : rclcpp::Node("nfr_bridge_node")
        {
            std::string tableName = declare_parameter("table_name", "xavier");
            instance = nt::NetworkTableInstance::GetDefault();
            table = instance.GetTable(tableName);
            {
                odometry.table = table->GetSubTable("odometry");
                odometry.deltaX = odometry.table->GetDoubleTopic("vx").Subscribe(0.0);
                odometry.deltaY = odometry.table->GetDoubleTopic("vy").Subscribe(0.0);
                odometry.stamp = odometry.table->GetIntegerTopic("stamp").Subscribe(0);
                odometry.publisher = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
                instance.AddListener(odometry.stamp, nt::EventFlags::kValueAll, std::bind(&NFRBridgeNode::recieveOdometry, this, std::placeholders::_1));
            }
            {
                imu.table = table->GetSubTable("imu");
                imu.theta = imu.table->GetDoubleTopic("theta").Subscribe(0.0);
                imu.stamp = imu.table->GetIntegerTopic("stamp").Subscribe(0);
                imu.publisher = create_publisher<sensor_msgs::msg::Imu>("imu", 10);
                instance.AddListener(imu.stamp, nt::EventFlags::kValueAll, std::bind(&NFRBridgeNode::receiveIMU, this, std::placeholders::_1));
            }
            {
                targetPose.table = table->GetSubTable("target_pose");
                targetPose.x = targetPose.table->GetDoubleTopic("x").Subscribe(0.0);
                targetPose.y = targetPose.table->GetDoubleTopic("y").Subscribe(0.0);
                targetPose.theta = targetPose.table->GetDoubleTopic("theta").Subscribe(0.0);
                targetPose.stamp = targetPose.table->GetIntegerTopic("stamp").Subscribe(0);
                targetPose.cancel = targetPose.table->GetBooleanTopic("cancel").GetEntry(false);
                targetPose.client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
                instance.AddListener(targetPose.stamp, nt::EventFlags::kValueAll,
                    std::bind(&NFRBridgeNode::recieveTargetPose, this, std::placeholders::_1));
                instance.AddListener(targetPose.cancel, nt::EventFlags::kValueAll, std::bind(&NFRBridgeNode::recieveCancel, this, std::placeholders::_1));
            }
            {
                globalSetPose.table = table->GetSubTable("global_set_pose");
                globalSetPose.x = globalSetPose.table->GetDoubleTopic("x").Subscribe(0.0);
                globalSetPose.y = globalSetPose.table->GetDoubleTopic("y").Subscribe(0.0);
                globalSetPose.theta = globalSetPose.table->GetDoubleTopic("theta").Subscribe(0.0);
                globalSetPose.stamp = globalSetPose.table->GetIntegerTopic("stamp").Subscribe(0);
                globalSetPose.publisher = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("global_set_pose", 10);
                instance.AddListener(globalSetPose.stamp, nt::EventFlags::kValueAll,
                    std::bind(&NFRBridgeNode::recieveGlobalSetPose, this, std::placeholders::_1));
            }
            {
                cmdVel.table = table->GetSubTable("cmd_vel");
                cmdVel.x = cmdVel.table->GetDoubleTopic("x").Publish();
                cmdVel.y = cmdVel.table->GetDoubleTopic("y").Publish();
                cmdVel.theta = cmdVel.table->GetDoubleTopic("theta").Publish();
                cmdVel.subscription = create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10,
                    std::bind(&NFRBridgeNode::cmdVelCallback, this, std::placeholders::_1));
            }
            {
                pose.table = table->GetSubTable("pose");
                pose.x = pose.table->GetDoubleTopic("x").Publish();
                pose.y = pose.table->GetDoubleTopic("y").Publish();
                pose.theta = pose.table->GetDoubleTopic("theta").Publish();
                pose.stamp = pose.table->GetIntegerTopic("stamp").Publish();
                pose.timer = create_wall_timer(50ms, [&]() {
                    if (!instance.IsConnected())
                    {
                        return;
                    }
                    if (buffer->canTransform("map", "base_link", tf2::TimePointZero))
                    {
                        auto transform = buffer->lookupTransform("map", "base_link", tf2::TimePointZero);
                        pose.x.Set(transform.transform.translation.x);
                        pose.y.Set(transform.transform.translation.y);
                        tf2::Quaternion quaternion;
                        tf2::fromMsg(transform.transform.rotation, quaternion);
                        double roll, pitch, yaw;
                        tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
                        pose.theta.Set(yaw);
                        auto stamp = rclcpp::Time(transform.header.stamp);
                        std::chrono::microseconds offset = (std::chrono::microseconds)instance.GetServerTimeOffset().value();
                        pose.stamp.Set(((std::chrono::nanoseconds)((std::chrono::nanoseconds)stamp.nanoseconds() + offset)).count());
                    }
                });
            }
            auto cameraNames = declare_parameter("camera_names", std::vector<std::string>());
            for (size_t i = 0; i < cameraNames.size(); i++)
            {
                cameras.push_back(Camera());
                Camera& camera = cameras[i];
                camera.table = table->GetSubTable(cameraNames[i]);
                camera.x = camera.table->GetDoubleTopic("x").Publish();
                camera.y = camera.table->GetDoubleTopic("y").Publish();
                camera.z = camera.table->GetDoubleTopic("z").Publish();
                camera.qx = camera.table->GetDoubleTopic("qx").Publish();
                camera.qy = camera.table->GetDoubleTopic("qy").Publish();
                camera.qz = camera.table->GetDoubleTopic("qz").Publish();
                camera.qw = camera.table->GetDoubleTopic("qw").Publish();
                camera.stamp = camera.table->GetIntegerTopic("stamp").Publish();
                camera.subscription = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(cameraNames[i] + "/pose_estimations",
                    10, [&](const geometry_msgs::msg::PoseWithCovarianceStamped& msg) {
                    camera.x.Set(msg.pose.pose.position.x);
                    camera.y.Set(msg.pose.pose.position.y);
                    camera.z.Set(msg.pose.pose.position.z);
                    camera.qx.Set(msg.pose.pose.orientation.x);
                    camera.qy.Set(msg.pose.pose.orientation.y);
                    camera.qz.Set(msg.pose.pose.orientation.z);
                    camera.qw.Set(msg.pose.pose.orientation.w);
                    auto stamp = rclcpp::Time(msg.header.stamp);
                    std::chrono::microseconds offset = (std::chrono::microseconds)instance.GetServerTimeOffset().value();
                    camera.stamp.Set(((std::chrono::nanoseconds)((std::chrono::nanoseconds)stamp.nanoseconds() + offset)).count());
                });
            }
            buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            listener = std::make_shared<tf2_ros::TransformListener>(*buffer);
            auto hosts = declare_parameter("hosts", std::vector<std::string>({"10.1.72.2:5810", "localhost:5810"}));
            instance.StartClient4("xavier");
            RCLCPP_INFO(get_logger(), "Started NT client4 as 'xavier'");
        }
        inline rclcpp::Time toHostTime(std::chrono::nanoseconds timestamp)
        {
            return rclcpp::Time(((std::chrono::nanoseconds)(timestamp -
                (std::chrono::microseconds)instance.GetServerTimeOffset().value())).count());
        }
        void recieveOdometry(const nt::Event& event)
        {
            (void)event;
            if (!instance.IsConnected())
            {
                return;
            }
            nav_msgs::msg::Odometry msg;
            msg.child_frame_id = "base_link";
            msg.header.frame_id = "odom";
            std::chrono::nanoseconds timestamp = (std::chrono::nanoseconds)odometry.stamp.Get();
            msg.header.stamp = toHostTime(timestamp);
            msg.twist.twist.linear.x = odometry.deltaX.Get();
            msg.twist.twist.linear.y = odometry.deltaY.Get();
            odometry.publisher->publish(msg);
        }
        void receiveIMU(const nt::Event& event)
        {
            (void)event;
            if (!instance.IsConnected())
            {
                return;
            }
            sensor_msgs::msg::Imu msg;
            msg.header.frame_id = "base_link";
            std::chrono::nanoseconds timestamp = (std::chrono::nanoseconds)imu.stamp.Get();
            msg.header.stamp = toHostTime(timestamp);
            tf2::Quaternion quaternion;
            quaternion.setRPY(0, 0, imu.theta.Get());
            msg.orientation = tf2::toMsg(quaternion);
            imu.publisher->publish(msg);
        }
        void recieveGlobalSetPose(const nt::Event& event)
        {
            (void)event;
            if (!instance.IsConnected())
            {
                return;
            }
            geometry_msgs::msg::PoseWithCovarianceStamped pose;
            std::chrono::nanoseconds timestamp = (std::chrono::nanoseconds)globalSetPose.stamp.Get();
            pose.header.stamp = toHostTime(timestamp);
            pose.header.frame_id = "map";
            pose.pose.pose.position.x = globalSetPose.x.Get();
            pose.pose.pose.position.y = globalSetPose.y.Get();
            tf2::Quaternion quaternion;
            quaternion.setRPY(0, 0, globalSetPose.theta.Get());
            pose.pose.pose.orientation = tf2::toMsg(quaternion);
            globalSetPose.publisher->publish(pose);
        }
        void recieveTargetPose(const nt::Event& event)
        {
            (void)event;
            if (!instance.IsConnected())
            {
                return;
            }
            RCLCPP_INFO(get_logger(), "Setting target pose");
            nav2_msgs::action::NavigateToPose::Goal goal;
            goal.behavior_tree = "";
            goal.pose.header.frame_id = "map";
            std::chrono::nanoseconds timestamp = (std::chrono::nanoseconds)targetPose.stamp.Get();
            goal.pose.header.stamp = toHostTime(timestamp);
            goal.pose.pose.position.x = targetPose.x.Get();
            goal.pose.pose.position.y = targetPose.y.Get();
            tf2::Quaternion quaternion;
            quaternion.setRPY(0, 0, targetPose.theta.Get());
            goal.pose.pose.orientation = tf2::toMsg(quaternion);
            targetPose.future = targetPose.client->async_send_goal(goal);
        }
        void recieveCancel(const nt::Event& event)
        {
            (void)event;
            if (!instance.IsConnected())
            {
                return;
            }
            if (targetPose.cancel.Get())
            {
                RCLCPP_INFO(get_logger(), "Cancelling goal");
                targetPose.client->async_cancel_all_goals();
                targetPose.cancel.Set(false);
            }
        }
        void cmdVelCallback(const geometry_msgs::msg::Twist msg)
        {
            if (!instance.IsConnected())
            {
                return;
            }
            cmdVel.x.Set(msg.linear.x);
            cmdVel.y.Set(msg.linear.y);
            cmdVel.theta.Set(msg.angular.z);
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