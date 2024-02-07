#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/MultiSubscriber.h>
#include <networktables/DoubleTopic.h>
#include <networktables/IntegerTopic.h>
#include <networktables/BooleanTopic.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/StructTopic.h>
#include <networktables/StructArrayTopic.h>
#include <networktables/IntegerArrayTopic.h>
#include <nfr_msgs/msg/target_list.hpp>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Twist2d.h>
#include <fuse_msgs/srv/set_pose.hpp>
double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& quat)
{
    double roll, pitch, yaw;
    tf2::Quaternion quaternion;
    tf2::fromMsg(quat, quaternion);
    tf2::Matrix3x3 mat(quaternion);
    mat.getRPY(roll, pitch, yaw);
    return yaw;
}
using namespace std::chrono_literals;
template<>
struct wpi::Struct<nfr_msgs::msg::Target>
{
    static constexpr std::string_view GetTypeString()
    {
        return "struct:TargetDetection";
    }
    static constexpr size_t GetSize()
    {
        return sizeof(double) * 6 + sizeof(long);
    }
    static constexpr std::string_view GetSchema() {
        return "double area; double tx; double ty; double pitch; double yaw; double depth; long fiducialID";
    }
    static nfr_msgs::msg::Target Unpack(std::span<const uint8_t> data)
    {
        nfr_msgs::msg::Target target;
        target.area = wpi::UnpackStruct<double, 0>(data);
        target.center.x = (int)wpi::UnpackStruct<double, sizeof(double)>(data);
        target.center.y = (int)wpi::UnpackStruct<double, sizeof(double) * 2>(data);
        target.pitch = wpi::UnpackStruct<double, sizeof(double) * 3>(data);
        target.yaw = wpi::UnpackStruct<double, sizeof(double) * 4>(data);
        target.depth = wpi::UnpackStruct<double, sizeof(double) * 5>(data);
        target.fiducial_id = wpi::UnpackStruct<long, sizeof(double) * 6>(data);
    }
    static void Pack(std::span<uint8_t> data, const nfr_msgs::msg::Target& value)
    {
        wpi::PackStruct<0>(data, value.area);
        wpi::PackStruct<sizeof(double)>(data, (double)value.center.x);
        wpi::PackStruct<sizeof(double) * 2>(data, (double)value.center.x);
        wpi::PackStruct<sizeof(double) * 3>(data, value.pitch);
        wpi::PackStruct<sizeof(double) * 4>(data, value.yaw);
        wpi::PackStruct<sizeof(double) * 5>(data, value.depth);
        wpi::PackStruct<sizeof(double) * 6>(data, value.fiducial_id);
    }
    static void ForEachNested(std::invocable<std::string_view, std::string_view> auto fn)
    {
    }
};
namespace nfr
{
    class NFRBridgeNode : public rclcpp::Node
    {
    private:
        nt::NetworkTableInstance instance;
        std::shared_ptr<nt::NetworkTable> table;
        nt::StructSubscriber<frc::Pose2d> poseSubscriber;
        nt::StructSubscriber<frc::Twist2d> odomSubscriber;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPublisher;
        struct
        {
            nt::StructSubscriber<frc::Pose2d> subscriber;
            nt::BooleanEntry cancel;
            rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client;
            std::shared_future<std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>>> future;
        } targetPose;
        struct
        {
            nt::StructPublisher<frc::Twist2d> publisher;
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription;
        } cmdVel;
        struct
        {
            nt::StructSubscriber<frc::Pose2d> subscriber;
            rclcpp::Client<fuse_msgs::srv::SetPose>::SharedPtr publisher;
        } globalSetPose;
        struct
        {
            nt::StructPublisher<frc::Pose2d> publisher;
            rclcpp::TimerBase::SharedPtr timer;
        } pose;
        struct TargetCamera
        {
            nt::StructArrayPublisher<nfr_msgs::msg::Target> publisher;
            rclcpp::Subscription<nfr_msgs::msg::TargetList>::SharedPtr subscription;
        };
        std::vector<TargetCamera> cameras;
        struct PoseSupplier
        {
            nt::StructPublisher<frc::Pose2d> publisher;
            rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription;
        };
        std::vector<PoseSupplier> poseSuppliers;
        std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster;
        std::unique_ptr<tf2_ros::TransformListener> listener;
        std::unique_ptr<tf2_ros::Buffer> buffer;
    public:
        NFRBridgeNode() : rclcpp::Node("nfr_bridge_node")
        {
            std::string tableName = declare_parameter("table_name", "xavier");
            instance = nt::NetworkTableInstance::GetDefault();
            table = instance.GetTable(tableName);
            instance.SetServerTeam(declare_parameter("team_number", 172));
            {
                odomSubscriber = table->GetStructTopic<frc::Twist2d>("odometry").Subscribe(frc::Twist2d());
                odomPublisher = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
                instance.AddListener(odomSubscriber, nt::EventFlags::kValueAll, std::bind(&NFRBridgeNode::recieveOdometry, this, std::placeholders::_1));
            }
            {
                targetPose.subscriber = table->GetStructTopic<frc::Pose2d>("target_pose").Subscribe(frc::Pose2d());
                targetPose.cancel = table->GetBooleanTopic("cancel").GetEntry(false);
                targetPose.client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
                instance.AddListener(targetPose.subscriber, nt::EventFlags::kValueAll,
                    std::bind(&NFRBridgeNode::recieveTargetPose, this, std::placeholders::_1));
                instance.AddListener(targetPose.cancel, nt::EventFlags::kValueAll, std::bind(&NFRBridgeNode::recieveCancel, this, std::placeholders::_1));
            }
            {
                cmdVel.publisher = table->GetStructTopic<frc::Twist2d>("cmd_vel").Publish();
                cmdVel.subscription = create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10,
                    std::bind(&NFRBridgeNode::cmdVelCallback, this, std::placeholders::_1));
            }
            {
                globalSetPose.subscriber = table->GetStructTopic<frc::Pose2d>("global_set_pose").Subscribe(frc::Pose2d());
                globalSetPose.publisher = create_client<fuse_msgs::srv::SetPose>("/global_localization_node/set_pose", 10);
                instance.AddListener(globalSetPose.subscriber, nt::EventFlags::kValueAll,
                    std::bind(&NFRBridgeNode::recieveGlobalSetPose, this, std::placeholders::_1));
            }
            {
                pose.publisher = table->GetStructTopic<frc::Pose2d>("pose").Publish();
                pose.timer = create_wall_timer(20ms, std::bind(&NFRBridgeNode::publishPose, this));
            }
            auto cameraNames = declare_parameter("target_cameras", std::vector<std::string>());
            for (size_t i = 0; i < cameraNames.size(); i++)
            {
                cameras.push_back(TargetCamera());
                cameras[i].publisher = table->GetStructArrayTopic<nfr_msgs::msg::Target>(cameraNames[i]).Publish();
                std::function<void(const nfr_msgs::msg::TargetList&)> cameraCallback =
                    std::bind(&NFRBridgeNode::sendDetection, this, std::placeholders::_1, i);
                cameras[i].subscription = create_subscription<nfr_msgs::msg::TargetList>(cameraNames[i] + "/targets", 10,
                    cameraCallback);
            }
            auto poseNames = declare_parameter("pose_suppliers", std::vector<std::string>());
            for (size_t i = 0; i < poseNames.size(); i++)
            {
                poseSuppliers.push_back(PoseSupplier());
                poseSuppliers[i].publisher = table->GetSubTable("poses")->GetStructTopic<frc::Pose2d>(poseNames[i]).Publish();
                std::function<void(const geometry_msgs::msg::PoseWithCovarianceStamped&)> poseCallback =
                    std::bind(&NFRBridgeNode::sendPose, this, std::placeholders::_1, i);
                poseSuppliers[i].subscription = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(poseNames[i] + "/pose_estimations", 10,
                    poseCallback);
            }
            broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
            buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
            listener = std::make_unique<tf2_ros::TransformListener>(*buffer);
            instance.SetServerTeam(172);
            std::string clientName = declare_parameter("client_name", "xavier");
            instance.StartClient4(clientName);
            RCLCPP_INFO(get_logger(), "Started NT client4 as '%s'", clientName.c_str());
        }
        inline rclcpp::Time toHostTime(std::chrono::nanoseconds timestamp)
        {
            return rclcpp::Time(((std::chrono::nanoseconds)(timestamp -
                (std::chrono::microseconds)instance.GetServerTimeOffset().value_or(0))).count());
        }
        void sendPose(const geometry_msgs::msg::PoseWithCovarianceStamped& msg, int i)
        {
            PoseSupplier& supplier = poseSuppliers[i];
            std::chrono::microseconds offset = (std::chrono::microseconds)instance.GetServerTimeOffset().value_or(0);
            supplier.publisher.Set(
                frc::Pose2d((units::meter_t)msg.pose.pose.position.x, (units::meter_t)msg.pose.pose.position.y,
                    frc::Rotation2d((units::radian_t)getYawFromQuaternion(msg.pose.pose.orientation))), rclcpp::Time(msg.header.stamp).nanoseconds());
        }
        void sendDetection(const nfr_msgs::msg::TargetList& msg, int i)
        {
            cameras[i].publisher.Set(msg.targets, rclcpp::Time(msg.header.stamp).nanoseconds());
        }
        void recieveOdometry(const nt::Event& event)
        {
            (void)event;
            if (!instance.IsConnected())
            {
                return;
            }
            auto odometry = odomSubscriber.GetAtomic();
            nav_msgs::msg::Odometry msg;
            msg.child_frame_id = "base_link";
            msg.header.frame_id = "odom";
            msg.header.stamp = rclcpp::Time(odometry.time);
            msg.twist.twist.linear.x = (double)odometry.value.dx;
            msg.twist.twist.linear.y = (double)odometry.value.dy;
            msg.twist.twist.angular.z = (double)odometry.value.dtheta;
            msg.twist.covariance = {
                0.81, 0, 0, 0, 0, 0,
                0, 0.81, 0, 0, 0, 0,
                0, 0, 1e6, 0, 0, 0,
                0, 0, 0, 1e6, 0, 0,
                0, 0, 0, 0, 1e6, 0,
                0, 0, 0, 0, 0, 0.81
            };
            odomPublisher->publish(msg);
        }
        void recieveGlobalSetPose(const nt::Event& event)
        {
            (void)event;
            if (!instance.IsConnected())
            {
                return;
            }
            geometry_msgs::msg::PoseWithCovarianceStamped pose;
            auto global = globalSetPose.subscriber.GetAtomic();
            pose.header.stamp = rclcpp::Time(global.time);
            pose.header.frame_id = "map";
            pose.pose.pose.position.x = (double)global.value.X();
            pose.pose.pose.position.y = (double)global.value.Y();
            pose.pose.covariance = {
                1e-6, 0, 0, 0, 0, 0,
                0, 1e-6, 0, 0, 0, 0,
                0, 0, 1e6, 0, 0, 0,
                0, 0, 0, 1e6, 0, 0,
                0, 0, 0, 0, 1e6, 0,
                0, 0, 0, 0, 0, 1e-6
            };
            tf2::Quaternion quaternion;
            quaternion.setRPY(0, 0, (double)global.value.Rotation().Radians());
            pose.pose.pose.orientation = tf2::toMsg(quaternion);
            fuse_msgs::srv::SetPose::Request request;
            request.pose = pose;
            globalSetPose.publisher->async_send_request(std::make_shared<fuse_msgs::srv::SetPose::Request>(request));
        }
        void publishPose()
        {
            if (buffer->canTransform("map", "base_link", tf2::TimePointZero) && instance.IsConnected())
            {
                auto transform = buffer->lookupTransform("map", "base_link", tf2::TimePointZero);
                pose.publisher.Set(frc::Pose2d((units::meter_t)transform.transform.translation.x, (units::meter_t)transform.transform.translation.y,
                    (units::radian_t)getYawFromQuaternion(transform.transform.rotation)),
                    ((rclcpp::Time)transform.header.stamp).nanoseconds());
            }
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
            auto pose = targetPose.subscriber.GetAtomic();
            goal.behavior_tree = "";
            goal.pose.header.frame_id = "map";
            goal.pose.header.stamp = rclcpp::Time(pose.time);
            goal.pose.pose.position.x = (double)pose.value.X();
            goal.pose.pose.position.y = (double)pose.value.Y();
            tf2::Quaternion quaternion;
            quaternion.setRPY(0, 0, (double)pose.value.Rotation().Radians());
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
            cmdVel.publisher.Set(frc::Twist2d((units::meter_t)msg.linear.x, (units::meter_t)msg.linear.y, (units::radian_t)msg.angular.z));
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