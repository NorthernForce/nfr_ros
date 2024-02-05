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
#include <networktables/IntegerArrayTopic.h>
#include <nfr_msgs/msg/target_list.hpp>
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
            nt::DoubleSubscriber x, y, theta, deltaX, deltaY, deltaTheta;
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
            rclcpp::Client<fuse_msgs::srv::SetPose>::SharedPtr publisher;
        } globalSetPose;
        // struct
        // {
        //     std::shared_ptr<nt::NetworkTable> table;
        //     nt::DoubleSubscriber x, y, theta;
        //     nt::IntegerSubscriber stamp;
        // } pose;
        struct
        {
            std::shared_ptr<nt::NetworkTable> table;
            nt::DoublePublisher x, y, theta;
            nt::IntegerPublisher stamp;
            rclcpp::TimerBase::SharedPtr timer;
        } pose;
        struct TargetCamera
        {
            std::shared_ptr<nt::NetworkTable> table;
            nt::DoubleArrayPublisher area, pitch, yaw, tx, ty, depth, distance;
            nt::IntegerArrayPublisher fiducialID, stamp;
            rclcpp::Subscription<nfr_msgs::msg::TargetList>::SharedPtr subscription;
        };
        std::vector<TargetCamera> cameras;
        struct PoseSupplier
        {
            std::shared_ptr<nt::NetworkTable> table;
            nt::DoublePublisher x, y, theta;
            nt::IntegerPublisher stamp;
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
                odometry.table = table->GetSubTable("odometry");
                odometry.deltaX = odometry.table->GetDoubleTopic("vx").Subscribe(0.0);
                odometry.deltaY = odometry.table->GetDoubleTopic("vy").Subscribe(0.0);
                odometry.deltaTheta = odometry.table->GetDoubleTopic("vtheta").Subscribe(0.0);
                odometry.x = odometry.table->GetDoubleTopic("x").Subscribe(0.0);
                odometry.y = odometry.table->GetDoubleTopic("y").Subscribe(0.0);
                odometry.theta = odometry.table->GetDoubleTopic("theta").Subscribe(0.0);
                odometry.stamp = odometry.table->GetIntegerTopic("stamp").Subscribe(0);
                odometry.publisher = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
                instance.AddListener(odometry.stamp, nt::EventFlags::kValueAll, std::bind(&NFRBridgeNode::recieveOdometry, this, std::placeholders::_1));
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
                cmdVel.table = table->GetSubTable("cmd_vel");
                cmdVel.x = cmdVel.table->GetDoubleTopic("x").Publish();
                cmdVel.y = cmdVel.table->GetDoubleTopic("y").Publish();
                cmdVel.theta = cmdVel.table->GetDoubleTopic("theta").Publish();
                cmdVel.subscription = create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10,
                    std::bind(&NFRBridgeNode::cmdVelCallback, this, std::placeholders::_1));
            }
            // {
            //     pose.table = table->GetSubTable("pose");
            //     pose.x = pose.table->GetDoubleTopic("x").Subscribe(0.0);
            //     pose.y = pose.table->GetDoubleTopic("y").Subscribe(0.0);
            //     pose.theta = pose.table->GetDoubleTopic("theta").Subscribe(0.0);
            //     pose.stamp = pose.table->GetIntegerTopic("stamp").Subscribe(0);
            //     instance.AddListener(pose.stamp, nt::EventFlags::kValueAll, std::bind(&NFRBridgeNode::receivePose, this, std::placeholders::_1));
            // }
            {
                globalSetPose.table = table->GetSubTable("global_set_pose");
                globalSetPose.x = globalSetPose.table->GetDoubleTopic("x").Subscribe(0.0);
                globalSetPose.y = globalSetPose.table->GetDoubleTopic("y").Subscribe(0.0);
                globalSetPose.theta = globalSetPose.table->GetDoubleTopic("theta").Subscribe(0.0);
                globalSetPose.stamp = globalSetPose.table->GetIntegerTopic("stamp").Subscribe(0);
                globalSetPose.publisher = create_client<fuse_msgs::srv::SetPose>("/global_localization_node/set_pose", 10);
                instance.AddListener(globalSetPose.stamp, nt::EventFlags::kValueAll,
                    std::bind(&NFRBridgeNode::recieveGlobalSetPose, this, std::placeholders::_1));
            }
            {
                pose.table = table->GetSubTable("pose");
                pose.x = pose.table->GetDoubleTopic("x").Publish();
                pose.y = pose.table->GetDoubleTopic("y").Publish();
                pose.theta = pose.table->GetDoubleTopic("theta").Publish();
                pose.stamp = pose.table->GetIntegerTopic("stamp").Publish();
                pose.timer = create_wall_timer(20ms, std::bind(&NFRBridgeNode::publishPose, this));
            }
            auto cameraNames = declare_parameter("target_cameras", std::vector<std::string>());
            for (size_t i = 0; i < cameraNames.size(); i++)
            {
                cameras.push_back(TargetCamera());
                cameras[i].table = table->GetSubTable(cameraNames[i]);
                cameras[i].area = cameras[i].table->GetDoubleArrayTopic("area").Publish();
                cameras[i].pitch = cameras[i].table->GetDoubleArrayTopic("pitch").Publish();
                cameras[i].yaw = cameras[i].table->GetDoubleArrayTopic("yaw").Publish();
                cameras[i].tx = cameras[i].table->GetDoubleArrayTopic("tx").Publish();
                cameras[i].ty = cameras[i].table->GetDoubleArrayTopic("ty").Publish();
                cameras[i].depth = cameras[i].table->GetDoubleArrayTopic("depth").Publish();
                cameras[i].distance = cameras[i].table->GetDoubleArrayTopic("distance").Publish();
                cameras[i].fiducialID = cameras[i].table->GetIntegerArrayTopic("fiducial_id").Publish();
                cameras[i].stamp = cameras[i].table->GetIntegerArrayTopic("stamp").Publish();
                std::function<void(const nfr_msgs::msg::TargetList&)> cameraCallback =
                    std::bind(&NFRBridgeNode::sendDetection, this, std::placeholders::_1, i);
                cameras[i].subscription = create_subscription<nfr_msgs::msg::TargetList>(cameraNames[i] + "/targets", 10,
                    cameraCallback);
            }
            auto poseNames = declare_parameter("pose_suppliers", std::vector<std::string>());
            for (size_t i = 0; i < poseNames.size(); i++)
            {
                poseSuppliers.push_back(PoseSupplier());
                poseSuppliers[i].table = table->GetSubTable("poses")->GetSubTable(poseNames[i]);
                poseSuppliers[i].x = poseSuppliers[i].table->GetDoubleTopic("x").Publish();
                poseSuppliers[i].y = poseSuppliers[i].table->GetDoubleTopic("y").Publish();
                poseSuppliers[i].theta = poseSuppliers[i].table->GetDoubleTopic("theta").Publish();
                poseSuppliers[i].stamp = poseSuppliers[i].table->GetIntegerTopic("stamp").Publish();
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
            supplier.x.Set(msg.pose.pose.position.x);
            supplier.y.Set(msg.pose.pose.position.y);
            supplier.theta.Set(getYawFromQuaternion(msg.pose.pose.orientation));
            std::chrono::microseconds offset = (std::chrono::microseconds)instance.GetServerTimeOffset().value_or(0);
            supplier.stamp.Set(((std::chrono::nanoseconds)((std::chrono::nanoseconds)
                ((rclcpp::Time)msg.header.stamp).nanoseconds() + offset)).count());
        }
        void sendDetection(const nfr_msgs::msg::TargetList& msg, int i)
        {
            std::vector<double> area, pitch, yaw, tx, ty, depth, distance;
            std::vector<long> fiducialID, stamp;
            for (size_t j = 0; j < msg.targets.size(); j++)
            {
                area.push_back(msg.targets[j].area);
                pitch.push_back(msg.targets[j].pitch);
                yaw.push_back(msg.targets[j].yaw);
                tx.push_back(msg.targets[j].center.x);
                ty.push_back(msg.targets[j].center.y);
                depth.push_back(msg.targets[j].depth);
                distance.push_back(msg.targets[j].distance);
                fiducialID.push_back(msg.targets[j].fiducial_id);
                std::chrono::microseconds offset = (std::chrono::microseconds)instance.GetServerTimeOffset().value_or(0);
                stamp.push_back(((std::chrono::nanoseconds)((std::chrono::nanoseconds)
                    ((rclcpp::Time)msg.targets[j].header.stamp).nanoseconds() + offset)).count());
            }
            cameras[i].area.Set(area);
            cameras[i].pitch.Set(pitch);
            cameras[i].yaw.Set(yaw);
            cameras[i].tx.Set(tx);
            cameras[i].ty.Set(ty);
            cameras[i].depth.Set(depth);
            cameras[i].distance.Set(distance);
            cameras[i].fiducialID.Set(fiducialID);
            cameras[i].stamp.Set(stamp);
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
            msg.pose.pose.position.x = odometry.x.Get();
            msg.pose.pose.position.y = odometry.y.Get();
            msg.twist.twist.linear.x = odometry.deltaX.Get();
            msg.twist.twist.linear.y = odometry.deltaY.Get();
            msg.twist.twist.angular.z = odometry.deltaTheta.Get();
            msg.twist.covariance = {
                0.81, 0, 0, 0, 0, 0,
                0, 0.81, 0, 0, 0, 0,
                0, 0, 1e6, 0, 0, 0,
                0, 0, 0, 1e6, 0, 0,
                0, 0, 0, 0, 1e6, 0,
                0, 0, 0, 0, 0, 0.81
            };
            odometry.publisher->publish(msg);
            geometry_msgs::msg::TransformStamped transform;
            transform.transform.translation.x = odometry.x.Get();
            transform.transform.translation.y = odometry.y.Get();
            tf2::Quaternion quaternion;
            quaternion.setRPY(0, 0, odometry.theta.Get());
            transform.transform.rotation = tf2::toMsg(quaternion);
            transform.child_frame_id = "base_link";
            transform.header.frame_id = "odom";
            transform.header.stamp = msg.header.stamp;
            broadcaster->sendTransform(transform);
        }
        // void receivePose(const nt::Event& event)
        // {
        //     (void)event;
        //     if (!instance.IsConnected())
        //     {
        //         return;
        //     }
        //     geometry_msgs::msg::TransformStamped transform;
        //     transform.transform.translation.x = pose.x.Get();
        //     transform.transform.translation.y = pose.y.Get();
        //     tf2::Quaternion quaternion;
        //     quaternion.setRPY(0, 0, pose.theta.Get());
        //     transform.transform.rotation = tf2::toMsg(quaternion);
        //     transform.child_frame_id = "base_link";
        //     transform.header.frame_id = "map";
        //     std::chrono::nanoseconds timestamp = (std::chrono::nanoseconds)pose.stamp.Get();
        //     transform.header.stamp = toHostTime(timestamp);
        //     broadcaster->sendTransform(transform);
        // }
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
            pose.pose.covariance = {
                1e-6, 0, 0, 0, 0, 0,
                0, 1e-6, 0, 0, 0, 0,
                0, 0, 1e6, 0, 0, 0,
                0, 0, 0, 1e6, 0, 0,
                0, 0, 0, 0, 1e6, 0,
                0, 0, 0, 0, 0, 1e-6
            };
            tf2::Quaternion quaternion;
            quaternion.setRPY(0, 0, globalSetPose.theta.Get());
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
                pose.x.Set(transform.transform.translation.x);
                pose.y.Set(transform.transform.translation.y);
                pose.theta.Set(getYawFromQuaternion(transform.transform.rotation));
                std::chrono::microseconds offset = (std::chrono::microseconds)instance.GetServerTimeOffset().value_or(0);
                pose.stamp.Set(((std::chrono::nanoseconds)((std::chrono::nanoseconds)
                    ((rclcpp::Time)transform.header.stamp).nanoseconds() + offset)).count());
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