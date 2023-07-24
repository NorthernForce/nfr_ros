#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <map>
#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
using namespace rclcpp;
using namespace std;
using Odometry = nav_msgs::msg::Odometry;
using TransformStamped = geometry_msgs::msg::TransformStamped;
using TransformBroadcaster = tf2_ros::TransformBroadcaster;
using TransformBuffer = tf2_ros::Buffer;
using TransformListener = tf2_ros::TransformListener;
using std::placeholders::_1;
class OdometryNode : public Node
{
private:
    shared_ptr<TransformBroadcaster> broadcaster;
    shared_ptr<TransformListener> listener;
    shared_ptr<TransformBuffer> buffer;
    Subscription<Odometry>::SharedPtr odometrySubscription;
public:
    OdometryNode() : Node("odometry_node")
    {
        initializeTransforms();
        string odometryTopic = declare_parameter("odom_topic", "odom");
        initializeOdometry(odometryTopic);
    }
    void initializeTransforms()
    {
        broadcaster = std::make_shared<TransformBroadcaster>(*this);
        buffer = std::make_shared<TransformBuffer>(get_clock());
        listener = std::make_shared<TransformListener>(*buffer);
    }
    void initializeOdometry(string odometryTopic)
    {
        odometrySubscription = create_subscription<Odometry>(odometryTopic, 10, bind(&OdometryNode::odometryCallback, this, _1));
        RCLCPP_INFO(get_logger(), "Started subscribing to odometry");
    }
    void odometryCallback(const Odometry& odometry)
    {
        TransformStamped transform;
        transform.header = odometry.header;
        transform.child_frame_id = odometry.child_frame_id;
        transform.transform.translation.x = odometry.pose.pose.position.x;
        transform.transform.translation.y = odometry.pose.pose.position.y;
        transform.transform.translation.z = 0;
        transform.transform.rotation = odometry.pose.pose.orientation;
        broadcaster->sendTransform(transform);
    }
};
#include <stdio.h>
#include <execinfo.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
void handler(int sig)
{
    void* array[10];
    size_t size;
    size = backtrace(array, 10);
    backtrace_symbols_fd(array, size, STDERR_FILENO);
    exit(1);
}
int main(int argc, char const *argv[])
{
    signal(SIGSEGV, handler);
    init(argc, argv);
    OdometryNode::SharedPtr node = make_shared<OdometryNode>();
    spin(node);
    shutdown();
    return 0;
}