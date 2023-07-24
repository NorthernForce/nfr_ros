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
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <map>
#include <nlohmann/json.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <fstream>
#include <nav2_msgs/srv/load_map.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/twist.hpp>
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
using AprilTagDetectionArray = apriltag_msgs::msg::AprilTagDetectionArray;
using TransformBuffer = tf2_ros::Buffer;
using TransformListener = tf2_ros::TransformListener;
using TransformException = tf2::TransformException;
using Transform = geometry_msgs::msg::Transform;
using LoadMap = nav2_msgs::srv::LoadMap;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using Twist = geometry_msgs::msg::Twist;
template<typename T>
using ActionClient = rclcpp_action::Client<T>;
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
class SquishyNode : public Node
{
private:
    shared_ptr<TransformBroadcaster> broadcaster;
    shared_ptr<TransformListener> listener;
    shared_ptr<TransformBuffer> buffer;

    NetworkTableInstance instance;
    TimerBase::SharedPtr networkTablesTimer;
    shared_ptr<NetworkTable> rosTable;
    NetworkTableEntry rosStatus;

    TimerBase::SharedPtr odometryTimer;
    Publisher<Odometry>::SharedPtr odometryPublisher;
    shared_ptr<NetworkTable> odometryTable;
    NetworkTableEntry odomX, odomY, odomTheta, odomDeltaX, odomDeltaY, odomDeltaTheta, odomTimestamp, odomStatus;

    TimerBase::SharedPtr setPoseTimer;
    Publisher<PoseWithCovarianceStamped>::SharedPtr setPosePublisher;
    shared_ptr<NetworkTable> setPoseTable;
    NetworkTableEntry setPoseX, setPoseY, setPoseTheta, setPoseDirty, setPoseStatus;

    map<int, Transform> tagPoses;

    Publisher<PoseWithCovarianceStamped>::SharedPtr realsensePublisher;
    Subscription<AprilTagDetectionArray>::SharedPtr realsenseSubscription;
    shared_ptr<NetworkTable> realsenseTable;
    NetworkTableEntry realsenseStatus;

    TimerBase::SharedPtr poseEstimateTimer;
    shared_ptr<NetworkTable> poseEstimateTable;
    NetworkTableEntry poseEstimateX, poseEstimateY, poseEstimateTheta, poseEstimateStatus;

    Client<LoadMap>::SharedPtr loadMapClient;
    shared_ptr<NetworkTable> mapTable;
    NetworkTableEntry mapStatus;

    TimerBase::SharedPtr goToPoseTimer;
    ActionClient<NavigateToPose>::SharedPtr goToPoseClient;
    shared_ptr<NetworkTable> goToPoseTable;
    NetworkTableEntry goToPoseX, goToPoseY, goToPoseTheta, goToPoseDirty, goToPoseStatus;

    Subscription<Twist>::SharedPtr speedSubscriber;
    shared_ptr<NetworkTable> speedTable;
    NetworkTableEntry speedX, speedY, speedTheta, speedStatus;
public:
    SquishyNode() : Node("squishy_node")
    {
        // initializeTransforms();
        // int teamNumber = declare_parameter("team_number", 172);
        // string serverName = declare_parameter("server_name", "");
        // string clientName = declare_parameter("nt_name", "xavier");
        // string rosTablePath = declare_parameter("ros_table_path", "/ros");
        // initializeNetworkTables(teamNumber, clientName, rosTablePath);
        // string odomTablePath = declare_parameter("odom_table_path", "odom");
        // initializeOdometry(odomTablePath);
        // string setPoseTablePath = declare_parameter("set_pose_table_path", "set_pose");
        // initializeSetPose(setPoseTablePath);
        // const auto& defaultFieldPath = filesystem::path(get_package_share_directory("nfr_charged_up"))
        //     / "config" / "field.json";
        // string fieldPath = declare_parameter("field_path", defaultFieldPath);
        // initializeFieldFromJson(fieldPath);
        // string realsenseTablePath = declare_parameter("realsense_table_path", "realsense");
        // initializeRealsense(realsenseTablePath);
        // string poseEstimateTablePath = declare_parameter("pose_estimate_table_path", "pose_estimate");
        // initializePoseEstimatePublisher(poseEstimateTablePath);
        // string mapPath = declare_parameter("map_path", filesystem::path(get_package_share_directory("nfr_charged_up"))
        //     / "config" / "map.yaml");
        // string mapTablePath = declare_parameter("map_table_path", "map");
        // initializeMap(mapPath, mapTablePath);
        // string goToPoseTablePath = declare_parameter("go_to_pose_table_path", "go_to_pose");
        // initializeGoToPose(goToPoseTablePath);
        // string speedTable = declare_parameter("speed_table", "speed");
        // initializeSpeedPublisher(speedTable);
    }
    // void initializeNetworkTables(int team, string name, string rosTablePath)
    // {
    //     instance = NetworkTableInstance::Create();
    //     instance.SetServerTeam(team);
    //     instance.StartClient4(name);
    //     RCLCPP_INFO(get_logger(), "Starting NetworkTable Client4 to connect to team %d as '%s'", team, name);
    //     networkTablesTimer = create_wall_timer(2.5s, [&]()
    //     {
    //         if (!instance.IsConnected())
    //         {
    //             RCLCPP_WARN(get_logger(), "Not connected to NetworkTables");
    //         }
    //     });
    //     RCLCPP_INFO(get_logger(), "Started to check for NetworkTable connectivity every 2.5s");
    //     rosTable = instance.GetTable(rosTablePath);
    //     rosStatus = rosTable->GetEntry("status");
    //     rosStatus.SetString("online");
    //     RCLCPP_INFO(get_logger(), "Set ROS status to online");
    // }
    // void initializeTransforms()
    // {
    //     broadcaster = std::make_shared<TransformBroadcaster>(*this);
    //     buffer = std::make_shared<TransformBuffer>(get_clock());
    //     listener = std::make_shared<TransformListener>(*buffer);
    // }
    // void initializeOdometry(string odometryTablePath)
    // {
    //     odometryTable = rosTable->GetSubTable(odometryTablePath);
    //     odomX = odometryTable->GetEntry("x");
    //     odomY = odometryTable->GetEntry("y");
    //     odomTheta = odometryTable->GetEntry("theta");
    //     odomDeltaX = odometryTable->GetEntry("vx");
    //     odomDeltaY = odometryTable->GetEntry("vy");
    //     odomDeltaTheta = odometryTable->GetEntry("vtheta");
    //     odomTimestamp = odometryTable->GetEntry("timestamp");
    //     odomStatus = odometryTable->GetEntry("status");
    //     odometryPublisher = create_publisher<Odometry>("odom", 10);
    //     odometryTimer = create_wall_timer(40ms, bind(&SquishyNode::odometryCallback, this, _1));
    //     RCLCPP_INFO(get_logger(), "Started odometry timer at 40ms interval");
    //     odomStatus = odometryTable->GetEntry("status");
    //     odomStatus.SetString("online");
    //     RCLCPP_INFO(get_logger(), "Set odometry status to online");
    // }
    // void odometryCallback()
    // {
    //     double x = odomX.GetDouble(0);
    //     double y = odomY.GetDouble(0);
    //     double theta = odomTheta.GetDouble(0);
    //     double vx = odomDeltaX.GetDouble(0);
    //     double vy = odomDeltaY.GetDouble(0);
    //     double vtheta = odomDeltaTheta.GetDouble(0);
    //     Odometry odometry;
    //     odometry.child_frame_id = "base_link";
    //     odometry.header.frame_id = "odom";
    //     Time time = Time(odomTimestamp.GetInteger(get_clock()->now().nanoseconds()));
    //     odometry.header.stamp = time;
    //     odometry.pose.pose.position.x = x;
    //     odometry.pose.pose.position.y = y;
    //     odometry.pose.pose.position.z = 0;
    //     Quaternion quaternion;
    //     quaternion.setRPY(0, 0, theta);
    //     odometry.pose.pose.orientation = tf2::toMsg(quaternion);
    //     odometry.twist.twist.linear.x = vx;
    //     odometry.twist.twist.linear.y = vy;
    //     odometry.twist.twist.linear.z = 0;
    //     odometry.twist.twist.angular.x = 0;
    //     odometry.twist.twist.angular.y = 0;
    //     odometry.twist.twist.angular.z = vtheta;
    //     odometryPublisher->publish(odometry);
    //     TransformStamped transform;
    //     transform.header = odometry.header;
    //     transform.child_frame_id = odometry.child_frame_id;
    //     transform.transform.translation.x = x;
    //     transform.transform.translation.y = y;
    //     transform.transform.translation.z = 0;
    //     transform.transform.rotation = odometry.pose.pose.orientation;
    //     broadcaster->sendTransform(transform);
    // }
    // void initializeSetPose(string setPoseTablePath)
    // {
    //     setPoseTable = rosTable->GetSubTable(setPoseTablePath);
    //     setPoseX = setPoseTable->GetEntry("x");
    //     setPoseY = setPoseTable->GetEntry("y");
    //     setPoseTheta = setPoseTable->GetEntry("theta");
    //     setPoseDirty = setPoseTable->GetEntry("dirty");
    //     setPosePublisher = create_publisher<PoseWithCovarianceStamped>("set_pose", 10);
    //     setPoseTimer = create_wall_timer(50ms, bind(&SquishyNode::setPoseCallback, this, _1));
    //     RCLCPP_INFO(get_logger(), "Started set pose timer at 50ms interval");
    //     setPoseStatus = setPoseTable->GetEntry("status");
    //     setPoseStatus.SetString("online");
    //     RCLCPP_INFO(get_logger(), "Set set pose status to online");
    // }
    // void setPoseCallback()
    // {
    //     if (setPoseDirty.GetBoolean(false))
    //     {
    //         setPoseDirty.SetBoolean(false);
    //         PoseWithCovarianceStamped pose;
    //         pose.header.stamp = get_clock()->now();
    //         pose.pose.pose.position.x = setPoseX.GetDouble(0);
    //         pose.pose.pose.position.y = setPoseY.GetDouble(0);
    //         pose.pose.pose.position.z = 0;
    //         Quaternion quaternion;
    //         quaternion.setRPY(0, 0, setPoseTheta.GetDouble(0));
    //         pose.pose.pose.orientation = tf2::toMsg(quaternion);
    //         setPosePublisher->publish(pose);
    //     }
    // }
    // void initializeFieldFromJson(string fieldPath)
    // {
    //     ifstream fieldFile(fieldPath);
    //     const auto& fieldRead = nlohmann::json::parse(fieldFile);
    //     for (const auto& tag : fieldRead["tags"])
    //     {
    //         Transform transform;
    //         transform.translation.x = tag["pose"]["translation"]["x"];
    //         transform.translation.y = tag["pose"]["translation"]["y"];
    //         transform.translation.z = tag["pose"]["translation"]["z"];
    //         transform.rotation.w = tag["pose"]["rotation"]["quaternion"]["W"];
    //         transform.rotation.x = tag["pose"]["rotation"]["quaternion"]["X"];
    //         transform.rotation.y = tag["pose"]["rotation"]["quaternion"]["Y"];
    //         transform.rotation.z = tag["pose"]["rotation"]["quaternion"]["Z"];
    //         tagPoses[tag["ID"]] = transform;
    //         RCLCPP_INFO(get_logger(), "Loaded tag %d at [%f, %f, %f]", tag["ID"], transform.translation.x,
    //             transform.translation.y, transform.translation.z);
    //     }
    // }
    // void initializeRealsense(string realsenseTablePath)
    // {
    //     realsenseTable = rosTable->GetSubTable(realsenseTablePath);
    //     realsensePublisher = create_publisher<PoseWithCovarianceStamped>("realsense_pose", 10);
    //     realsenseSubscription = create_subscription<AprilTagDetectionArray>("realsense/detections", 10,
    //         bind(&SquishyNode::realsenseCallback, this, _1));
    //     RCLCPP_INFO(get_logger(), "Subscribed to realsense apriltag detections");
    //     realsenseStatus = realsenseTable->GetEntry("status");
    //     realsenseStatus.SetString("online");
    //     RCLCPP_INFO(get_logger(), "Set realsense status to online");
    // }
    // void realsenseCallback(const AprilTagDetectionArray& array)
    // {
    //     const auto& timestamp = array.header.stamp;
    //     for (const auto& detection : array.detections)
    //     {
    //         if (tagPoses.find(detection.id) != tagPoses.end())
    //         {
    //             string tagFrame = "tag16h5:" + to_string(detection.id);
    //             string cameraFrame = "camera_link";
    //             TransformStamped cameraToTag;
    //             try
    //             {
    //                 cameraToTag = buffer->lookupTransform(tagFrame, cameraFrame, tf2::TimePointZero);
    //             }
    //             catch (const TransformException& e)
    //             {
    //                 RCLCPP_ERROR(get_logger(), "Caught transform exception when trying to lookup %s to %s: %s",
    //                     cameraFrame.c_str(), tagFrame.c_str(), e.what());
    //                 break;
    //             }
    //             string baseFrame = "base_link";
    //             cameraFrame = "camera_link";
    //             TransformStamped baseToCamera;
    //             try
    //             {
    //                 baseToCamera = buffer->lookupTransform(cameraFrame, baseFrame, tf2::TimePointZero);
    //             }
    //             catch (const TransformException& e)
    //             {
    //                 RCLCPP_ERROR(get_logger(), "Caught transform exception when trying to lookup %s to %s: %s",
    //                     baseFrame.c_str(), cameraFrame.c_str(), e.what());
    //                 break;
    //             }
    //             Transform worldToTag = tagPoses[detection.id];
    //             Transform poseTransform = worldToTag - (cameraToTag.transform + baseToCamera.transform);
    //             PoseWithCovarianceStamped pose;
    //             pose.header.frame_id = "base_link";
    //             pose.header.stamp = timestamp;
    //             pose.pose.pose.position.x = poseTransform.translation.x;
    //             pose.pose.pose.position.y = poseTransform.translation.y;
    //             pose.pose.pose.position.z = poseTransform.translation.z;
    //             pose.pose.pose.orientation = poseTransform.rotation;
    //             realsensePublisher->publish(pose);
    //         }
    //     }
    // }
    // void initializePoseEstimatePublisher(string poseEstimateTablePath)
    // {
    //     poseEstimateTable = rosTable->GetSubTable(poseEstimateTablePath);
    //     poseEstimateX = poseEstimateTable->GetEntry("x");
    //     poseEstimateY = poseEstimateTable->GetEntry("y");
    //     poseEstimateTheta = poseEstimateTable->GetEntry("theta");
    //     poseEstimateTimer = create_wall_timer(50ms, bind(&SquishyNode::poseEstimateCallback, this, _1));
    //     RCLCPP_INFO(get_logger(), "Publishing pose estimates every 50ms");
    //     poseEstimateStatus = poseEstimateTable->GetEntry("status");
    //     poseEstimateStatus.SetString("online");
    //     RCLCPP_INFO(get_logger(), "Set pose estimate status to online");
    // }
    // void poseEstimateCallback()
    // {
    //     string baseFrame = "base_link";
    //     string mapFrame = "map";
    //     TransformStamped mapToBase;
    //     try
    //     {
    //         mapToBase = buffer->lookupTransform(baseFrame, mapFrame, tf2::TimePointZero);
    //     }
    //     catch (const TransformException& e)
    //     {
    //         RCLCPP_ERROR(get_logger(), "Caught transform exception when trying to lookup %s to %s: %s",
    //             mapFrame.c_str(), baseFrame.c_str(), e.what());
    //         return;
    //     }
    //     poseEstimateX.SetDouble(mapToBase.transform.translation.x);
    //     poseEstimateY.SetDouble(mapToBase.transform.translation.y);
    //     Quaternion q;
    //     tf2::fromMsg(mapToBase.transform.rotation, q);
    //     double yaw = atan2(2.0 * (q.y() * q.z() + q.w() * q.x()), q.w() * q.w() - q.x() * q.x() - q.y() *q.y() + q.z() * q.z());
    //     poseEstimateTheta.SetDouble(yaw);
    // }
    // void initializeMap(string mapPath, string mapTablePath)
    // {
    //     loadMapClient = create_client<LoadMap>("map_server/load_map");
    //     LoadMap::Request::SharedPtr loadMapRequest = std::make_shared<LoadMap::Request>();
    //     const auto& future = loadMapClient->async_send_request(loadMapRequest);
    //     mapTable = rosTable->GetSubTable(mapTablePath);
    //     mapStatus = mapTable->GetEntry("status");
    //     if (spin_until_future_complete(get_node_base_interface(), future) == FutureReturnCode::SUCCESS)
    //     {
    //         RCLCPP_INFO(get_logger(), "Successfully loaded map from '%s'", get_parameter("map_path").as_string().c_str());
    //         mapStatus.SetString("online");
    //     }
    //     else
    //     {
    //         RCLCPP_ERROR(get_logger(), "Failed to load map from '%s'", get_parameter("map_path").as_string().c_str());
    //         mapStatus.SetString("offline");
    //     }
    // }
    // void initializeGoToPose(string goToPoseTablePath)
    // {
    //     goToPoseTable = rosTable->GetSubTable(goToPoseTablePath);
    //     goToPoseX = goToPoseTable->GetEntry("x");
    //     goToPoseY = goToPoseTable->GetEntry("y");
    //     goToPoseTheta = goToPoseTable->GetEntry("theta");
    //     goToPoseClient = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    //     goToPoseDirty = goToPoseTable->GetEntry("dirty");
    //     goToPoseTimer = create_wall_timer(25ms, bind(&SquishyNode::goToPoseCallback, this, _1));
    //     RCLCPP_INFO(get_logger(), "Started checking for a target pose every 25ms");
    //     goToPoseStatus = goToPoseTable->GetEntry("status");
    //     goToPoseStatus.SetString("online");
    //     RCLCPP_INFO(get_logger(), "Set go to pose status to online");
    // }
    // void goToPoseCallback()
    // {
    //     if (goToPoseDirty.GetBoolean(false))
    //     {
    //         goToPoseDirty.SetBoolean(false);
    //         NavigateToPose::Goal goal;
    //         goal.behavior_tree = "";
    //         goal.pose.header.frame_id = "map";
    //         goal.pose.header.stamp = get_clock()->now();
    //         goal.pose.pose.position.x = goToPoseX.GetDouble(0);
    //         goal.pose.pose.position.y = goToPoseY.GetDouble(0);
    //         goal.pose.pose.position.z = 0;
    //         Quaternion quaternion;
    //         quaternion.setRPY(0, 0, goToPoseTheta.GetDouble(0));
    //         goal.pose.pose.orientation = tf2::toMsg(quaternion);
    //     }
    // }
    // void initializeSpeedPublisher(string speedTablePath)
    // {
    //     speedTable = rosTable->GetSubTable(speedTablePath);
    //     speedX = speedTable->GetEntry("vx");
    //     speedY = speedTable->GetEntry("vy");
    //     speedTheta = speedTable->GetEntry("vtheta");
    //     speedSubscriber = create_subscription<Twist>("cmd_vel", 10, bind(&SquishyNode::speedCallback, this, _1));
    //     RCLCPP_INFO(get_logger(), "Subscribed to cmd_vel for a twist");
    //     speedStatus = speedTable->GetEntry("status");
    //     speedStatus.SetString("online");
    //     RCLCPP_INFO(get_logger(), "Set speed publisher status to online");
    // }
    // void speedCallback(const Twist& twist)
    // {
    //     speedX.SetDouble(twist.linear.x);
    //     speedY.SetDouble(twist.linear.y);
    //     speedTheta.SetDouble(twist.angular.z);
    // }
};
int main(int argc, char const *argv[])
{
    init(argc, argv);
    SquishyNode::SharedPtr node = make_shared<SquishyNode>();
    spin(node);
    shutdown();
    return 0;
}