#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <filesystem>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <nlohmann/json.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <opencv2/calib3d.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
Eigen::Vector3d getCentroid(const Eigen::MatrixXd& points)
{
    return points.colwise().mean();
}
void centerPoints(Eigen::MatrixXd& points, const Eigen::Vector3d& centroid)
{
    points.rowwise() -= centroid.transpose();
}
Eigen::Matrix3d computeOptimalRotation(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B)
{
    Eigen::MatrixXd H = B.transpose() * A;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d R = svd.matrixV() * svd.matrixU().transpose();
    return R;
}
Eigen::Vector3d computeTranslation(const Eigen::Vector3d& centroidA, const Eigen::Vector3d& centroidB, const Eigen::Matrix3d& R)
{
    return centroidB - R * centroidA;
}
namespace nfr
{
    class NFRAprilTagLocalizationNode : public rclcpp::Node
    {
    private:
        double tagEdgeSize;
        bool useMultiTagPnP;
        std::string cameraFrame, baseFrame;
        std::map<int, tf2::Transform> tagPoses;
        std::array<cv::Point3d, 4> singleTagCorners;
        std::map<int, std::vector<cv::Point3d>> multiTagCorners;
        std::shared_ptr<message_filters::Subscriber<apriltag_msgs::msg::AprilTagDetectionArray>> detectionSubscription;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>> cameraInfoSubscription;
        std::shared_ptr<message_filters::TimeSynchronizer<apriltag_msgs::msg::AprilTagDetectionArray, sensor_msgs::msg::CameraInfo>> synchronizer;
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr posePublisher;
        std::unique_ptr<tf2_ros::Buffer> buffer;
        std::unique_ptr<tf2_ros::TransformListener> listener;
        tf2::Quaternion toCVRotation, fromCVRotation;
        tf2::Vector3 fromCV(cv::Point3d point)
        {
            return tf2::quatRotate(fromCVRotation, tf2::Vector3(point.x, point.y, point.z));
        }
        cv::Point3d toCV(tf2::Vector3 point)
        {
            auto vector = tf2::quatRotate(toCVRotation, point);
            return cv::Point3d(vector.x(), vector.y(), vector.z());
        }
        tf2::Quaternion fromCV(cv::Mat rot)
        {
            tf2::Matrix3x3 quaternion(
                rot.at<double>(0, 0), rot.at<double>(0, 1), rot.at<double>(0, 2),
                rot.at<double>(1, 0), rot.at<double>(1, 1), rot.at<double>(1, 2),
                rot.at<double>(2, 0), rot.at<double>(2, 1), rot.at<double>(2, 2)
            );
            tf2::Quaternion quat;
            quaternion.getRotation(quat);
            return fromCVRotation.inverse() * (quat * fromCVRotation);
        }
    public:
        NFRAprilTagLocalizationNode(rclcpp::NodeOptions options) : rclcpp::Node("nfr_apriltag_localization_node", options)
        {
            tf2::Matrix3x3 toCVMatrix(0, -1, 0, 0, 0, -1, 1, 0, 0);
            toCVMatrix.getRotation(toCVRotation);
            tf2::Matrix3x3 fromCVMatrix(0, 0, 1, -1, 0, 0, 0, -1, 0);
            fromCVMatrix.getRotation(fromCVRotation);
            tf2::Vector3 vec(1, 2, 3);
            cv::Point3d point = toCV(vec);
            RCLCPP_INFO(get_logger(), "1, 2, 3 to CV is %f, %f, %f", point.x, point.y, point.z); 
            vec = fromCV(point);
            RCLCPP_INFO(get_logger(), "%f, %f, %f", vec.x(), vec.y(), vec.z());
            std::filesystem::path defaultPath = (std::filesystem::path)ament_index_cpp::get_package_share_directory("nfr_charged_up") / "config"
                / "field.json";
            std::string fieldPath = declare_parameter<std::string>("field_path", defaultPath);
            tagEdgeSize = declare_parameter<double>("tag_size", 0.165);
            useMultiTagPnP = declare_parameter<bool>("use_multi_tag_pnp", false);
            cameraFrame = declare_parameter<std::string>("camera_frame", "");
            baseFrame = declare_parameter<std::string>("base_frame", "base_link");
            std::ifstream fieldJson(fieldPath);
            nlohmann::json json = nlohmann::json::parse(fieldJson);
            for (auto tag : json["tags"])
            {
                tagPoses[tag["ID"]].setOrigin({tag["pose"]["translation"]["x"], tag["pose"]["translation"]["y"], tag["pose"]["translation"]["z"]});
                tagPoses[tag["ID"]].setRotation(tf2::Quaternion{tag["pose"]["rotation"]["quaternion"]["X"], tag["pose"]["rotation"]["quaternion"]["Y"],
                    tag["pose"]["rotation"]["quaternion"]["Z"], tag["pose"]["rotation"]["quaternion"]["W"]});
            }
            RCLCPP_INFO(get_logger(), "Loaded all tags from %s", fieldPath.c_str());
            singleTagCorners = {
                cv::Point3d(-tagEdgeSize / 2, tagEdgeSize / 2, 0),
                cv::Point3d(tagEdgeSize / 2, tagEdgeSize / 2, 0),
                cv::Point3d(tagEdgeSize / 2, -tagEdgeSize / 2, 0),
                cv::Point3d(-tagEdgeSize / 2, -tagEdgeSize / 2, 0)
            };
            std::array<tf2::Vector3, 4> otherCorners = {
                tf2::Vector3(0, -tagEdgeSize / 2, -tagEdgeSize / 2),
                tf2::Vector3(0, tagEdgeSize / 2, -tagEdgeSize / 2),
                tf2::Vector3(0, tagEdgeSize / 2, tagEdgeSize / 2),
                tf2::Vector3(0, -tagEdgeSize / 2, tagEdgeSize / 2)
            };
            // std::array<tf2::Vector3, 4> otherCorners = {
            //     fromCV(cv::Point3d(-tagEdgeSize, tagEdgeSize, 0)),
            //     fromCV(cv::Point3d(tagEdgeSize, tagEdgeSize, 0)),
            //     fromCV(cv::Point3d(tagEdgeSize, -tagEdgeSize, 0)),
            //     fromCV(cv::Point3d(-tagEdgeSize, -tagEdgeSize, 0))
            // };
            for (auto tag : json["tags"])
            {
                for (size_t i = 0; i < 4; i++)
                {
                    multiTagCorners[tag["ID"]].push_back(toCV(tagPoses[tag["ID"]](otherCorners[i])));
                    RCLCPP_INFO(get_logger(), "Tag %ld corner %ld: %f, %f, %f", tag["ID"], i, multiTagCorners[tag["ID"]][i].x,
                        multiTagCorners[tag["ID"]][i].y, multiTagCorners[tag["ID"]][i].z);
                }
            }
            detectionSubscription = std::make_shared<message_filters::Subscriber<apriltag_msgs::msg::AprilTagDetectionArray>>(this, "tag_detections");
            cameraInfoSubscription = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>>(this, "camera_info");
            synchronizer = std::make_shared<message_filters::TimeSynchronizer<apriltag_msgs::msg::AprilTagDetectionArray, sensor_msgs::msg::CameraInfo>>
                (*detectionSubscription, *cameraInfoSubscription, 10);
            synchronizer->registerCallback(std::bind(&NFRAprilTagLocalizationNode::onCameraCallback, this, std::placeholders::_1,
                std::placeholders::_2));
            posePublisher = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose_estimations", 10);
            buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
            listener = std::make_unique<tf2_ros::TransformListener>(*buffer);
        }
        void printRotation(tf2::Quaternion rot)
        {
            tf2::Matrix3x3 mat(rot);
            double x, y, z;
            mat.getRPY(x, y, z);
            RCLCPP_INFO(get_logger(), "rpy: %f, %f, %f", x, y, z);
        }
        void printTranslation(tf2::Vector3 vec)
        {
            RCLCPP_INFO(get_logger(), "xyz: %f, %f, %f", vec.x(), vec.y(), vec.z());
        }
        void printTransform(tf2::Transform transform)
        {
            printTranslation(transform.getOrigin());
            printRotation(transform.getRotation());
        }
        tf2::Transform singleTagPnP(apriltag_msgs::msg::AprilTagDetection detection, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cameraInfo)
        {
            cv::Mat rvec(3, 1, CV_64FC1), tvec(3, 1, CV_64FC1);
            std::vector<cv::Point2d> detectionCorners;
            for (size_t i = 0; i < 4; i++)
            {
                detectionCorners.push_back(cv::Point2d(detection.corners[i].x, detection.corners[i].y));
            }
            cv::Mat d(5, 1, CV_64FC1);
            cv::Mat k(3, 3, CV_64FC1);
            d.at<double>(0, 0) = cameraInfo->d[0];
            d.at<double>(1, 0) = cameraInfo->d[1];
            d.at<double>(2, 0) = cameraInfo->d[2];
            d.at<double>(3, 0) = cameraInfo->d[3];
            d.at<double>(4, 0) = cameraInfo->d[4];
            k.at<double>(0, 0) = cameraInfo->k[0];
            k.at<double>(0, 1) = cameraInfo->k[1];
            k.at<double>(0, 2) = cameraInfo->k[2];
            k.at<double>(1, 0) = cameraInfo->k[3];
            k.at<double>(1, 1) = cameraInfo->k[4];
            k.at<double>(1, 2) = cameraInfo->k[5];
            k.at<double>(2, 0) = cameraInfo->k[6];
            k.at<double>(2, 1) = cameraInfo->k[7];
            k.at<double>(2, 2) = cameraInfo->k[8];
            cv::solvePnP(singleTagCorners, detectionCorners, k, d, rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE);
            cv::Mat rot(3, 3, CV_64FC1);
            cv::Rodrigues(rvec, rot);
            auto quaternion = fromCV(rot);
            tf2::Vector3 translation = fromCV(cv::Point3d(tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0)));
            return tf2::Transform(quaternion, translation);
        }
        tf2::Transform multiTagPnP(apriltag_msgs::msg::AprilTagDetectionArray detections, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cameraInfo)
        {
            std::vector<cv::Point3d> cornerTransforms;
            std::vector<cv::Point2d> detectionCorners;
            for (size_t i = 0; i < detections.detections.size(); i++)
            {
                for (size_t j = 0; j < 4; j++)
                {
                    cornerTransforms.push_back(multiTagCorners[detections.detections[i].id][j]);
                    detectionCorners.push_back(cv::Point2d(detections.detections[i].corners[j].x, detections.detections[i].corners[j].y));
                    RCLCPP_INFO(get_logger(), "Tag %ld corner %ld: %f, %f, %f", detections.detections[i].id, j, multiTagCorners[detections.detections[i].id][j].x,
                        multiTagCorners[detections.detections[i].id][j].y, multiTagCorners[detections.detections[i].id][j].z);
                    RCLCPP_INFO(get_logger(), "Detection %ld corner %ld: %f, %f", i, j, detections.detections[i].corners[j].x, detections.detections[i].corners[j].y);
		        }
            }
            cv::Mat d(5, 1, CV_64FC1);
            cv::Mat k(3, 3, CV_64FC1);
            d.at<double>(0, 0) = cameraInfo->d[0];
            d.at<double>(1, 0) = cameraInfo->d[1];
            d.at<double>(2, 0) = cameraInfo->d[2];
            d.at<double>(3, 0) = cameraInfo->d[3];
            d.at<double>(4, 0) = cameraInfo->d[4];
            k.at<double>(0, 0) = cameraInfo->k[0];
            k.at<double>(0, 1) = cameraInfo->k[1];
            k.at<double>(0, 2) = cameraInfo->k[2];
            k.at<double>(1, 0) = cameraInfo->k[3];
            k.at<double>(1, 1) = cameraInfo->k[4];
            k.at<double>(1, 2) = cameraInfo->k[5];
            k.at<double>(2, 0) = cameraInfo->k[6];
            k.at<double>(2, 1) = cameraInfo->k[7];
            k.at<double>(2, 2) = cameraInfo->k[8];
            cv::Mat rvec(3, 1, CV_64FC1), tvec(3, 1, CV_64FC1);
            cv::solvePnP(cornerTransforms, detectionCorners, k, d, rvec, tvec, false, cv::SOLVEPNP_SQPNP);
            cv::Mat rot(3, 3, CV_64FC1);
            cv::Rodrigues(rvec, rot);
            tf2::Quaternion quaternion = fromCV(rot);
            RCLCPP_INFO(get_logger(), "Tvec: %f, %f, %f", tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0), tvec.at<double>(3, 0));
            tf2::Vector3 translation = fromCV(cv::Point3d(tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0)));
            return tf2::Transform(quaternion, translation);
        }
        void onCameraCallback(apriltag_msgs::msg::AprilTagDetectionArray::ConstSharedPtr detections,
            sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfo)
        {
            if (detections->detections.size() == 0)
            {
                return;
            }
            if (!buffer->canTransform(baseFrame, cameraFrame.empty() ? detections->header.frame_id : cameraFrame, tf2::TimePointZero))
            {
                RCLCPP_ERROR(get_logger(), "Could not transform %s to %s", baseFrame.c_str(),
                    (cameraFrame.empty() ? detections->header.frame_id : cameraFrame).c_str());
                return;
            }
            geometry_msgs::msg::TransformStamped robotToCameraMessage = buffer->lookupTransform(baseFrame,
                cameraFrame.empty() ? detections->header.frame_id : cameraFrame, tf2::TimePointZero);
            tf2::Transform robotToCamera;
            tf2::fromMsg(robotToCameraMessage.transform, robotToCamera);
            if (useMultiTagPnP && detections->detections.size() > 1)
            {
                tf2::Transform fieldToCamera = multiTagPnP(*detections, cameraInfo).inverse();
                printTransform(fieldToCamera);
                tf2::Transform robotPose = robotToCamera * fieldToCamera.inverse();
                RCLCPP_INFO(get_logger(), "Calculated pose using MultiTagPnP");
                geometry_msgs::msg::PoseWithCovarianceStamped pose;
                pose.header.frame_id = baseFrame;
                pose.header.stamp = detections->header.stamp;
                pose.pose.pose.position.x = robotPose.getOrigin().getX();
                pose.pose.pose.position.y = robotPose.getOrigin().getY();
                pose.pose.pose.position.z = robotPose.getOrigin().getZ();
                pose.pose.pose.orientation = tf2::toMsg(robotPose.getRotation());
                pose.pose.covariance = {
                    0.2, 0, 0, 0, 0, 0,
                    0, 0.2, 0, 0, 0, 0,
                    0, 0, 1e6, 0, 0, 0,
                    0, 0, 0, 1e6, 0, 0,
                    0, 0, 0, 0, 1e6, 0,
                    0, 0, 0, 0, 0, 0.2
                };
                posePublisher->publish(pose);
            }
            else
            {
                for (auto detection : detections->detections)
                {
                    tf2::Transform cameraToTag = singleTagPnP(detection, cameraInfo);
                    printTransform(cameraToTag);
                    tf2::Transform fieldToTag = tagPoses[detection.id];
                    printTransform(fieldToTag);
                    tf2::Transform robotPose = fieldToTag * cameraToTag.inverse() * robotToCamera.inverse();
                    printTransform(robotPose);
                    RCLCPP_INFO(get_logger(), "Calculated pose using SingleTagPnP");
                    geometry_msgs::msg::PoseWithCovarianceStamped pose;
                    pose.header.frame_id = baseFrame;
                    pose.header.stamp = detections->header.stamp;
                    pose.pose.pose.position.x = robotPose.getOrigin().getX();
                    pose.pose.pose.position.y = robotPose.getOrigin().getY();
                    pose.pose.pose.position.z = robotPose.getOrigin().getZ();
                    pose.pose.pose.orientation = tf2::toMsg(robotPose.getRotation() * tf2::Quaternion(0, 0, 1, 0));
                    pose.pose.covariance = {
                        0.2, 0, 0, 0, 0, 0,
                        0, 0.2, 0, 0, 0, 0,
                        0, 0, 1e6, 0, 0, 0,
                        0, 0, 0, 1e6, 0, 0,
                        0, 0, 0, 0, 1e6, 0,
                        0, 0, 0, 0, 0, 0.2
                    };
                    posePublisher->publish(pose);
                }
            }
        }
    };
}
RCLCPP_COMPONENTS_REGISTER_NODE(nfr::NFRAprilTagLocalizationNode);
