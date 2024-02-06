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
namespace nfr
{
    class NFRAprilTagLocalizationNode : public rclcpp::Node
    {
    private:
        double tagEdgeSize;
        bool useMultiTagPnP;
        std::string cameraFrame, baseFrame;
        std::map<int, tf2::Transform> tagPoses;
        std::vector<cv::Point3d> singleTagCorners;
        std::map<int, std::vector<cv::Point3d>> multiTagCorners;
        std::shared_ptr<message_filters::Subscriber<apriltag_msgs::msg::AprilTagDetectionArray>> detectionSubscription;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>> cameraInfoSubscription;
        std::shared_ptr<message_filters::TimeSynchronizer<apriltag_msgs::msg::AprilTagDetectionArray, sensor_msgs::msg::CameraInfo>> synchronizer;
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr posePublisher;
        std::unique_ptr<tf2_ros::Buffer> buffer;
        std::unique_ptr<tf2_ros::TransformListener> listener;
        tf2::Vector3 fromCV(cv::Point3d point)
        {
            return tf2::Vector3(point.z, point.x, point.y);
        }
        cv::Point3d toCV(tf2::Vector3 point)
        {
            return cv::Point3d(point.y(), point.z(), point.x());
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
            return tf2::Quaternion(quat.z(), quat.x(), quat.y(), quat.w());
        }
    public:
        NFRAprilTagLocalizationNode(rclcpp::NodeOptions options) : rclcpp::Node("nfr_apriltag_localization_node", options)
        {
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
                tagPoses[tag["ID"]].setRotation({tag["pose"]["rotation"]["quaternion"]["X"], tag["pose"]["rotation"]["quaternion"]["Y"],
                    tag["pose"]["rotation"]["quaternion"]["Z"], tag["pose"]["rotation"]["quaternion"]["W"]});
            }
            RCLCPP_INFO(get_logger(), "Loaded all tags from %s", fieldPath.c_str());
            singleTagCorners = {
                cv::Point3d(-tagEdgeSize / 2, tagEdgeSize / 2, 0),
                cv::Point3d(tagEdgeSize / 2, tagEdgeSize / 2, 0),
                cv::Point3d(tagEdgeSize / 2, -tagEdgeSize / 2, 0),
                cv::Point3d(-tagEdgeSize / 2, -tagEdgeSize / 2, 0)
            };
            for (auto tag : json["tags"])
            {
                for (size_t i = 0; i < 4; i++)
                {
                    tf2::Vector3 tf2Point = fromCV(singleTagCorners[i]);
                    tf2Point = tf2::quatRotate(tagPoses[tag["ID"]].getRotation(), tf2Point) + tagPoses[tag["ID"]].getOrigin();
                    multiTagCorners[tag["ID"]].emplace_back(toCV(tf2Point));
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
            tf2::Vector3 translation = fromCV(cv::Point3d(tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0)));
            return tf2::Transform(quaternion, translation).inverse();
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
            if (useMultiTagPnP)
            {
                tf2::Transform fieldToCamera = multiTagPnP(*detections, cameraInfo);
                tf2::Vector3 robotTranslation = fieldToCamera.getOrigin() - robotToCamera.getOrigin();
                tf2::Quaternion robotRotation = fieldToCamera.getRotation() * robotToCamera.getRotation().inverse();
                geometry_msgs::msg::PoseWithCovarianceStamped pose;
                pose.header.frame_id = baseFrame;
                pose.header.stamp = detections->header.stamp;
                pose.pose.pose.position.x = robotTranslation.getX();
                pose.pose.pose.position.y = robotTranslation.getY();
                pose.pose.pose.position.z = robotTranslation.getZ();
                pose.pose.pose.orientation = tf2::toMsg(robotRotation);
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
                    tf2::Transform fieldToTag = tagPoses[detection.id];
                    tf2::Vector3 fieldToCameraTranslation = fieldToTag.getOrigin() +
                        tf2::quatRotate(fieldToTag.getRotation(), tf2::quatRotate(cameraToTag.getRotation(), cameraToTag.getOrigin()));
                    tf2::Quaternion fieldToCameraRotation = fieldToTag.getRotation() * cameraToTag.getRotation().inverse();
                    tf2::Vector3 robotTranslation = fieldToCameraTranslation - robotToCamera.getOrigin();
                    tf2::Quaternion robotRotation = fieldToCameraRotation * robotToCamera.getRotation().inverse();
                    geometry_msgs::msg::PoseWithCovarianceStamped pose;
                    pose.header.frame_id = baseFrame;
                    pose.header.stamp = detections->header.stamp;
                    pose.pose.pose.position.x = robotTranslation.getX();
                    pose.pose.pose.position.y = robotTranslation.getY();
                    pose.pose.pose.position.z = robotTranslation.getZ();
                    pose.pose.pose.orientation = tf2::toMsg(robotRotation);
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