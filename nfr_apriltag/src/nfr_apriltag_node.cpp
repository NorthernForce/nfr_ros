#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nlohmann/json.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <filesystem>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <nfr_msgs/msg/target_list.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_transport/camera_subscriber.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <apriltag.h>
#include <Eigen/Dense>
#include <tag36h11.h>
#include <opencv2/calib3d.hpp>
#include <math.h>
using std::placeholders::_1;
namespace nfr
{
    class NFRAprilTagNode : public rclcpp::Node
    {
    private:
        apriltag_family_t* family;
        apriltag_detector_t* detector;
        image_transport::CameraSubscriber cameraSubscription;
        rclcpp::Publisher<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detectionPublisher;
        double distanceFactor;
        int maxHamming;
        bool useMultiTagPNP;
        std::mutex mutex;
        std::unique_ptr<tf2_ros::Buffer> buffer;
        std::shared_ptr<tf2_ros::TransformListener> listener;
        std::map<int, tf2::Transform> tagPoses;
        std::string baseFrame;
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher;
        rclcpp::Publisher<nfr_msgs::msg::TargetList>::SharedPtr targetPublisher;
        std::vector<cv::Point3d> corners;
        std::map<int, std::vector<cv::Point3d>> tagCorners;
        tf2::Quaternion toOpenCVRotation, fromOpenCVRotation;
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
            return quat * fromOpenCVRotation;
        }
    public:
        NFRAprilTagNode(const rclcpp::NodeOptions& options) : rclcpp::Node("nfr_apriltag_node", options)
        {
            detector = apriltag_detector_create();
            distanceFactor = declare_parameter("distance_factor", 0.1);
            std::string tagFamily = declare_parameter("family", "36h11");
            double tagEdgeSize = declare_parameter("size", 0.22);
            detector->nthreads = declare_parameter("detector.threads", detector->nthreads);
            detector->quad_decimate = declare_parameter("detector.decimate", detector->quad_decimate);
            detector->quad_sigma = declare_parameter("detector.blur", detector->quad_sigma);
            detector->refine_edges = declare_parameter("detector.refine", detector->refine_edges);
            detector->decode_sharpening = declare_parameter("detector.sharpening", detector->decode_sharpening);
            detector->debug = declare_parameter("detector.debug", detector->debug);
            maxHamming = declare_parameter("max_hamming", 0);
            useMultiTagPNP = declare_parameter("use_multi_tag_pnp", false);
            family = tag36h11_create(); // 36h11 is assumed. Maybe add other tags later???
            apriltag_detector_add_family(detector, family);
            cameraSubscription = image_transport::CameraSubscriber(this, "image", std::bind(&NFRAprilTagNode::onCamera, this, std::placeholders::_1, std::placeholders::_2), "raw");
            publisher = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose_estimations", 10);
            targetPublisher = create_publisher<nfr_msgs::msg::TargetList>("targets", 10);
            buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
            listener = std::make_shared<tf2_ros::TransformListener>(*buffer);
            std::filesystem::path defaultPath = (std::filesystem::path)ament_index_cpp::get_package_share_directory("nfr_charged_up") / "config"
                / "field.json";
            std::string fieldPath = declare_parameter<std::string>("field_path", defaultPath);
            std::ifstream fieldJson(fieldPath);
            nlohmann::json json = nlohmann::json::parse(fieldJson);
            for (auto tag : json["tags"])
            {
                tagPoses[tag["ID"]].setOrigin({tag["pose"]["translation"]["x"], tag["pose"]["translation"]["y"], tag["pose"]["translation"]["z"]});
                tagPoses[tag["ID"]].setRotation({tag["pose"]["rotation"]["quaternion"]["X"], tag["pose"]["rotation"]["quaternion"]["Y"],
                    tag["pose"]["rotation"]["quaternion"]["Z"], tag["pose"]["rotation"]["quaternion"]["W"]});
            }
            baseFrame = declare_parameter<std::string>("base_frame", "base_link");
            RCLCPP_INFO(get_logger(), "Loaded all tags from %s", get_parameter("field_path").as_string().c_str());
            corners = {cv::Point3d(-tagEdgeSize / 2, -tagEdgeSize / 2, 0), cv::Point3d(tagEdgeSize / 2, -tagEdgeSize / 2, 0), cv::Point3d(tagEdgeSize / 2, tagEdgeSize / 2, 0), cv::Point3d(-tagEdgeSize / 2, tagEdgeSize / 2, 0)};
            for (auto tag : json["tags"])
            {
                std::vector<cv::Point3d> points = corners;
                for (size_t i = 0; i < 4; i++)
                {
                    tf2::Vector3 tf2Point = fromCV(points[i]);
                    tf2Point = tf2::quatRotate(tagPoses[tag["ID"]].getRotation(), tf2Point) + tagPoses[tag["ID"]].getOrigin();
                    tagCorners[tag["ID"]].emplace_back(toCV(tf2Point));
                    RCLCPP_INFO(get_logger(), "Corner %f, %f, %f", tf2Point.x(), tf2Point.y(), tf2Point.z());
                }
            }
            tf2::Matrix3x3 mat{0, 0, 1, -1, 0, 0, 0, -1, 0};
            mat.getRotation(fromOpenCVRotation);
            tf2::Matrix3x3 other{0, -1, 0, 0, 0, -1, 1, 0, 0};
            mat.getRotation(toOpenCVRotation);
        }
        void onCamera(const sensor_msgs::msg::Image::ConstSharedPtr& image, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cameraInfo)
        {
            cv::Mat monoImage = cv_bridge::toCvShare(image, "mono8")->image;
            image_u8_t imageU8{monoImage.cols, monoImage.rows, monoImage.cols, monoImage.data};
            mutex.lock();
            auto detections = apriltag_detector_detect(detector, &imageU8);
            mutex.unlock();
            apriltag_msgs::msg::AprilTagDetectionArray msg;
            std::vector<tf2::Transform> cameraToTagEstimates;
            RCLCPP_INFO(get_logger(), "Detected %d tags", zarray_size(detections));
            for (size_t i = 0; i < zarray_size(detections); i++)
            {
                apriltag_detection* detection;
                zarray_get(detections, i, &detection);
                if (detection->hamming > maxHamming || tagPoses.find(detection->id) == tagPoses.end())
                {
                    zarray_remove_index(detections, i, false);
                    i--;
                }
            }
            RCLCPP_INFO(get_logger(), "Detected %d tags", zarray_size(detections));
            nfr_msgs::msg::TargetList targets;
            for (size_t i = 0; i < zarray_size(detections); i++)
            {
                apriltag_detection* detection;
                zarray_get(detections, i, &detection);
                apriltag_msgs::msg::AprilTagDetection detectionMessage;
                detectionMessage.family = std::string(detection->family->name);
                detectionMessage.id = detection->id;
                detectionMessage.hamming = detection->hamming;
                detectionMessage.decision_margin = detection->decision_margin;
                detectionMessage.centre.x = detection->c[0];
                detectionMessage.centre.y = detection->c[1];
                std::memcpy(detectionMessage.corners.data(), detection->p, sizeof(double) * 8);
                std::memcpy(detectionMessage.homography.data(), detection->H->data, sizeof(double) * 9);
                msg.detections.push_back(detectionMessage);
                nfr_msgs::msg::Target target;
                target.header = image->header;
                target.area = 0;
                target.center.x = detectionMessage.centre.x;
                target.center.y = detectionMessage.centre.y;
                target.fiducial_id = detectionMessage.id;
                target.yaw = atan((target.center.x - cameraInfo->k[2]) / cameraInfo->k[0]);
                target.pitch = atan((target.center.y - cameraInfo->k[5]) / cameraInfo->k[4]);
                targets.targets.push_back(target);
                if (zarray_size(detections) == 1 || !useMultiTagPNP)
                {
                    cv::Mat rvec(3, 1, CV_64FC1), tvec(3, 1, CV_64FC1);
                    std::vector<cv::Point2d> detectionCorners;
                    for (size_t i = 0; i < 4; i++)
                    {
                        detectionCorners.push_back(cv::Point2d(detectionMessage.corners[i].x, detectionMessage.corners[i].y));
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
                    cv::solvePnP(corners, detectionCorners, k, d, rvec, tvec);
                    cv::Mat rot(3, 3, CV_64FC1);
                    cv::Rodrigues(rvec, rot);
                    auto quaternion = fromCV(rot);
                    tf2::Matrix3x3 quat(quaternion);
                    tf2::Vector3 translation = fromCV(cv::Point3d(tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0)));
                    cameraToTagEstimates.push_back(tf2::Transform(quaternion, translation));
                    double roll, pitch, yaw;
                    quat.getRPY(roll, pitch, yaw);
                    RCLCPP_INFO(get_logger(), "%d: [%f, %f, %f]", detectionMessage.id, translation.x(), translation.y(), translation.z());
                    RCLCPP_INFO(get_logger(), "%f, %f", detectionMessage.decision_margin, yaw / M_PI * 180);
                }
            }
            if (!buffer->canTransform(baseFrame, image->header.frame_id, tf2::TimePointZero))
            {
                RCLCPP_ERROR(get_logger(), "Could not transform %s to %s.", baseFrame.c_str(), image->header.frame_id.c_str());
                return;
            }
            auto baseToCamera = buffer->lookupTransform(baseFrame, image->header.frame_id, tf2::TimePointZero);
            tf2::Transform baseToCameraTransform;
            tf2::fromMsg(baseToCamera.transform, baseToCameraTransform);
            if (zarray_size(detections) > 1 && useMultiTagPNP)
            {
                std::vector<cv::Point3d> cornerTransforms;
                std::vector<cv::Point2d> detectionCorners;
                for (size_t i = 0; i < zarray_size(detections); i++)
                {
                    for (size_t j = 0; j < 4; j++)
                    {
                        cornerTransforms.push_back(tagCorners[msg.detections[i].id][j]);
                        detectionCorners.push_back(cv::Point2d(msg.detections[i].corners[j].x, msg.detections[i].corners[j].y));
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
                k.at<double>(1, 0) = cameraInfo->k[1];
                k.at<double>(2, 0) = cameraInfo->k[2];
                k.at<double>(0, 1) = cameraInfo->k[3];
                k.at<double>(1, 1) = cameraInfo->k[4];
                k.at<double>(2, 1) = cameraInfo->k[5];
                k.at<double>(0, 2) = cameraInfo->k[6];
                k.at<double>(1, 2) = cameraInfo->k[7];
                k.at<double>(2, 2) = cameraInfo->k[8];
                cv::Mat rvec(3, 1, CV_64FC1), tvec(3, 1, CV_64FC1);
                cv::solvePnP(cornerTransforms, detectionCorners, k, d, rvec, tvec, false, cv::SOLVEPNP_SQPNP);
                cv::Mat rot(3, 3, CV_64FC1);
                cv::Rodrigues(rvec, rot);
                tf2::Quaternion quaternion = fromCV(rot);
                tf2::Vector3 translation = fromCV(cv::Point3d(tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0)));
                tf2::Transform poseEstimate = tf2::Transform(quaternion, translation).inverse() * baseToCameraTransform.inverse();
                geometry_msgs::msg::PoseWithCovarianceStamped pose;
                pose.header.stamp = image->header.stamp;
                pose.header.frame_id = "map";
                pose.pose.pose.position.x = poseEstimate.getOrigin().getX();
                pose.pose.pose.position.y = poseEstimate.getOrigin().getY();
                pose.pose.pose.position.z = poseEstimate.getOrigin().getZ();
                double distance = std::sqrt(std::pow(poseEstimate.getOrigin().getX(), 2) + std::pow(poseEstimate.getOrigin().getY(), 2));
                pose.pose.covariance = {
                    distanceFactor * distance, 0, 0, 0, 0, 0,
                    0, distanceFactor * distance, 0, 0, 0, 0,
                    0, 0, 1e3, 0, 0, 0,
                    0, 0, 0, 1e3, 0, 0,
                    0, 0, 0, 0, 1e3, 0,
                    0, 0, 0, 0, 0, distanceFactor * distance
                };
                pose.pose.pose.orientation = tf2::toMsg(poseEstimate.getRotation());
                RCLCPP_INFO(get_logger(), "Estimated pose to be [%f, %f, %f]", poseEstimate.getOrigin().getX(), poseEstimate.getOrigin().getY(),
                    poseEstimate.getOrigin().getZ());
                publisher->publish(pose);
            }
            else
            {
                for (size_t i = 0; i < zarray_size(detections); i++)
                {
                    tf2::Transform fieldToTag = tagPoses[msg.detections[i].id];
                    tf2::Transform poseEstimate = fieldToTag * cameraToTagEstimates[i] * baseToCameraTransform.inverse();
                    geometry_msgs::msg::PoseWithCovarianceStamped pose;
                    pose.header.stamp = image->header.stamp;
                    pose.header.frame_id = "map";
                    pose.pose.pose.position.x = poseEstimate.getOrigin().getX();
                    pose.pose.pose.position.y = poseEstimate.getOrigin().getY();
                    pose.pose.pose.position.z = poseEstimate.getOrigin().getZ();
                    pose.pose.pose.orientation = tf2::toMsg(poseEstimate.getRotation());
                    RCLCPP_INFO(get_logger(), "Estimated pose to be [%f, %f, %f]", poseEstimate.getOrigin().getX(), poseEstimate.getOrigin().getY(),
                        poseEstimate.getOrigin().getZ());
                    publisher->publish(pose);
                }
            }
            targetPublisher->publish(targets);
            apriltag_detections_destroy(detections);
        }
    };
}
RCLCPP_COMPONENTS_REGISTER_NODE(nfr::NFRAprilTagNode);