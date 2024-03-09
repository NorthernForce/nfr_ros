#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nfr_msgs/msg/target_list.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <cv_bridge/cv_bridge.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <message_filters/sync_policies/approximate_time.h>
namespace nfr
{
    class NFRTargetFinderNode : public rclcpp::Node
    {
    private:
        bool useDepthSensor;
        using DepthPolicy = message_filters::sync_policies::ApproximateTime<apriltag_msgs::msg::AprilTagDetectionArray, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::CameraInfo,
            sensor_msgs::msg::Image>;
        using DepthSynchronizer = message_filters::Synchronizer<DepthPolicy>;
        using Policy = message_filters::sync_policies::ApproximateTime<apriltag_msgs::msg::AprilTagDetectionArray, sensor_msgs::msg::CameraInfo>;
        using Synchronizer = message_filters::Synchronizer<Policy>;
        bool rectify;
        std::shared_ptr<message_filters::Subscriber<apriltag_msgs::msg::AprilTagDetectionArray>> detectionSubscription;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>> cameraInfoSubscription;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>> depthInfoSubscription;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depthSubscription;
        std::shared_ptr<DepthSynchronizer> depthSynchronizer;
        std::shared_ptr<Synchronizer> synchronizer;
        rclcpp::Publisher<nfr_msgs::msg::TargetList>::SharedPtr targetPublisher;
        std::unique_ptr<tf2_ros::Buffer> buffer;
        std::unique_ptr<tf2_ros::TransformListener> listener;
        std::string cameraFrame, depthFrame;
    public:
        NFRTargetFinderNode(rclcpp::NodeOptions options) : rclcpp::Node("nfr_target_finder_node", options)
        {
            rectify = declare_parameter("rectify", true);
            useDepthSensor = declare_parameter<bool>("use_depth_subscription", true);
            detectionSubscription = std::make_shared<message_filters::Subscriber<apriltag_msgs::msg::AprilTagDetectionArray>>(this, "tag_detections");
            cameraInfoSubscription = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>>(this, "camera_info");
            if (useDepthSensor)
            {
                cameraFrame = declare_parameter("camera_frame", "camera_link");
                depthFrame = declare_parameter("depth_frame", "depth_link");
                depthInfoSubscription = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>>(this, "depth_info");
                depthSubscription = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "depth_raw");
                depthSynchronizer = std::make_shared<DepthSynchronizer>
                    (DepthPolicy(5), *detectionSubscription, *cameraInfoSubscription, *depthInfoSubscription, *depthSubscription);
                depthSynchronizer->registerCallback(std::bind(&NFRTargetFinderNode::onDepthCamera, this, std::placeholders::_1, std::placeholders::_2,
                    std::placeholders::_3, std::placeholders::_4));
            }
            else
            {
                synchronizer = std::make_shared<Synchronizer>
                    (Policy(5), *detectionSubscription, *cameraInfoSubscription);
                synchronizer->registerCallback(std::bind(&NFRTargetFinderNode::onCamera, this, std::placeholders::_1, std::placeholders::_2));
            }
            targetPublisher = create_publisher<nfr_msgs::msg::TargetList>("targets", 10);
            buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
            listener = std::make_unique<tf2_ros::TransformListener>(*buffer);
        }
        void printSize(cv::Size size)
        {
            RCLCPP_INFO(get_logger(), "[%d, %d]", size.width, size.height);
        }
        cv::Point2d transformPoint(cv::Point2d cameraPoint, geometry_msgs::msg::TransformStamped transform, sensor_msgs::msg::CameraInfo cameraInfo, sensor_msgs::msg::CameraInfo depthInfo)
        {
            cv::Mat cameraK;
            if (rectify)
            {
                cameraK = cv::Mat(cameraInfo.p, true).reshape(0, 3)(cv::Rect(0, 0, 3, 3));
            }
            else
            {
                cameraK = cv::Mat(cameraInfo.k, true).reshape(0, 3);
            }
            cv::Mat depthK = cv::Mat(depthInfo.k, true).reshape(0, 3);
            auto quaternion = transform.transform.rotation;
            auto translation = transform.transform.translation;
            cv::Mat T_quaternion_translation = cv::Mat::eye(4, 4, CV_64F);
            T_quaternion_translation.at<double>(0, 0) = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
            T_quaternion_translation.at<double>(0, 1) = 2 * (quaternion.x * quaternion.y - quaternion.w * quaternion.z);
            T_quaternion_translation.at<double>(0, 2) = 2 * (quaternion.x * quaternion.z + quaternion.w * quaternion.y);
            T_quaternion_translation.at<double>(1, 0) = 2 * (quaternion.x * quaternion.y + quaternion.w * quaternion.z);
            T_quaternion_translation.at<double>(1, 1) = 1 - 2 * (quaternion.x * quaternion.x + quaternion.z * quaternion.z);
            T_quaternion_translation.at<double>(1, 2) = 2 * (quaternion.y * quaternion.z - quaternion.w * quaternion.x);
            T_quaternion_translation.at<double>(2, 0) = 2 * (quaternion.x * quaternion.z - quaternion.w * quaternion.y);
            T_quaternion_translation.at<double>(2, 1) = 2 * (quaternion.y * quaternion.z + quaternion.w * quaternion.x);
            T_quaternion_translation.at<double>(2, 2) = 1 - 2 * (quaternion.x * quaternion.x + quaternion.y * quaternion.y);
            T_quaternion_translation.at<double>(0, 3) = translation.x;
            T_quaternion_translation.at<double>(1, 3) = translation.y;
            T_quaternion_translation.at<double>(2, 3) = translation.z;
            cv::Mat_<double> p_cam1 = (cv::Mat_<double>(3, 1) << cameraPoint.x, cameraPoint.y, 1);
            cv::Mat_<double> p_norm_cam1 = cameraK.inv() * p_cam1;
            cv::Mat_<double> p_norm_cam1_homogeneous = (cv::Mat_<double>(4, 1) << p_norm_cam1(0, 0), p_norm_cam1(1, 0), 1, 0);
            cv::Mat_<double> p_norm_cam2_homogeneous = T_quaternion_translation * p_norm_cam1_homogeneous;
            cv::Mat_<double> p_norm_cam2 = (cv::Mat_<double>(3, 1) << p_norm_cam2_homogeneous(0, 0), p_norm_cam2_homogeneous(1, 0), p_norm_cam2_homogeneous(2, 0));
            cv::Point2d pixel_cam2;
            pixel_cam2.x = (p_norm_cam2(0, 0) / p_norm_cam2(2, 0));
            pixel_cam2.y = (p_norm_cam2(1, 0) / p_norm_cam2(2, 0));
            pixel_cam2 = cv::Point2d(depthK.at<double>(0, 0) * pixel_cam2.x + depthK.at<double>(0, 2),
                depthK.at<double>(1, 1) * pixel_cam2.y + depthK.at<double>(1, 2));
            return pixel_cam2;
        }
        void onDepthCamera(apriltag_msgs::msg::AprilTagDetectionArray::ConstSharedPtr detections, sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfo, sensor_msgs::msg::CameraInfo::ConstSharedPtr depthInfo,
            sensor_msgs::msg::Image::ConstSharedPtr depthImage)
        {
            if (!buffer->canTransform(cameraFrame, depthFrame, tf2::TimePointZero))
            {
                RCLCPP_WARN(get_logger(), "Could not transform %s to %s", cameraFrame.c_str(), depthFrame.c_str());
                return;
            }
            auto transform = buffer->lookupTransform(cameraFrame, depthFrame, tf2::TimePointZero);
            nfr_msgs::msg::TargetList targets;
            targets.header = detections->header;
            for (auto detection : detections->detections)
            {
                nfr_msgs::msg::Target target;
                target.area = 0;
                target.center.x = detection.centre.x;
                target.center.y = detection.centre.y;
                auto mat = cv_bridge::toCvCopy(*depthImage, sensor_msgs::image_encodings::TYPE_16UC1);
                auto point = transformPoint(cv::Point2d(target.center.x, target.center.y), transform, *cameraInfo, *depthInfo);
                RCLCPP_INFO(get_logger(), "Calculated point to be %f, %f", point.x, point.y);
                if (point.inside(cv::Rect(0, 0, depthImage->width, depthImage->height)))
                {
                    target.depth = (double)mat->image.at<uint16_t>(point) / 1000;
                }
                target.fiducial_id = detection.id;
                target.yaw = atan((target.center.x - cameraInfo->k[2]) / cameraInfo->k[0]);
                target.pitch = atan((target.center.y - cameraInfo->k[5]) / cameraInfo->k[4]);
                targets.targets.push_back(target);
            }
            targetPublisher->publish(targets);
        }
        void onCamera(apriltag_msgs::msg::AprilTagDetectionArray::ConstSharedPtr detections, sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfo)
        {
            nfr_msgs::msg::TargetList targets;
            targets.header = detections->header;
            for (auto detection : detections->detections)
            {
                nfr_msgs::msg::Target target;
                target.area = 0;
                target.center.x = detection.centre.x;
                target.center.y = detection.centre.y;
                target.depth = 0;
                target.fiducial_id = detection.id;
                target.yaw = atan((target.center.x - cameraInfo->k[2]) / cameraInfo->k[0]);
                target.pitch = atan((target.center.y - cameraInfo->k[5]) / cameraInfo->k[4]);
                targets.targets.push_back(target);
            }
            targetPublisher->publish(targets);
        }
    };
}
RCLCPP_COMPONENTS_REGISTER_NODE(nfr::NFRTargetFinderNode);
