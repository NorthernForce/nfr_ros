#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <fstream>
#include <filesystem>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_transport/camera_subscriber.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <apriltag.h>
#include <tag36h11.h>
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
        std::mutex mutex;
        std::string baseFrame;
    public:
        NFRAprilTagNode(rclcpp::NodeOptions options) : rclcpp::Node("nfr_apriltag_node", options)
        {
            detector = apriltag_detector_create();
            distanceFactor = declare_parameter("distance_factor", 0.1);
            std::string tagFamily = declare_parameter("family", "36h11");
            double tagEdgeSize = declare_parameter("tag_size", 0.165);
            detector->nthreads = declare_parameter("detector.threads", detector->nthreads);
            detector->quad_decimate = declare_parameter("detector.decimate", detector->quad_decimate);
            detector->quad_sigma = declare_parameter("detector.blur", detector->quad_sigma);
            detector->refine_edges = declare_parameter("detector.refine", detector->refine_edges);
            detector->decode_sharpening = declare_parameter("detector.sharpening", detector->decode_sharpening);
            detector->debug = declare_parameter("detector.debug", detector->debug);
            maxHamming = declare_parameter("max_hamming", 0);
            family = tag36h11_create(); // 36h11 is assumed. Maybe add other tags later???
            apriltag_detector_add_family(detector, family);
            cameraSubscription = image_transport::CameraSubscriber(this, "image", std::bind(&NFRAprilTagNode::onCamera, this, std::placeholders::_1,
                std::placeholders::_2), "raw");
            detectionPublisher = create_publisher<apriltag_msgs::msg::AprilTagDetectionArray>("tag_detections", 10);
        }
        void removeBadDetections(zarray_t* detections)
        {
            for (size_t i = 0; i < zarray_size(detections); i++)
            {
                apriltag_detection* detection;
                zarray_get(detections, i, &detection);
                if (detection->hamming > maxHamming)
                {
                    zarray_remove_index(detections, i, false);
                    i--;
                }
            }
        }
        apriltag_msgs::msg::AprilTagDetection createApriltagDetectionMessage(apriltag_detection* detection)
        {
            apriltag_msgs::msg::AprilTagDetection detectionMessage;
            detectionMessage.family = std::string(detection->family->name);
            detectionMessage.id = detection->id;
            detectionMessage.hamming = detection->hamming;
            detectionMessage.decision_margin = detection->decision_margin;
            detectionMessage.centre.x = detection->c[0];
            detectionMessage.centre.y = detection->c[1];
            std::memcpy(detectionMessage.corners.data(), detection->p, sizeof(double) * 8);
            std::memcpy(detectionMessage.homography.data(), detection->H->data, sizeof(double) * 9);
            return detectionMessage;
        }
        void onCamera(const sensor_msgs::msg::Image::ConstSharedPtr& image, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cameraInfo)
        {
            cv::Mat monoImage = cv_bridge::toCvShare(image, "mono8")->image;
            image_u8_t imageU8{monoImage.cols, monoImage.rows, monoImage.cols, monoImage.data};
            mutex.lock();
            auto detections = apriltag_detector_detect(detector, &imageU8);
            mutex.unlock();
            apriltag_msgs::msg::AprilTagDetectionArray msg;
            msg.header = image->header;
            removeBadDetections(detections);
            for (size_t i = 0; i < zarray_size(detections); i++)
            {
                apriltag_detection* detection;
                zarray_get(detections, i, &detection);
                msg.detections.push_back(createApriltagDetectionMessage(detection));
            }
            detectionPublisher->publish(msg);
            apriltag_detections_destroy(detections);
        }
    };
}
RCLCPP_COMPONENTS_REGISTER_NODE(nfr::NFRAprilTagNode);