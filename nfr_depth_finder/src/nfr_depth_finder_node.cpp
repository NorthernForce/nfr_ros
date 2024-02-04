#include <rclcpp/rclcpp.hpp>
#include <nfr_msgs/msg/target_list.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/cache.h>
#include <sensor_msgs/msg/image.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <cv_bridge/cv_bridge.hpp>
namespace nfr
{
    class NFRDepthFinderNode : public rclcpp::Node
    {
    private:
        rclcpp::Subscription<nfr_msgs::msg::TargetList>::SharedPtr targetSubscription{nullptr};
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depthSubscription{nullptr};
        std::shared_ptr<message_filters::Cache<sensor_msgs::msg::Image>> depthCache;
        rclcpp::Publisher<nfr_msgs::msg::TargetList>::SharedPtr targetPublisher;
        size_t colorWidth, colorHeight;
    public:
        NFRDepthFinderNode(rclcpp::NodeOptions options) : rclcpp::Node("nfr_depth_finder_node", options.clock_type(RCL_SYSTEM_TIME))
        {
            targetSubscription = create_subscription<nfr_msgs::msg::TargetList>("targets", 10, std::bind(&NFRDepthFinderNode::depthCallback,
                this, std::placeholders::_1));
            depthSubscription = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "depth/image_rect_raw");
            depthCache = std::make_shared<message_filters::Cache<sensor_msgs::msg::Image>>(*depthSubscription, 10);
            targetPublisher = create_publisher<nfr_msgs::msg::TargetList>("targets_filtered", 10);
            colorWidth = declare_parameter("color_width", 1920);
            colorHeight = declare_parameter("color_height", 1080);
            RCLCPP_INFO(get_logger(), "Set up nfr_depth_finder_node to receive information from a %dx%d", colorWidth, colorHeight);
        }
        void depthCallback(const nfr_msgs::msg::TargetList::ConstSharedPtr& targets)
        {
            if (targets->targets.size() == 0)
            {
                return;
            }
            auto depthImage = depthCache->getElemBeforeTime(targets->targets[0].header.stamp);
            nfr_msgs::msg::TargetList filteredTargets;
            for (auto target : targets->targets)
            {
                double centerX = (double)target.center.x / colorWidth * depthImage->width;
                double centerY = (double)target.center.y / colorHeight * depthImage->height;
                nfr_msgs::msg::Target filteredTarget = target;
                auto mat = cv_bridge::toCvCopy(*depthImage, sensor_msgs::image_encodings::TYPE_16UC1);
                filteredTarget.depth = (double)mat->image.at<uint16_t>((size_t)centerY, (size_t)centerX) / 1000;
                RCLCPP_INFO(get_logger(), "Depth calculated to be %f at (%f, %f)", filteredTarget.depth, centerX, centerY);
                double centerDepth = (double)mat->image.at<uint16_t>(depthImage->height / 2, depthImage->width / 2) / 1000;
                RCLCPP_INFO(get_logger(), "Center depth calculated to be %f", centerDepth);
                filteredTargets.targets.push_back(filteredTarget);
            }
            targetPublisher->publish(filteredTargets);
        }
    };
}
RCLCPP_COMPONENTS_REGISTER_NODE(nfr::NFRDepthFinderNode);