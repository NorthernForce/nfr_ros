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
    public:
        NFRDepthFinderNode(rclcpp::NodeOptions options) : rclcpp::Node("nfr_depth_finder_node", options.clock_type(RCL_SYSTEM_TIME))
        {
            targetSubscription = create_subscription<nfr_msgs::msg::TargetList>("targets", 10, std::bind(&NFRDepthFinderNode::depthCallback,
                this, std::placeholders::_1));
            depthSubscription = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "depth_raw");
            depthCache = std::make_shared<message_filters::Cache<sensor_msgs::msg::Image>>(*depthSubscription, 10);
            targetPublisher = create_publisher<nfr_msgs::msg::TargetList>("targets_filtered", 10);
        }
        void depthCallback(const nfr_msgs::msg::TargetList::ConstSharedPtr& targets)
        {
            if (targets->targets.size() == 0)
            {
                return;
            }
            auto depthImage = depthCache->getElemBeforeTime(targets->targets[0].header.stamp);
            if (depthImage == nullptr) return;
            RCLCPP_INFO(get_logger(), "Depth callback: %d, %d", depthImage->width, depthImage->height);
            nfr_msgs::msg::TargetList filteredTargets;
            for (auto target : targets->targets)
            {
                nfr_msgs::msg::Target filteredTarget = target;
                auto mat = cv_bridge::toCvCopy(*depthImage, sensor_msgs::image_encodings::TYPE_16UC1);
                filteredTarget.depth = (double)mat->image.at<uint16_t>(target.center.y, target.center.x) / 1000;
                filteredTargets.targets.push_back(filteredTarget);
            }
            targetPublisher->publish(filteredTargets);
        }
    };
}
RCLCPP_COMPONENTS_REGISTER_NODE(nfr::NFRDepthFinderNode);