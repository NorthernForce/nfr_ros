#include <rclcpp/rclcpp.hpp>
#include <nfr_msgs/msg/target_list.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/msg/image.hpp>
#include <rclcpp_components/register_node_macro.hpp>
namespace nfr
{
    class NFRDepthFinderNode : public rclcpp::Node
    {
    private:
        std::shared_ptr<message_filters::Subscriber<nfr_msgs::msg::TargetList>> targetSubscription{nullptr};
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depthSubscription{nullptr};
        std::shared_ptr<message_filters::TimeSynchronizer<nfr_msgs::msg::TargetList, sensor_msgs::msg::Image>> synchronizer{nullptr};
        rclcpp::Publisher<nfr_msgs::msg::TargetList>::SharedPtr targetPublisher;
    public:
        NFRDepthFinderNode(rclcpp::NodeOptions options) : rclcpp::Node("nfr_depth_finder_node", options.clock_type(RCL_SYSTEM_TIME))
        {
            targetSubscription = std::make_shared<message_filters::Subscriber<nfr_msgs::msg::TargetList>>(this, "targets");
            depthSubscription = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "depth/image_rect_raw");
            synchronizer = std::make_shared<message_filters::TimeSynchronizer<nfr_msgs::msg::TargetList, sensor_msgs::msg::Image>>
                (*targetSubscription, *depthSubscription, 10);
            targetPublisher = create_publisher<nfr_msgs::msg::TargetList>("targets_filtered", 10);
            synchronizer->registerCallback(std::bind(&NFRDepthFinderNode::depthCallback, this, std::placeholders::_1, std::placeholders::_2));
        }
        void depthCallback(const nfr_msgs::msg::TargetList::ConstSharedPtr& targets, const sensor_msgs::msg::Image::ConstSharedPtr& depthImage)
        {
            RCLCPP_INFO(get_logger(), "Calcualting depth of %d tags", targets->targets.size());
            nfr_msgs::msg::TargetList filteredTargets;
            for (auto target : targets->targets)
            {
                nfr_msgs::msg::Target filteredTarget = target;
                filteredTarget.depth = (double)depthImage->data[target.center.y * depthImage->width + target.center.x] / 1000;
                filteredTargets.targets.push_back(filteredTarget);
            }
            targetPublisher->publish(filteredTargets);
        }
    };
}
RCLCPP_COMPONENTS_REGISTER_NODE(nfr::NFRDepthFinderNode);