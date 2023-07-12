#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
using namespace rclcpp;
using namespace std;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
template<typename T>
using ActionClient = rclcpp_action::Client<T>;
using PoseStamped = geometry_msgs::msg::PoseStamped;
namespace nfr_ros
{
    class NFRNavNode : public Node
    {
    private:
        shared_ptr<ActionClient<NavigateToPose>> navigateToPose;
        shared_ptr<Subscription<PoseStamped>> targetPoseSubscriber;
    public:
        NFRNavNode() : Node("nfr_nav_node")
        {
            navigateToPose = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
            targetPoseSubscriber = create_subscription<PoseStamped>("go_to_pose_destination", 10, [&](const PoseStamped& pose)
            {
                NavigateToPose::Goal goal;
                goal.pose = pose;
                navigateToPose->async_send_goal(goal);
            });
        }
    };
}
int main(int argc, char const* argv[])
{
    init(argc, argv);
    const auto& node = make_shared<nfr_ros::NFRNavNode>();
    spin(node);
    shutdown();
    return 0;
}