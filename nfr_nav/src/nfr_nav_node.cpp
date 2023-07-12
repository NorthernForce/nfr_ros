#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/srv/manage_lifecycle_nodes.hpp>
using namespace rclcpp;
using namespace std;
using ManageLifecycleNodes = nav2_msgs::srv::ManageLifecycleNodes;
using ManageLifecycleNodes_Request = nav2_msgs::srv::ManageLifecycleNodes_Request;
namespace nfr_ros
{
    class NFRNavNode : public Node
    {
    private:
        shared_ptr<Client<ManageLifecycleNodes>> manageLifecycleNodes;
    public:
        NFRNavNode() : Node("nfr_nav_node")
        {
            manageLifecycleNodes = create_client<ManageLifecycleNodes>("/lifecycle_manager/manage_nodes");
            const auto& request = std::make_shared<ManageLifecycleNodes_Request>();
            request->command = ManageLifecycleNodes_Request::STARTUP;
            RCLCPP_INFO(get_logger(), "Trying to start up nodes");
            const auto& future = manageLifecycleNodes->async_send_request(request);
            future.wait();
            RCLCPP_INFO(get_logger(), "Successfully started up nodes");
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