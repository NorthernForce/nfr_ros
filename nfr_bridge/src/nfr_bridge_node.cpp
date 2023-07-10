#include <rclcpp/rclcpp.hpp>
#include <ntcore/networktables/NetworkTableInstance.h>
#include <ntcore/networktables/NetworkTable.h>
#include <tinyxml2.h>
#include <cscore/cscore_oo.h>
#include <nav_msgs/msg/odometry.hpp>
using namespace rclcpp;
using namespace std;
using namespace tinyxml2;
using Odometry = nav_msgs::msg::Odometry;
namespace nfr_ros
{
    class NFRBridgeNode : public Node
    {
    private:
        nt::NetworkTableInstance instance;
        shared_ptr<nt::NetworkTable> rosTable, odometryTable;
        shared_ptr<Publisher<Odometry>> odometryPublisher;
    public:
        NFRBridgeNode() : Node("nfr_bridge_node")
        {
            instance = nt::NetworkTableInstance::Create();
            instance.SetServerTeam(declare_parameter("team", 172));
            instance.StartClient4(declare_parameter("client_name", "coprocessor"));
            rosTable = instance.GetTable(declare_parameter("ros_table", "ros"));
            XMLDocument document;
            document.LoadFile(declare_parameter("topics_xml", "").c_str());
            if (document.Error())
            {
                RCLCPP_ERROR(get_logger(), "Could not load %s", get_parameter("topics_xml").get_value<string>());
            }
            else
            {
                const auto& bridge = document.FirstChildElement("bridge");
                const auto& imports = bridge->FirstChildElement("imports");
                const auto& exports = bridge->FirstChildElement("exports");
                for (auto import = imports->FirstChildElement(); import; import = import->NextSiblingElement())
                {
                    if (string(import->Name()) == "odometry")
                    {
                        odometryTable = rosTable->GetSubTable(import->FirstChildElement("source_topic")->Value());
                        odometryPublisher = create_publisher<Odometry>(
                            string(import->FirstChildElement("target_topic")->Value()),
                            10
                        );
                        create_wall_timer(10ms, [&]() {
                            double timestamp = odometryTable->GetEntry("timestamp").GetDouble(0);
                            Odometry odom;
                            odom.header.frame_id = "odom";
                            odom.child_frame_id = "base_link";
                            odom.header.stamp = Time(timestamp);
                            odom.pose.pose.position.x = odometryTable->GetEntry("x").GetDouble(0);
                        });
                    }
                }
                for (auto _export = exports->FirstChildElement(); _export; _export = _export->NextSiblingElement())
                {
                    if (string(_export->Name()) == "camera")
                    {
                        string sourceTopic = _export->FirstChildElement("source_topic")->Value();
                        string cameraName = _export->FirstChildElement("name")->Value();
                        int cameraPort = stoi(_export->FirstChildElement("port")->Value());
                    }
                }
            }
        }
    };
}
int main(int argc, char const* argv[])
{
    init(argc, argv);
    const auto& node = make_shared<nfr_ros::NFRBridgeNode>();
    spin(node);
    shutdown();
    return 0;
}