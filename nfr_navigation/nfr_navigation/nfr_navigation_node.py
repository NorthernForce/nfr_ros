import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
class NFRNavigationNode(Node):
    def __init__(self):
        super().__init__('nfr_navigation_node')
        self.go_to_pose_topic = self.declare_parameter('go_to_pose_topic', 'navigate_to_pose').value
        self.go_to_pose_client = ActionClient(self, NavigateToPose, self.go_to_pose_topic)
        self.target_pose_topic = self.declare_parameter('target_pose_topic', 'target_pose').value
        self.target_pose_subscription = self.create_subscription(PoseStamped, self.target_pose_topic,
            self.target_pose_callback, 10)
    def target_pose_callback(self, target_pose: PoseStamped):
        go_to_pose_goal = NavigateToPose.Goal()
        go_to_pose_goal.pose = target_pose
        go_to_pose_goal.behavior_tree = ''
        self.go_to_pose_client.wait_for_server()
        self.go_to_pose_client.send_goal_async(go_to_pose_goal)
def main(args=None):
    rclpy.init(args=args)
    navigation_node = NFRNavigationNode()
    rclpy.spin(navigation_node)
    navigation_node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()