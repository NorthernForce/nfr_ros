import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
class NFROdometryNode(Node):
    def __init__(self):
        super().__init__('nfr_odometry_node')
        self.odom_topic = self.declare_parameter('odom_topic', 'odom').value
        self.odom_subscription = self.create_subscription(Odometry, self.odom_topic, self.odometry_callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
    def odometry_callback(self, odometry: Odometry):
        odom_to_base_link = TransformStamped()
        odom_to_base_link.header = odometry.header
        odom_to_base_link.child_frame_id = odometry.child_frame_id
        odom_to_base_link.transform.rotation = odometry.pose.pose.orientation
        odom_to_base_link.transform.translation.x = odometry.pose.pose.position.x
        odom_to_base_link.transform.translation.y = odometry.pose.pose.position.y
        odom_to_base_link.transform.translation.z = odometry.pose.pose.position.z
        self.tf_broadcaster.sendTransform(odom_to_base_link)
def main(args=None):
    rclpy.init(args=args)
    odometry_node = NFROdometryNode()
    rclpy.spin(odometry_node)
    odometry_node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()