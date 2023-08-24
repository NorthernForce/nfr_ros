import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
class NFROdometryNode(Node):
    def __init__(self):
        super().__init__('nfr_odometry_node')
        self.odom_topic = self.declare_parameter('odom_topic', 'odom').value
        self.odom_subscription = self.create_subscription(Odometry, self.odom_topic, self.odometry_callback,
            qos_profile_sensor_data)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_timer = self.create_timer(0.05, self.publish_tf)
        self.odom_to_base_link = None
    def odometry_callback(self, odometry: Odometry):
        odometry_time = odometry.header.stamp.sec + odometry.header.stamp.nanosec / 1.0e+9
        current_stamp = self.get_clock().now().to_msg()
        current_time = current_stamp.sec + current_stamp.nanosec / 1.0e+9
        if abs(current_time - odometry_time) > 1:
            self.get_logger().warn('Recieved odometry message that is over one second late')
        if self.odom_to_base_link is None:
            self.odom_to_base_link = TransformStamped()
        self.odom_to_base_link.header = odometry.header
        self.odom_to_base_link.child_frame_id = odometry.child_frame_id
        self.odom_to_base_link.transform.rotation = odometry.pose.pose.orientation
        self.odom_to_base_link.transform.translation.x = odometry.pose.pose.position.x
        self.odom_to_base_link.transform.translation.y = odometry.pose.pose.position.y
        self.odom_to_base_link.transform.translation.z = odometry.pose.pose.position.z
    def publish_tf(self):
        if self.odom_to_base_link is not None:
            self.tf_broadcaster.sendTransform(self.odom_to_base_link)
def main(args=None):
    rclpy.init(args=args)
    odometry_node = NFROdometryNode()
    rclpy.spin(odometry_node)
    odometry_node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()