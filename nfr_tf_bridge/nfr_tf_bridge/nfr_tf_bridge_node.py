import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.time import Time
from nfr_tf_bridge_msgs.msg import NFRTFRequest
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
class NFRTFBridgeNode(Node):
    def __init__(self):
        super().__init__('nfr_tf_bridge_node')
        self.request_topic = self.declare_parameter('request_topic', 'requests').value
        self.requests = []
        self.lookup_timers = []
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_publishers: dict[str, Publisher] = {}
        self.request_subscription = self.create_subscription(NFRTFRequest, self.request_topic, self.request_callback, 10)
    def request_callback(self, request: NFRTFRequest):
        self.get_logger().info('Recieved TFRequest for %s to %s: Publishing to %s at a rate of %f'
            % (request.source_frame, request.target_frame, request.publish_topic, request.update_rate))
        self.requests.append(request)
        self.tf_publishers[request.publish_topic] = self.create_publisher(TransformStamped, request.publish_topic, 10)
        new_timer = self.create_timer(1.0 / request.update_rate, lambda : self.lookup_callback(request))
        self.lookup_timers.append(new_timer)
    def lookup_callback(self, request: NFRTFRequest):
        self.get_logger().info('Looking up %s to %s' % (request.source_frame, request.target_frame))
        if self.tf_buffer.can_transform(request.target_frame, request.source_frame, Time()):
            self.get_logger().info('Publishing %s to %s to topic \'%s\'' % (request.source_frame, request.target_frame,
                request.publish_topic))
            transform = self.tf_buffer.lookup_transform(request.target_frame, request.source_frame, Time())
            self.tf_publishers[request.publish_topic].publish(transform)
def main(args=None):
    rclpy.init(args=args)
    tf_bridge_node = NFRTFBridgeNode()
    rclpy.spin(tf_bridge_node)
    tf_bridge_node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()