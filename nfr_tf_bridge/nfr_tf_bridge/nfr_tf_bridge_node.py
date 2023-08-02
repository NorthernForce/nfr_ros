import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.publisher import Publisher
from nfr_tf_bridge_msgs.srv import LookupTransform
from nfr_tf_bridge_msgs.msg import RequestTransform
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
class NFRTFBridgeNode(Node):
    def __init__(self):
        super().__init__('nfr_tf_bridge_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.lookup_transform_service = self.create_service(LookupTransform, 'lookup_transform', self.lookup_transform)
        self.request_transform_subscription = self.create_subscription(RequestTransform, 'request_transform',
            self.request_transform, 10)
        self.transform_publishers: dict[str, Publisher] = {}
        self.transform_timers = {}
    def lookup_transform(self, request: LookupTransform.Request, response: LookupTransform.Response):
        if self.tf_buffer.can_transform(request.child_frame, request.base_frame, request.time):
            response.transform = self.tf_buffer.lookup_transform(request.child_frame, request.base_frame, request.time)
            response.success = True
            return response
        response.success = False
        return response
    def request_transform(self, request: RequestTransform):
        self.transform_publishers[request.publish_topic] = self.create_publisher(TransformStamped, request.publish_topic,
            10)
        self.transform_timers[request.publish_topic] = self.create_timer(1.0 / request.frequency, lambda :
            self.publish_transform(request))
    def publish_transform(self, request: RequestTransform):
        if self.tf_buffer.can_transform(request.child_frame, request.base_frame, Time()):
            self.transform_publishers[request.publish_topic].publish(
                self.tf_buffer.lookup_transform(request.child_frame, request.base_frame, Time()))
def main(args=None):
    rclpy.init(args=args)
    nfr_tf_bridge_node = NFRTFBridgeNode()
    rclpy.spin(nfr_tf_bridge_node)
    nfr_tf_bridge_node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()