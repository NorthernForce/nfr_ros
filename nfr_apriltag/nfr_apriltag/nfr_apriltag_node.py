import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped, Transform, Quaternion
from tf2_ros import Buffer, TransformListener
from apriltag_msgs.msg import AprilTagDetectionArray, AprilTagDetection
import os
from ament_index_python import get_package_share_directory
import json
def add_quaternions(a: Quaternion, b: Quaternion):
    c = Quaternion()
    c.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
    c.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y
    c.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x
    c.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
    return c
def add_transforms(a: Transform, b: Transform):
    c = Transform()
    c.translation.x = a.translation.x + b.translation.x
    c.translation.y = a.translation.y + b.translation.y
    c.translation.z = a.translation.z + b.translation.z
    c.rotation = add_quaternions(a.rotation, b.rotation)
    return c
def inverse_quaternion(a: Quaternion):
    c = Quaternion()
    magnitude_squared = a.w ** 2 + a.x ** 2 + a.y ** 2 + a.z ** 2
    c.w = a.w / magnitude_squared
    c.x = a.x / magnitude_squared
    c.y = a.y / magnitude_squared
    c.z = a.z / magnitude_squared
    return c
def subtract_transforms(a: Transform, b: Transform):
    c = Transform()
    c.translation.x = a.translation.x - b.translation.x
    c.translation.y = a.translation.y - b.translation.y
    c.translation.z = a.translation.z - b.translation.z
    c.rotation = add_quaternions(a.rotation, inverse_quaternion(b.rotation))
    return c
class NFRApriltagNode(Node):
    def __init__(self):
        super().__init__('nfr_apriltag_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.field_path = self.declare_parameter('field_path',
            os.path.join(get_package_share_directory('nfr_charged_up'), 'config', 'field.json')).value
        self.field: dict[int, Transform] = {}
        self.get_logger().info('Loading field json from %s' % self.field_path)
        with open(self.field_path) as f:
            field_json = json.load(f)
            for tag in field_json['tags']:
                tag_pose = Transform()
                tag_pose.translation.x = tag['pose']['translation']['x']
                tag_pose.translation.y = tag['pose']['translation']['y']
                tag_pose.translation.z = tag['pose']['translation']['z']
                tag_pose.rotation.w = tag['pose']['rotation']['quaternion']['W']
                tag_pose.rotation.x = tag['pose']['rotation']['quaternion']['X']
                tag_pose.rotation.y = tag['pose']['rotation']['quaternion']['Y']
                tag_pose.rotation.z = tag['pose']['rotation']['quaternion']['Z']
                self.field[tag['ID']] = tag_pose
        self.camera_frame = self.declare_parameter('camera_frame', 'camera_link').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.detections_topic = self.declare_parameter('detections_topic', 'detections').value
        self.pose_topic = self.declare_parameter('pose_topic', 'estimated_pose').value
        self.pose_publisher = self.create_publisher(PoseWithCovarianceStamped, self.pose_topic, 10)
        self.detections_subscription = self.create_subscription(AprilTagDetectionArray, self.detections_topic,
            self.detections_callback, 10)
    def detections_callback(self, detections: AprilTagDetectionArray):
        for detection in detections.detections:
            detection: AprilTagDetection
            if detection.id in self.field.keys():
                tag_frame = '%s:%d' % (detection.family, detection.id)
                if not self.tf_buffer.can_transform(tag_frame, self.camera_frame, Time()):
                    self.get_logger().warn('Cannot transform %s to %s' % (self.camera_frame, tag_frame))
                    continue
                camera_to_tag = self.tf_buffer.lookup_transform(tag_frame, self.camera_frame, Time())
                if not self.tf_buffer.can_transform(self.camera_frame, self.base_frame, Time()):
                    self.get_logger().warn('Cannot transform %s to %s' % (self.base_frame, self.camera_frame))
                    continue
                base_to_camera = self.tf_buffer.lookup_transform(self.camera_frame, self.base_frame, Time())
                world_to_tag = self.field[detection.id]
                pose_transform = subtract_transforms(world_to_tag,
                    add_transforms(camera_to_tag.transform, base_to_camera.transform))
                pose = PoseWithCovarianceStamped()
                pose.header.frame_id = "map"
                pose.header.stamp = self.get_clock().now()
                pose.pose.pose.orientation = pose_transform.rotation
                pose.pose.pose.position.x = pose_transform.translation.x
                pose.pose.pose.position.y = pose_transform.translation.y
                pose.pose.pose.position.z = pose_transform.translation.z
                self.pose_publisher.publish(pose)
def main(args=None):
    rclpy.init(args=args)
    apriltag_node = NFRApriltagNode()
    rclpy.spin(apriltag_node)
    apriltag_node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()