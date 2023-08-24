import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped, Transform, Quaternion
from tf2_ros import Buffer, TransformListener
from apriltag_msgs.msg import AprilTagDetectionArray, AprilTagDetection
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray as IsaacAprilTagDetectionArray
from isaac_ros_apriltag_interfaces.msg import AprilTagDetection as IsaacAprilTagDetection
import os
from ament_index_python import get_package_share_directory
import json
import numpy as np
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
def calculate_tag_area(coordinates):
    x0 = coordinates[0].x
    y0 = coordinates[0].y
    x1 = coordinates[1].x
    y1 = coordinates[1].y
    x2 = coordinates[2].x
    y2 = coordinates[2].y
    x3 = coordinates[3].x
    y3 = coordinates[3].y
    return abs(0.5 * (x0 * y1 - y0 * x1 + x1 * y2 - y1 * x2 + x2 * y3 - y2 * x3 + x3 * y0 - y3 * x0))
def euler_from_quaternion(quaternion: Quaternion):
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw
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
        self.use_cuda = self.declare_parameter('use_cuda', False).value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.detections_topic = self.declare_parameter('detections_topic', 'detections').value
        self.pose_topic = self.declare_parameter('pose_topic', 'estimated_pose').value
        self.area_threshold = self.declare_parameter('area_threshold', 400.0).value
        self.decision_margin_threshold = self.declare_parameter('decision_margin_threshold', 20.0).value
        self.pose_publisher = self.create_publisher(PoseWithCovarianceStamped, self.pose_topic, 10)
        if self.use_cuda:
            self.detections_subscription = self.create_subscription(IsaacAprilTagDetectionArray, self.detections_topic,
                self.cuda_detections_callback, qos_profile_sensor_data)
        else:
            self.detections_subscription = self.create_subscription(AprilTagDetectionArray, self.detections_topic,
                self.detections_callback, qos_profile_sensor_data)
    def detections_callback(self, detections: AprilTagDetectionArray):
        camera_frame = detections.header.frame_id
        for detection in detections.detections:
            detection: AprilTagDetection
            area = calculate_tag_area(detection.corners)
            self.get_logger().info('Detected tag #%d. Area: %f' % (detection.id, area))
            self.get_logger().info('Tag goodness: %f' % detection.goodness)
            self.get_logger().info('Tag hamming: %d' % detection.hamming)
            self.get_logger().info('Tag decision margin: %f' % detection.decision_margin)
            if detection.decision_margin < self.decision_margin_threshold:
                self.get_logger().warn('Rejecting bad tag detection')
                continue
            if detection.id in self.field.keys():
                tag_frame = '%s:%d' % (detection.family, detection.id)
                if not self.tf_buffer.can_transform(tag_frame, camera_frame, Time()):
                    self.get_logger().warn('Cannot transform %s to %s' % (camera_frame, tag_frame))
                    continue
                camera_to_tag = self.tf_buffer.lookup_transform(tag_frame, camera_frame, Time())
                if not self.tf_buffer.can_transform(camera_frame, self.base_frame, Time()):
                    self.get_logger().warn('Cannot transform %s to %s' % (self.base_frame, camera_frame))
                    continue
                base_to_camera = self.tf_buffer.lookup_transform(camera_frame, self.base_frame, Time())
                world_to_tag = self.field[detection.id]
                pose_transform = subtract_transforms(world_to_tag,
                    add_transforms(camera_to_tag.transform, base_to_camera.transform))
                pose = PoseWithCovarianceStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = detections.header.stamp
                pose.pose.pose.orientation = pose_transform.rotation
                pose.pose.pose.position.x = pose_transform.translation.x
                pose.pose.pose.position.y = pose_transform.translation.y
                pose.pose.pose.position.z = pose_transform.translation.z
                self.pose_publisher.publish(pose)
    def cuda_detections_callback(self, detections: IsaacAprilTagDetectionArray):
        for detection in detections.detections:
            area = calculate_tag_area(detection.corners)
            self.get_logger().info('Detected tag #%d. Area: %f' % (detection.id, area))
            detection: IsaacAprilTagDetection
            if detection.id in self.field.keys() and area > self.area_threshold:
                if not self.tf_buffer.can_transform(self.camera_frame, self.base_frame, Time()):
                    self.get_logger().warn('Cannot transform %s to %s' % (self.base_frame, self.camera_frame))
                    continue
                base_to_camera = self.tf_buffer.lookup_transform(self.camera_frame, self.base_frame, Time())
                camera_to_tag = Transform()
                camera_to_tag.translation.x = detection.pose.pose.pose.position.x
                camera_to_tag.translation.y = detection.pose.pose.pose.position.y
                camera_to_tag.translation.z = detection.pose.pose.pose.position.z
                camera_to_tag.rotation = detection.pose.pose.pose.orientation
                world_to_tag = self.field[detection.id]
                pose_transform = subtract_transforms(world_to_tag,
                    add_transforms(camera_to_tag, base_to_camera.transform))
                yaw = euler_from_quaternion(pose_transform.rotation)[2]
                self.get_logger().info('Estimated Pose to be [%f, %f] with a yaw of %f' % (pose_transform.translation.x,
                    pose_transform.translation.y, yaw))
                pose = PoseWithCovarianceStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.pose.orientation = pose_transform.rotation
                pose.pose.pose.position.x = pose_transform.translation.x
                pose.pose.pose.position.y = pose_transform.translation.y
                pose.pose.pose.position.z = pose_transform.translation.z
                self.pose_publisher.publish(pose)
            else:
                self.get_logger().info('Bad tag detection rejected')
def main(args=None):
    rclpy.init(args=args)
    apriltag_node = NFRApriltagNode()
    rclpy.spin(apriltag_node)
    apriltag_node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()