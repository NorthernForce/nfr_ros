import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray, AprilTagDetection
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray as IsaacAprilTagDetectionArray
from isaac_ros_apriltag_interfaces.msg import AprilTagDetection as IsaacAprilTagDetection
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import TransformStamped
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from networktables import NetworkTables
class NFRBridgeNode(Node):
    def __init__(self):
        super().__init__('nfr_bridge_node')
        self.declare_parameter('robot_ip', '10.1.72.2')
        self.declare_parameter('apriltag_ros_cameras', rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter('isaac_ros_apriltag_cameras', rclpy.Parameter.Type.STRING_ARRAY)
        for camera_name in self.get_parameter('apriltag_ros_cameras').get_parameter_value().string_array_value:
            self.get_logger().log('Subscribing to apriltag detections at %s' % ('/' + camera_name + '/tag_detections'))
            self.create_subscription(AprilTagDetectionArray, '/' + camera_name + '/tag_detections',
                lambda detection : self.apriltag_ros_detection(camera_name, detection), 10)
        for camera_name in self.get_parameter('isaac_ros_apriltag_cameras').get_parameter_value().string_array_value:
            self.get_logger().log('Subscribing to isaac ros apriltag detections at %s' % ('/' + camera_name + '/tag_detections'))
            self.create_subscription(IsaacAprilTagDetectionArray, '/' + camera_name + '/tag_detections',
                lambda detection : self.isaac_ros_apriltag_detection(camera_name, detection), 10)
        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer, self)
        NetworkTables.initialize(self.get_parameter('robot_ip').get_parameter_value().string_value)
        self.status_entry = NetworkTables.getEntry('/ros/status')
        self.status_entry.setString('online')
        self.odom_table = NetworkTables.getTable('/ros/odometry')
        self.odom_publisher = self.create_publisher(Odometry, 'odom', self.publish_odom, 10)
    def publish_odom(self):
        odom: Odometry
        odom.header.
    def apriltag_ros_detection(self, camera_name: str, detections: AprilTagDetectionArray):
        camera_table = NetworkTables.getTable('/ros/' + camera_name)
        ids_entry = camera_table.getEntry('ids')
        pose_estimations_entry = camera_table.getEntry('pose_estimations')
        ids = []
        poses = []
        for detection in detections:
            detection: AprilTagDetection
            ids.append(float(detection.id))
            pose: TransformStamped = self.tf_buffer.lookup_transform('tag16h5:' + detection.id, camera_name, 0)
            poses.append(pose.transform.translation.x)
            poses.append(pose.transform.translation.y)
            poses.append(pose.transform.translation.z)
            poses.append(pose.transform.rotation.w)
            poses.append(pose.transform.rotation.x)
            poses.append(pose.transform.rotation.y)
            poses.append(pose.transform.rotation.z)
        ids_entry.setDoubleArray(ids)
        pose_estimations_entry.setDoubleArray(poses)
    def isaac_apriltag_ros_detection(self, camera_name: str, detections: IsaacAprilTagDetectionArray):
        camera_table = NetworkTables.getTable('/ros/' + camera_name)
        ids_entry = camera_table.getEntry('ids')
        pose_estimations_entry = camera_table.getEntry('pose_estimations')
        ids = []
        pose_estimations = []
        for detection in detections.detections:
            detection: IsaacAprilTagDetection
            ids.append(float(detection.id))
            pose: Pose = detection.pose.pose.pose
            pose_estimations.append(pose.position.x)
            pose_estimations.append(pose.position.y)
            pose_estimations.append(pose.position.z)
            pose_estimations.append(pose.orientation.w)
            pose_estimations.append(pose.orientation.x)
            pose_estimations.append(pose.orientation.y)
            pose_estimations.append(pose.orientation.z)
        ids_entry.setDoubleArray(ids)
        pose_estimations_entry.setDoubleArray(pose_estimations)
def main(args=None):
    rclpy.init(args)
    bridge_node = NFRBridgeNode()
    rclpy.spin(bridge_node)
    bridge_node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()