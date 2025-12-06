#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_transformations


class AprilTagMarkerNode(Node):
    def __init__(self):
        super().__init__('apriltag_marker_node')

        # Create a subscriber to receive the left ZED camera video feed
        self.subscription = self.create_subscription(
            Image,
            '/zed/zed_node/left/image_rect_color',
            self.image_callback,
            10
        )

        # Create a subscriber to get camera info (for calibration data)
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/zed/zed_node/left/camera_info',
            self.camera_info_callback,
            10
        )

        # Create a publisher for the AprilTag marker pose
        self.publisher = self.create_publisher(PoseStamped, 'apriltag_marker_pose', 200)

        # Create a TF broadcaster to broadcast the marker pose as a transform
        self.tf_broadcaster = TransformBroadcaster(self)

        # Create a publisher for the processed video frame
        self.image_publisher = self.create_publisher(Image, 'apriltag_detection_image', 200)

        self.bridge = CvBridge()

        # Load predefined dictionary of AprilTag markers (Tag36h11)
        # Note: OpenCV's ArUco module supports AprilTag dictionaries
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        self.parameters = cv2.aruco.DetectorParameters_create()

        # Variables to store camera calibration data
        self.camera_matrix = None
        self.dist_coeffs = None

        # Frame skip count to reduce publish frequency
        self.frame_skip = 3
        self.frame_count = 0
        
        self.get_logger().info("AprilTag Marker Node (Tag36h11) Initialized")

    def camera_info_callback(self, msg):
        """Callback to store camera calibration data."""
        self.camera_matrix = np.array(msg.k).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d)

    def image_callback(self, msg):
        # Skip frames to reduce the load on the system
        self.frame_count += 1
        if self.frame_count % self.frame_skip != 0 or self.camera_matrix is None or self.dist_coeffs is None:
            return

        # Convert the ROS2 Image message to an OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Detect the markers in the frame
        corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.parameters)

        if ids is not None:
            # Estimate pose of each marker
            marker_size = 0.1  # Size of AprilTag marker (meters) - Adjust if yours is different!
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_size, self.camera_matrix, self.dist_coeffs)

            for i, marker_id in enumerate(ids):
                if marker_id[0] != 23: continue # Filter for ID 23 only
                
                rvec, tvec = rvecs[i][0], tvecs[i][0]

                # Draw axis on the marker for visualization
                cv2.aruco.drawDetectedMarkers(frame, corners)
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)

                # Convert rotation vector to a rotation matrix
                rotation_matrix, _ = cv2.Rodrigues(rvec)

                # Convert rotation matrix to Euler angles
                euler_angles = tf_transformations.euler_from_matrix(rotation_matrix, 'rxyz')

                # Adjust Euler angles (correcting axis alignment for ZED/ROS standard)
                adjusted_euler = (
                    euler_angles[0],  # Roll
                    -euler_angles[1],   # Pitch
                    -euler_angles[2]   # Yaw
                )

                # Convert adjusted Euler angles back to a rotation matrix
                adjusted_rotation_matrix = tf_transformations.euler_matrix(*adjusted_euler, 'rxyz')[:3, :3]

                # Optional: Apply additional rotation (180 degrees on Z-axis if needed)
                # Apply additional rotation to fix X and Y alignment
                rotation_matrix_x = tf_transformations.euler_matrix(np.pi, 0, 0, 'rxyz')[:3, :3]  # Rotate 180 degrees around Y-axis

                # Apply the rotations in sequence: Z rotation first, then Y rotation
                adjusted_rotation_matrix = np.dot(rotation_matrix_x, adjusted_rotation_matrix)

                # Convert adjusted rotation matrix to quaternion
                quaternion = self.rotation_matrix_to_quaternion(adjusted_rotation_matrix)

                # Create PoseStamped message for ROS2
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = 'zed_left_camera_frame'

                # Set the position
                pose_msg.pose.position.x = tvec[2]
                pose_msg.pose.position.y = -tvec[0]
                pose_msg.pose.position.z = -tvec[1]

                # Set the orientation
                pose_msg.pose.orientation.x = quaternion[0]
                pose_msg.pose.orientation.y = quaternion[1]
                pose_msg.pose.orientation.z = quaternion[2]
                pose_msg.pose.orientation.w = quaternion[3]
                # Publish the pose to rviz2
                self.publisher.publish(pose_msg)

                # Broadcast the transform for TF
                transform = TransformStamped()
                transform.header.stamp = self.get_clock().now().to_msg()
                transform.header.frame_id = 'zed_left_camera_frame'
                transform.child_frame_id = f'apriltag_marker_{marker_id[0]}'
                transform.transform.translation.x = tvec[2]
                transform.transform.translation.y = -tvec[0]
                transform.transform.translation.z = -tvec[1]
                transform.transform.rotation.x = quaternion[0]
                transform.transform.rotation.y = quaternion[1]
                transform.transform.rotation.z = quaternion[2]
                transform.transform.rotation.w = quaternion[3]

                # Send the transform
                self.tf_broadcaster.sendTransform(transform)

        # Publish the video feed with the detected markers
        detection_image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.image_publisher.publish(detection_image_msg)

    def rotation_matrix_to_quaternion(self, R):
        # Convert a rotation matrix to a quaternion
        qw = np.sqrt(1.0 + R[0, 0] + R[1, 1] + R[2, 2]) / 2.0
        qx = (R[2, 1] - R[1, 2]) / (4.0 * qw)
        qy = (R[0, 2] - R[2, 0]) / (4.0 * qw)
        qz = (R[1, 0] - R[0, 1]) / (4.0 * qw)
        return [qx, qy, qz, qw]


def main(args=None):
    rclpy.init(args=args)

    node = AprilTagMarkerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
