#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
import yaml
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_transformations


class ArucoMarkerNode(Node):
    def __init__(self):
        super().__init__('aruco_marker_node')

        # Create a subscriber to receive the left ZED camera video feed
        self.subscription = self.create_subscription(
            Image,
            '/zed/zed_node/left/image_rect_color',
            self.image_callback,
            10
        )

        # Create a publisher for the ArUco marker pose
        self.publisher = self.create_publisher(PoseStamped, 'aruco_marker_pose', 200)

        # Create a TF broadcaster to broadcast the marker pose as a transform
        self.tf_broadcaster = TransformBroadcaster(self)

        # Create a publisher for the processed video frame
        self.image_publisher = self.create_publisher(Image, 'aruco_detection_image', 200)

        self.bridge = CvBridge()

        # Load predefined dictionary of ArUco markers
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_100)
        self.parameters = cv2.aruco.DetectorParameters()

        # Load camera calibration parameters from file
        with open('/home/luke/Ros2_Directory/recon_ws/src/recon_bot_aruco_pose_estimator/config/calibration_matrix.yaml', 'r') as file:
            calibration_data = yaml.safe_load(file)
            self.camera_matrix = np.array(calibration_data['camera_matrix'])
            self.dist_coeffs = np.array(calibration_data.get('dist_coefficients', np.zeros((1, 5))))

        # Frame skip count to reduce publish frequency
        self.frame_skip = 3
        self.frame_count = 0

    def image_callback(self, msg):
        # Skip frames to reduce the load on the system
        self.frame_count += 1
        if self.frame_count % self.frame_skip != 0:
            return

        # Convert the ROS2 Image message to an OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Detect the markers in the frame
        corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.parameters)

        if ids is not None:
            print("Detected")
            # Estimate pose of each marker
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, self.camera_matrix, self.dist_coeffs)

            for i, marker_id in enumerate(ids):
                print(f"arUco {marker_id[0]} found")
                rvec, tvec = rvecs[i][0], tvecs[i][0]

                # Draw axis on the marker for visualization
                cv2.aruco.drawDetectedMarkers(frame, corners)
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)

                # Draw the coordinates on the marker
                position_str = f"x: {tvec[0]:.2f}, y: {tvec[1]:.2f}, z: {tvec[2]:.2f}"
                corner = corners[i][0][0]  # Top-left corner of the marker
                cv2.putText(frame, position_str, (int(corner[0]), int(corner[1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Convert rotation vector to a rotation matrix
                rotation_matrix, _ = cv2.Rodrigues(rvec)

                # Convert rotation matrix to Euler angles
                euler_angles = tf_transformations.euler_from_matrix(rotation_matrix, 'rxyz')

                # Adjust Euler angles if needed
                # Example: Rotate around X axis (roll) by 90 degrees
                adjusted_euler = (
                    euler_angles[0],  # Keep roll the same
                    euler_angles[1],  # Keep pitch the same
                    euler_angles[2]   # Keep yaw the same
                )

                # Convert adjusted Euler angles back to a rotation matrix
                rotation_matrix = tf_transformations.euler_matrix(*adjusted_euler, 'rxyz')[:3, :3]

                # Create PoseStamped message for ROS2
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = 'zed_left_camera_frame'

                # Set the position
                pose_msg.pose.position.x = tvec[0]
                pose_msg.pose.position.y = tvec[1]
                pose_msg.pose.position.z = tvec[2]

                # Convert adjusted rotation matrix to quaternion
                quaternion = self.rotation_matrix_to_quaternion(rotation_matrix)
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
                transform.child_frame_id = f'aruco_marker_{marker_id[0]}'
                transform.transform.translation.x = tvec[0]
                transform.transform.translation.y = tvec[1]
                transform.transform.translation.z = tvec[2]
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

    node = ArucoMarkerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
