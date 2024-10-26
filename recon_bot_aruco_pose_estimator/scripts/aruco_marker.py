#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped

class ArucoMarkerNode(Node):
    def __init__(self):
        super().__init__('aruco_marker_node')
        
        # Create a subscriber to receive the left ZED camera video feed
        self.subscription = self.create_subscription(
            Image,
            '/zed2/zed_node/left/image_rect_color',
            self.image_callback,
            10
        )
        
        # Create a publisher for the ArUco marker pose
        self.publisher = self.create_publisher(PoseStamped, '/aruco_marker_pose', 10)
        
        self.bridge = CvBridge()
        
        # Load predefined dictionary of ArUco markers
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters_create()
        
        # Camera calibration parameters loaded from file (calibration_matrix.yaml)
        self.camera_matrix = np.array([[700, 0, 640], [0, 700, 360], [0, 0, 1]])  # Placeholder values, replace with actual calibration data
        self.dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0])  # Placeholder values, replace with actual calibration data

    def image_callback(self, msg):
        # Convert the ROS2 Image message to an OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Detect the markers in the frame
        corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.parameters)

        if ids is not None:
            # Estimate pose of each marker
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, self.camera_matrix, self.dist_coeffs)
            
            for i, marker_id in enumerate(ids):
                rvec, tvec = rvecs[i][0], tvecs[i][0]
                
                # Draw axis on the marker for visualization
                cv2.aruco.drawAxis(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)
                
                # Convert rotation vector to a rotation matrix
                rotation_matrix, _ = cv2.Rodrigues(rvec)
                
                # Create PoseStamped message for ROS2
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = 'zed_left_camera'
                
                # Set the position
                pose_msg.pose.position.x = tvec[0]
                pose_msg.pose.position.y = tvec[1]
                pose_msg.pose.position.z = tvec[2]
                
                # Convert rotation matrix to quaternion
                quaternion = self.rotation_matrix_to_quaternion(rotation_matrix)
                pose_msg.pose.orientation.x = quaternion[0]
                pose_msg.pose.orientation.y = quaternion[1]
                pose_msg.pose.orientation.z = quaternion[2]
                pose_msg.pose.orientation.w = quaternion[3]
                
                # Publish the pose to rviz2
                self.publisher.publish(pose_msg)

        # Show the video feed (optional)
        cv2.imshow('Left ZED Camera - ArUco Detection', frame)
        cv2.waitKey(1)

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
