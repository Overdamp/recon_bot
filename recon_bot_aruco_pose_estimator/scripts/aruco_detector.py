#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from cv2 import aruco
import tf2_ros
import geometry_msgs.msg as gm
import numpy as np
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Header
import time
import datetime

from ros2_unitree_legged_msgs.msg import ArucoData

class ArUcoDetector(Node):
    def __init__(self):
        self.cv_image = None  # เพิ่มตัวแปร cv_image
        super().__init__('aruco_detector')
        self.declare_parameter('camera_topic', '/front_camera/image_raw')
        self.declare_parameter('aruco_dict', 'DICT_5X5_1000')
        self.camera_topic = self.get_parameter('camera_topic').value
        self.aruco_dict = self.get_parameter('aruco_dict').value
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, self.camera_topic, self.image_callback, 5)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.data_list = [0, 0, 0, 0] # velo0 velo1 angula linerz linearx angulaz

        # Camera calibration parameters for robodog
        #self.camera_matrix = np.array([[463.39707108486357, 0, 478.2778309916386], [0, 465.95929687109356, 412.79773730103966], [0, 0, 1]],dtype=np.float64)
        #self.dist_coeffs = np.array([0.015339922734818098, -0.021919688308920233, 0.005629532882637937, 0.0058299275834138, 0.00865459264669645], dtype=np.float64)
        
        # Camera calibration parameters for logitech1080
        self.camera_matrix = np.array([[223.881126232682, 0.0, 159.8378364170643],
                         [0.0, 218.45267820493902, 122.04625808191398],
                         [0.0, 0.0, 1.0]])

        self.dist_coeffs = np.array([-0.016009178729562527, 0.11921019486840884, 0.009659941300842727, -0.010098681419324329, -0.36397715238090717])
        
        self.aruco_data = self.create_publisher(
            ArucoData,
            'aruco_data',
            10)
        self.time_state=0.8
   	

### -------------------------------------- delay time -----------------------------------------------------------------###           
    def delay(self, seconds):
        end_time = datetime.datetime.now() + datetime.timedelta(seconds=seconds)
        while datetime.datetime.now() < end_time:
           pass
### -------------------------------------- delay time -----------------------------------------------------------------###  
      

# ------------------------------------------------------------------------- z axis -----------------------------------------------------------#         
    def calculationz(self,distance):
        set_point = [0.8, 1.0]
        mid_point = (set_point[0] + set_point[1])/2

        mid_velocity = 0.2

        if distance > set_point[1]:
           error = distance - mid_point
           velocity = round((mid_velocity + (error * 0.3)),2)
        
        # print(f'forward {velocity}')

        elif distance < set_point[0]:
           error = distance - mid_point
           velocity = round((-mid_velocity + (error * 0.3)),2)
        # print(f'backward {velocity}')
    
        #if distance >= set_point[0] and distance <= set_point[1]:
        else:
           velocity = 0
        # print(f'stop {velocity}')
        self.check_limitz(velocity)

    def check_limitz(self, velocity):
        limit = 2
        if velocity > 0:
           if velocity >= limit:
              velocity_use = limit
           if velocity < limit:
              velocity_use = velocity
           print(f'forward {velocity_use}')
        if velocity < 0:
           velocity = abs(velocity)
           Dir = -1
           if velocity >= limit:
              velocity_use = Dir * limit
           if velocity < limit:
              velocity_use = velocity * Dir
           print(f'backward {velocity_use}')
        elif velocity == 0 :
           velocity_use = 0
           print(f'stop {velocity_use}')  
        self.data_list[0] = velocity_use
# ------------------------------------------------------------------------- z axis -----------------------------------------------------------#   
# ------------------------------------------------------------------------- x axis -----------------------------------------------------------#         
    def calculationx(self,distance):
        set_point = [-0.1, 0.1]
        mid_point = (set_point[0] + set_point[1])/2

        mid_velocity = 0.2
        

        if distance > set_point[1]:
           error = distance - mid_point
           velocity = round((mid_velocity + (error * 0.2)),2)
        
        # print(f'left {velocity}')

        elif distance < set_point[0]:
           error = distance - mid_point
           velocity = round((-mid_velocity + (error * 0.2)),2)
        # print(f'right {velocity}')
    
        #if distance >= set_point[0] and distance <= set_point[1]:
        else:
           velocity = 0
        # print(f'stop {velocity}')
        self.check_limitx(velocity)

    def check_limitx(self, velocity):
        limit = 1
        if velocity > 0:
           if velocity >= limit:
              velocity_use = limit * -1
           if velocity < limit:
              velocity_use = velocity * -1
           print(f'right {velocity_use}')
        if velocity < 0:
           velocity = abs(velocity)
           Dir = -1
           if velocity >= limit:
              velocity_use = Dir * limit * -1
           if velocity < limit:
              velocity_use = velocity * Dir * -1
           print(f'left {velocity_use}')
        elif velocity == 0 :
           velocity_use = 0
           print(f'stop {velocity_use}')  
        self.data_list[1] = velocity_use
# ------------------------------------------------------------------------- x axis -----------------------------------------------------------# 
# ------------------------------------------------------------------------- pitch -----------------------------------------------------------#         
    def calculation_yaw(self, angular):
        set_point = [-7, 7]
        mid_point = (set_point[0] + set_point[1])/2

        mid_velocity = 0.1

        if angular > set_point[1]:
           error = angular - mid_point
           velocity = round((mid_velocity + (error * 0.02)),2)
        
        # print(f'left {velocity}')

        elif angular < set_point[0]:
           error = angular - mid_point
           velocity = round((-mid_velocity + (error * 0.02)),2)
        # print(f'right {velocity}')
    
        #if angular >= set_point[0] and angular <= set_point[1]:
        else:
           velocity = 0
        # print(f'stop {velocity}')
        self.check_limit_yaw(velocity)

    def check_limit_yaw(self, velocity):
        limit = 2.5
        if velocity > 0:
           if velocity >= limit:
              velocity_use = limit
           if velocity < limit:
              velocity_use = velocity
           print(f'CCW {velocity_use}')
        if velocity < 0:
           velocity = abs(velocity)
           Dir = -1
           if velocity >= limit:
              velocity_use = Dir * limit
           if velocity < limit:
              velocity_use = velocity * Dir
           print(f'CW {velocity_use}')
        elif velocity == 0 :
           velocity_use = 0
           print(f'stop {velocity_use}')  
        self.data_list[2] = velocity_use
# ------------------------------------------------------------------------- pitch -----------------------------------------------------------#                         
                                       
    def image_callback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            # ...
        except Exception as e:
            self.get_logger().error(f'Error converting image: {str(e)}')
            return

        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)
        parameters = aruco.DetectorParameters()
        corners, ids, _ = aruco.detectMarkers(self.cv_image, aruco_dict, parameters=parameters)
        
        if ids is not None:
            
            for i in range(len(ids)):
                # Estimate pose of the marker
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[i], 0.20, self.camera_matrix, self.dist_coeffs)

                # Convert rvec and tvec to ROS TF2 transform
                transform = gm.TransformStamped()
                transform.header.stamp = self.get_clock().now().to_msg()
                transform.header.frame_id = 'camera_frame'  # Change to your camera frame ID
                transform.child_frame_id = f'aruco_{ids[i]}'  # Change to a suitable frame ID

                # Check if tvec is not None and has the correct shape (1, 1, 3)
                if tvec is not None and tvec.shape == (1, 1, 3):
                    # Convert tvec to a Python list
                    tvec_list = tvec.tolist()[0][0]

                    # Assign the transformed values
                    transform.transform.translation.x = float(tvec_list[0])
                    transform.transform.translation.y = float(tvec_list[1])
                    transform.transform.translation.z = float(tvec_list[2])
                    self.get_logger().info(f"ArUco {ids[i]} => Translation: x={tvec_list[0]}, y={tvec_list[1]}, z={tvec_list[2]}")
                    #self.delay(0.5)
                    linear_z = float(tvec_list[2])
                    linear_x = float(tvec_list[0])
                    
                    
                else:
                    # Handle the case where tvec is invalid or has an unexpected shape
                    self.get_logger().error("Invalid tvec: {0}".format(tvec))
                    continue

                # Check if rvec is not None and has the correct shape (1, 1, 3)
                if rvec is not None and rvec.shape == (1, 1, 3):
                    # Convert rvec to a rotation matrix
                    rot = R.from_rotvec(rvec[0][0])
                    # Convert the rotation matrix to a quaternion
                    quat = rot.as_quat()

                    # Assign the quaternion values to the transform
                    transform.transform.rotation.x = quat[0]
                    transform.transform.rotation.y = quat[1]
                    transform.transform.rotation.z = quat[2]
                    transform.transform.rotation.w = quat[3]
                    #self.get_logger().info(f"ArUco {ids[i]} Rotation: x={quat[0]}, y={quat[1]}, z={quat[2]}, w={quat[3]}")
                    #self.delay(0.5)
                    
                    # Assuming you already have the quaternion in the `quat` variable
                    euler = R.from_quat([quat[0], quat[1], quat[2], quat[3]]).as_euler('xyz', degrees=True)
                    # Extract Euler angles (in degrees)
                    roll, pitch, yaw = euler[0], euler[1], euler[2]
                    # Print the Euler angles
                    #print(f"Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")
                    self.get_logger().info(f"Euler => Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")
                    #self.delay(0.5)

                else:
                    # Handle the case where rvec is invalid or has an unexpected shape
                    self.get_logger().error("Invalid rvec: {0}".format(rvec))
                    continue

                # Broadcast the transform
                print("++++++++++++++++++++++++++++++++++ start send value ++++++++++++++++++++++++++++++++++++++")
                self.tf_broadcaster.sendTransform(transform)
                #self.move_linear(linear_z)
                #self.move_angular(pitch)
                
                ArucoData_msg = ArucoData()
                ArucoData_msg.aruco_info  = [float(self.data_list[0]), float(self.data_list[1]), float(self.data_list[2]), float(self.data_list[3])]
                self.aruco_data.publish(ArucoData_msg)
            	 
                self.calculationz(linear_z)
                self.data_list[3] = linear_z
                #self.data_list[4] = linear_x
                #self.data_list[5] = pitch
                self.calculationx(linear_x)
                self.calculation_yaw(pitch)
                a = self.data_list
                print(a)
        else:
           ArucoData_msg = ArucoData()
           ArucoData_msg.aruco_info  = [0.0, 0.0, 0.0, 0.0]
           self.aruco_data.publish(ArucoData_msg)
           print("***** stop *****")
           
   
def main(args=None):
    rclpy.init(args=args)
    aruco_detector = ArUcoDetector()
    rclpy.spin(aruco_detector)
    aruco_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
