#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class StaticJointStatePublisher(Node):
    def __init__(self):
        super().__init__('static_joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, '/static_joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_static_joint_states)  # 10Hz

    def publish_static_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        # ชื่อ joint ที่ไม่มี sensor และต้องการค่าคงที่
        msg.name = [
            'Pin_Head_L', 'Pin_Head_R', 
            'Roller_LF_1', 'Roller_LF_2', 'Roller_LF_3', 'Roller_LF_4', 'Roller_LF_5', 'Roller_LF_6', 'Roller_LF_7', 'Roller_LF_8',
            'Roller_LR_1', 'Roller_LR_2', 'Roller_LR_3', 'Roller_LR_4', 'Roller_LR_5', 'Roller_LR_6', 'Roller_LR_7', 'Roller_LR_8',
            'Roller_RF_1', 'Roller_RF_2', 'Roller_RF_3', 'Roller_RF_4', 'Roller_RF_5', 'Roller_RF_6', 'Roller_RF_7', 'Roller_RF_8',
            'Roller_RR_1', 'Roller_RR_2', 'Roller_RR_3', 'Roller_RR_4', 'Roller_RR_5', 'Roller_RR_6', 'Roller_RR_7', 'Roller_RR_8'
        ]
        # กำหนดตำแหน่งค่าคงที่ เช่น 0.0 สำหรับทุก joint
        msg.position = [0.0] * len(msg.name)

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = StaticJointStatePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
