#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class MotorJointStatePublisher(Node):
    def __init__(self):
        super().__init__('motor_joint_state_publisher')
        self.motor_sub = self.create_subscription(JointState, '/joint_states', self.motor_callback, 10)
        self.static_sub = self.create_subscription(JointState, '/static_joint_states', self.static_callback, 10)
        self.publisher_ = self.create_publisher(JointState, '/joint_states_combined', 10)
        
        # Variable to store messages from each topic
        self.motor_msg = None
        self.static_msg = None

    def motor_callback(self, msg):
        self.motor_msg = msg
        self.try_publish()

    def static_callback(self, msg):
        self.static_msg = msg
        self.try_publish()

    def try_publish(self):
        # Ensure both messages have been received before publishing
        if self.motor_msg is None or self.static_msg is None:
            return

        # Create a new JointState message to combine both motor and static joints
        merged_msg = JointState()
        merged_msg.header.stamp = self.get_clock().now().to_msg()

        # Combine the names and positions from both motor and static messages
        merged_msg.name = list(self.motor_msg.name)  # Start with motor joints
        merged_msg.position = list(self.motor_msg.position)

        # Add the static joints if not already in the merged list
        for i, name in enumerate(self.static_msg.name):
            if name not in merged_msg.name:
                merged_msg.name.append(name)
                merged_msg.position.append(self.static_msg.position[i])

        # Publish the combined joint states
        self.publisher_.publish(merged_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MotorJointStatePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
