#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import serial
import time

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        
        # Open serial port
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        
        # Timer to periodically send data
        self.timer = self.create_timer(1.0, self.timer_callback)
        
    def timer_callback(self):
        # Send a message to ESP32
        self.serial_port.write(b'Hello from ROS 2!\n')
        self.get_logger().info('Sent: Hello from ROS 2!')

        # Read incoming data from ESP32
        if self.serial_port.in_waiting > 0:
            incoming_data = self.serial_port.readline().decode('utf-8').rstrip()
            self.get_logger().info(f"Received: {incoming_data}")

    def close_serial(self):
        # Close the serial port when done
        self.serial_port.close()

def main(args=None):
    rclpy.init(args=args)
    
    node = SerialNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.close_serial()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
