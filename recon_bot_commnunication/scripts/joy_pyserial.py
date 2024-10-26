#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy  # Import the Joy message type
import serial

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        
        # Open serial port
        self.serial_port = serial.Serial('/dev/ttyUSB_ESP32', 115200, timeout=1)
        
        # Create subscriber to the /joy topic
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.subscription  # Prevent unused variable warning

    def send_button_signal(self, button):
        if button == 0:  # Button 0 for CW
            self.serial_port.write(b'1\n')
            self.get_logger().info("Sent button 1 signal (OUT)")
        elif button == 1:  # Button 1 for CCW
            self.serial_port.write(b'2\n')
            self.get_logger().info("Sent button 2 signal (IN)")

    def joy_callback(self, joy_msg):
        # Check the state of button[0] (CW) and button[1] (CCW)
        if joy_msg.buttons[0]:  # Button 0 pressed
            self.send_button_signal(0)
            self.read_serial_status()  # Read status after sending command
        if joy_msg.buttons[1]:  # Button 1 pressed
            self.send_button_signal(1)
            self.read_serial_status()  # Read status after sending command

    def read_serial_status(self):
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
