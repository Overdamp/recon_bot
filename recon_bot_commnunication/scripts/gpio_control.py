#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import Jetson.GPIO as GPIO
import subprocess
import time


class GPIOController(Node):
    def __init__(self):
        super().__init__('gpio_controller')

        # Define GPIO pins (use BCM numbering)
        self.gpio_pins = [23, 24, 13]  # Replace with the actual GPIO pins you are using
        self.gpio_pin = 22  # Pin 22 (BOARD numbering)
        self.gpio_pin_shutdown = 27  # Pin 22 (BOARD numbering)

        # Set up GPIO pins as output
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.gpio_pins, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.gpio_pin_shutdown, GPIO.IN)
        # Subscribe to joystick messages
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Timer to periodically check and print GPIO states
        self.timer = self.create_timer(0.5, self.check_gpio_states)

    def joy_callback(self, msg):
        # Map joystick buttons to GPIO pins
        button_to_gpio = {
            0: self.gpio_pins[0],  # Button 0 -> GPIO pin 23
            1: self.gpio_pins[1],  # Button 1 -> GPIO pin 24
            2: self.gpio_pins[2]   # Button 2 -> GPIO pin 13
        }

        for button_index, gpio_pin in button_to_gpio.items():
            # Check if the button is pressed and update the corresponding GPIO
            if button_index < len(msg.buttons):
                state = GPIO.HIGH if msg.buttons[button_index] == 1 else GPIO.LOW
                self.set_gpio_state(gpio_pin, state)

    def set_gpio_state(self, gpio_pin, state):
        # Set the GPIO pin to the desired state
        GPIO.output(gpio_pin, state)
        self.get_logger().info(f"Set GPIO kkkkkkk {gpio_pin} to {'HIGH' if state == GPIO.HIGH else 'LOW'}")

    def check_gpio_states(self):
        # อ่านค่าสถานะ GPIO
        gpio_state = GPIO.input(self.gpio_pin_shutdown)
        if gpio_state == GPIO.LOW:
            self.get_logger().info('GPIO Pin {} is LOW, initiating shutdown...'.format(self.gpio_pin_shutdown))
            try:
                # ส่งคำสั่ง shutdown
                self.get_logger().info('ok')
                # subprocess.run([ 'shutdown', '-h', 'now'], check=True)
            except subprocess.CalledProcessError as e:
                self.get_logger().error('Failed to shutdown: {}'.format(e))
        else:
            self.get_logger().info('GPIO Pin {} is HIGH, no action taken'.format(self.gpio_pin_shutdown))
        

    def destroy_node(self):
        # Cleanup GPIO before exiting
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    gpio_controller = GPIOController()
    rclpy.spin(gpio_controller)
    gpio_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

