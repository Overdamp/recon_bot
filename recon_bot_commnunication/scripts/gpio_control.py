#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import Jetson.GPIO as GPIO


class GPIOController(Node):
    def __init__(self):
        super().__init__('gpio_controller')

        # Define GPIO pins (use BCM numbering)
        self.gpio_pins = [18, 23, 24]  # Replace with the actual GPIO pins you are using

        # Set up GPIO pins as output
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.gpio_pins, GPIO.OUT, initial=GPIO.LOW)

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
            0: self.gpio_pins[0],  # Button 0 -> GPIO pin 18
            1: self.gpio_pins[1],  # Button 1 -> GPIO pin 23
            2: self.gpio_pins[2]   # Button 2 -> GPIO pin 24
        }

        for button_index, gpio_pin in button_to_gpio.items():
            # Check if the button is pressed and update the corresponding GPIO
            if button_index < len(msg.buttons):
                state = GPIO.HIGH if msg.buttons[button_index] == 1 else GPIO.LOW
                self.set_gpio_state(gpio_pin, state)

    def set_gpio_state(self, gpio_pin, state):
        # Set the GPIO pin to the desired state
        GPIO.output(gpio_pin, state)
        self.get_logger().info(f"Set GPIO {gpio_pin} to {'HIGH' if state == GPIO.HIGH else 'LOW'}")

    def check_gpio_states(self):
        # Read and print the current state of all GPIO pins
        for gpio_pin in self.gpio_pins:
            state = GPIO.input(gpio_pin)
            self.get_logger().info(f"GPIO {gpio_pin} current state: {'HIGH' if state == GPIO.HIGH else 'LOW'}")

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
