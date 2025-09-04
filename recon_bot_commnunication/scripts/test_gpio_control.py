#!/usr/bin/env python3
"""
Jetson Nano GPIO Control Example
ตัวอย่างการควบคุม GPIO บน Jetson Nano
"""

import Jetson.GPIO as GPIO
import time

# ตั้งค่า GPIO Mode
GPIO.setmode(GPIO.BOARD)  # หรือใช้ GPIO.BCM

# กำหนด Pin Numbers (ตาม BOARD numbering)
# LED Output Pins
LED_PIN = 7    # GPIO 4 (Pin 7)
LED_PIN2 = 11  # GPIO 17 (Pin 11)

# Button Input Pins
BUTTON_PIN = 12  # GPIO 18 (Pin 12)
BUTTON_PIN2 = 13 # GPIO 27 (Pin 13)

# PWM Pin
PWM_PIN = 33  # GPIO 13 (Pin 33)

def setup_gpio():
    """ตั้งค่าเริ่มต้น GPIO"""
    try:
        # ตั้งค่า Output pins
        GPIO.setup(LED_PIN, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(LED_PIN2, GPIO.OUT, initial=GPIO.LOW)
        
        # ตั้งค่า Input pins with pull-up resistor
        GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(BUTTON_PIN2, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        # ตั้งค่า PWM
        GPIO.setup(PWM_PIN, GPIO.OUT)
        
        print("GPIO Setup Complete!")
        return True
    except Exception as e:
        print(f"GPIO Setup Error: {e}")
        return False

def led_control():
    """ควบคุม LED"""
    print("LED Control Test...")
    
    # เปิด LED
    GPIO.output(LED_PIN, GPIO.HIGH)
    print("LED ON")
    time.sleep(1)
    
    # ปิด LED
    GPIO.output(LED_PIN, GPIO.LOW)
    print("LED OFF")
    time.sleep(1)

def button_read():
    """อ่านค่า Button"""
    button_state = GPIO.input(BUTTON_PIN)
    button_state2 = GPIO.input(BUTTON_PIN2)
    
    print(f"Button 1: {'Pressed' if button_state == GPIO.LOW else 'Released'}")
    print(f"Button 2: {'Pressed' if button_state2 == GPIO.LOW else 'Released'}")
    
    return button_state, button_state2

def pwm_control():
    """ควบคุม PWM"""
    print("PWM Control Test...")
    
    # สร้าง PWM object (frequency 1000 Hz)
    pwm = GPIO.PWM(PWM_PIN, 1000)
    
    try:
        # เริ่ม PWM with 0% duty cycle
        pwm.start(0)
        
        # เพิ่ม brightness ทีละน้อย
        for duty_cycle in range(0, 101, 5):
            pwm.ChangeDutyCycle(duty_cycle)
            print(f"PWM Duty Cycle: {duty_cycle}%")
            time.sleep(0.1)
        
        # ลด brightness ทีละน้อย
        for duty_cycle in range(100, -1, -5):
            pwm.ChangeDutyCycle(duty_cycle)
            print(f"PWM Duty Cycle: {duty_cycle}%")
            time.sleep(0.1)
            
    finally:
        pwm.stop()

def interrupt_callback(channel):
    """Callback function สำหรับ interrupt"""
    print(f"Interrupt detected on channel {channel}")
    # Toggle LED เมื่อมี interrupt
    current_state = GPIO.input(LED_PIN2)
    GPIO.output(LED_PIN2, not current_state)

def setup_interrupt():
    """ตั้งค่า interrupt"""
    try:
        # ตั้งค่า interrupt สำหรับ falling edge (กดปุ่ม)
        GPIO.add_event_detect(BUTTON_PIN, GPIO.FALLING, 
                            callback=interrupt_callback, 
                            bouncetime=300)
        print("Interrupt setup complete!")
        return True
    except Exception as e:
        print(f"Interrupt setup error: {e}")
        return False

def main_loop():
    """Main program loop"""
    print("Starting Jetson Nano GPIO Control...")
    
    if not setup_gpio():
        return
    
    # ตั้งค่า interrupt
    setup_interrupt()
    
    print("\nPress Ctrl+C to exit")
    print("Press button to trigger interrupt")
    
    try:
        while True:
            # LED Control
            led_control()
            
            # Button Reading
            button_read()
            
            # PWM Control
            pwm_control()
            
            print("-" * 30)
            time.sleep(2)
            
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    
    except Exception as e:
        print(f"Error in main loop: {e}")
    
    finally:
        # Cleanup GPIO
        GPIO.cleanup()
        print("GPIO Cleanup Complete!")

def simple_examples():
    """ตัวอย่างง่ายๆ สำหรับเริ่มต้น"""
    
    # 1. Digital Output
    def digital_output_example():
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(7, GPIO.OUT)
        
        GPIO.output(7, GPIO.HIGH)  # เปิด
        time.sleep(1)
        GPIO.output(7, GPIO.LOW)   # ปิด
        
        GPIO.cleanup()
    
    # 2. Digital Input
    def digital_input_example():
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(12, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        if GPIO.input(12) == GPIO.LOW:
            print("Button Pressed!")
        else:
            print("Button Released!")
            
        GPIO.cleanup()
    
    # 3. PWM Example
    def pwm_example():
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(33, GPIO.OUT)
        
        pwm = GPIO.PWM(33, 1000)  # 1000 Hz
        pwm.start(50)  # 50% duty cycle
        time.sleep(2)
        pwm.stop()
        
        GPIO.cleanup()

if __name__ == "__main__":
    main_loop()