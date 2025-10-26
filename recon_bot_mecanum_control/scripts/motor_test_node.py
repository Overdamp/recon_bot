#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState # <--- เพิ่ม import นี้
import time
import threading # <--- เพิ่ม import นี้
from rclpy.time import Time # <--- เพิ่ม import นี้

class MotorTester(Node):
    def __init__(self):
        super().__init__('motor_tester')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- เพิ่ม Subscriber สำหรับ /joint_states ---
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10 # QoS profile depth
        )
        self.latest_joint_state = None # ตัวแปรเก็บข้อมูล joint_states ล่าสุด
        self.joint_state_lock = threading.Lock() # ป้องกัน Race Condition
        # -------------------------------------------

        # --- ค่าที่ปรับใหม่ ---
        self.move_speed = 0.05      # ความเร็วเล็กน้อย (m/s)
        self.move_duration = 4.0    # ระยะเวลาที่เคลื่อนที่ (วินาที) <-- ปรับ
        self.stop_duration = 30.0    # ระยะเวลาที่หยุดนิ่ง (วินาที) <-- ปรับ
        self.log_interval = 0.2     # ช่วงเวลาในการ log ตอนเคลื่อนที่ (วินาที)
        # --- จบการปรับค่า ---

        self.get_logger().info("Starting motor test sequence (Move 4s, Stop 2s)...")

        # ใช้ Timer เรียก run_test_step แทน loop ยาวๆ เพื่อให้ Callback ทำงานได้
        self.test_step = 0
        self.moves = [
            ("Forward", self.move_speed, 0.0, 0.0),
            ("Stop1", 0.0, 0.0, 0.0),
            ("Backward", -self.move_speed, 0.0, 0.0),
            ("Stop2", 0.0, 0.0, 0.0),
            ("Strafe Left", 0.0, self.move_speed, 0.0),
            ("Stop3", 0.0, 0.0, 0.0),
            ("Strafe Right", 0.0, -self.move_speed, 0.0),
            ("Stop4", 0.0, 0.0, 0.0)
        ]
        self.timer = self.create_timer(0.05, self.run_test_step) # เรียกถี่ขึ้นเล็กน้อย (ทุก 0.05 วิ)
        self.step_start_time = self.get_clock().now()
        self.last_log_time = Time(seconds=0, nanoseconds=0, clock_type=self.get_clock().clock_type) # ตัวแปรเก็บเวลา log ล่าสุด
        self.logged_this_stop_step = False # Flag เพื่อ log แค่ครั้งเดียวตอนหยุด <-- เพิ่ม

    def joint_state_callback(self, msg: JointState):
        """Callback เมื่อได้รับข้อมูล /joint_states"""
        with self.joint_state_lock:
            self.latest_joint_state = msg

    def send_cmd_vel(self, linear_x, linear_y, angular_z):
        """ส่งคำสั่ง Twist ไปยัง /cmd_vel"""
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.linear.y = float(linear_y)
        msg.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(msg)
        # Log การส่งคำสั่งใน run_test_step

    def log_joint_state(self, prefix=""):
        """ฟังก์ชันช่วย Log ข้อมูล JointState ล่าสุด"""
        with self.joint_state_lock:
            if self.latest_joint_state:
                log_str = f"{prefix} Joint States: "
                try:
                    # สร้าง dictionary {name: velocity} เพื่อให้อ่านง่าย
                    vel_dict = {name: vel for name, vel in zip(self.latest_joint_state.name, self.latest_joint_state.velocity)}
                    log_str += str(vel_dict)
                except Exception as e:
                    log_str += f"(Error formatting: {e})"
                self.get_logger().info(log_str)
            else:
                self.get_logger().warn(f"{prefix} No JointState received yet.")

    def run_test_step(self):
        """ทำงานทีละ Step ของการทดสอบ (ถูกเรียกโดย Timer)"""
        current_time = self.get_clock().now()
        elapsed_time_sec = (current_time - self.step_start_time).nanoseconds * 1e-9

        step_index = self.test_step % len(self.moves)
        name, lx, ly, az = self.moves[step_index]
        is_moving_step = not name.startswith("Stop")
        current_duration = self.move_duration if is_moving_step else self.stop_duration

        # --- ส่วน Logic การทำงาน ---
        if elapsed_time_sec < current_duration:
            # ยังอยู่ใน Step ปัจจุบัน
            if is_moving_step:
                self.send_cmd_vel(lx, ly, az)
                # Log การส่งคำสั่ง (อาจจะลดความถี่ลงถ้าต้องการ)
                if int(elapsed_time_sec * 10) % 5 == 0: # Log ทุก 0.5 วิ
                    self.get_logger().info(f"--> Moving {name} (Sent: x={lx}, y={ly}, z={az})")

                # --- แก้ไขการ LOG ตอนเคลื่อนที่ ---
                # Log joint states ทุกๆ log_interval (0.2 วินาที) ขณะเคลื่อนที่
                time_since_last_log_sec = (current_time - self.last_log_time).nanoseconds * 1e-9
                if time_since_last_log_sec >= self.log_interval:
                    self.log_joint_state(f"    (T+{elapsed_time_sec:.1f}s)")
                    self.last_log_time = current_time # อัปเดตเวลา log ล่าสุด
                # --- จบการแก้ไข ---

            else:
                # กำลังหยุดนิ่ง (ส่ง 0.0 ค้างไว้)
                self.send_cmd_vel(0.0, 0.0, 0.0)
                # Log การส่งคำสั่งแค่ครั้งเดียวตอนเริ่มหยุด
                if elapsed_time_sec < 0.1: # Log แค่ช่วง 0.1 วิแรก
                     self.get_logger().info(f"--> Stopping ({name}) (Sent: x=0, y=0, z=0)")

                # --- แก้ไขการ LOG ตอนหยุด ---
                # Log joint state ครั้งเดียวตอนเริ่มหยุด
                if not self.logged_this_stop_step:
                    self.log_joint_state("    (Stop Start)")
                    self.logged_this_stop_step = True # ตั้งค่า flag เพื่อไม่ให้ log ซ้ำ
                # --- จบการแก้ไข ---
        else:
            # หมดเวลาของ Step นี้แล้ว -> ไป Step ถัดไป
            self.get_logger().info(f"--- Finished {name} ---")

            # เปลี่ยนไป Step ถัดไป
            self.test_step += 1
            self.step_start_time = self.get_clock().now() # รีเซ็ตเวลาเริ่ม Step ใหม่
            self.last_log_time = Time(seconds=0, nanoseconds=0, clock_type=self.get_clock().clock_type) # รีเซ็ตเวลา log สำหรับ step ใหม่
            self.logged_this_stop_step = False # รีเซ็ต flag การ log ตอนหยุด <-- เพิ่ม

            # Log การเริ่ม Step ใหม่ (ถ้ามี)
            next_step_index = self.test_step % len(self.moves)
            next_name, _, _, _ = self.moves[next_step_index]
            self.get_logger().info(f"--- Starting {next_name} ---")


def main(args=None):
    rclpy.init(args=args)
    node = MotorTester()
    try:
        rclpy.spin(node) # ใช้ spin() เพื่อให้ Callback ทำงานได้
    except KeyboardInterrupt:
        node.get_logger().info("Test stopped by user.")
    finally:
        # สั่งหยุดครั้งสุดท้ายก่อนปิด
        temp_node = rclpy.create_node('final_stop_publisher')
        temp_pub = temp_node.create_publisher(Twist, '/cmd_vel', 1)
        stop_msg = Twist()
        temp_pub.publish(stop_msg)
        temp_node.get_logger().info("Final stop command sent via temporary node.")
        time.sleep(0.5) # รอให้ msg ส่งไปถึง
        temp_node.destroy_node()

        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()