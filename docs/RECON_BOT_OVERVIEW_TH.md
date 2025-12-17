# สรุปโครงการ Recon Bot - ระบบหุ่นยนต์เคลื่อนที่อัจฉริยะ

**เขียนเมื่อ**: 17 ธันวาคม 2025  
**ผู้ดูแล**: Thanatchai Wongjirad (KMUTT)  
**Repository**: recon_bot (Branch: develop)

---

## 📋 ภาพรวมโครงการ

**Recon Bot** คือ หุ่นยนต์เคลื่อนที่แบบ Mecanum wheel ที่ใช้ ROS 2 ซึ่งออกแบบมาเพื่อ:
- ✅ นำทางอย่างอิสระ (Autonomous Navigation)
- ✅ สร้างแผนที่ (SLAM - Simultaneous Localization and Mapping)
- ✅ สำรวจพื้นที่
- ✅ ประมวลผลสัญญาณจากกล้อง/เซนเซอร์

สิ่งสำคัญ: ใช้ **สถาปัตยกรรมแบบกระจาย** (Distributed Architecture) บน **3 บอร์ด Jetson + 1 Laptop**

---

## 🏗️ สถาปัตยกรรมระบบ

ระบบแบ่งออกเป็น 4 คอมพิวเตอร์ที่ทำงานอย่างอิสระแต่เชื่อมต่อกัน:

| คอมพิวเตอร์ | IP Address | บทบาท | หน้าที่หลัก |
|-----------|-----------|---------|-----------|
| **Jetson #1 (Lower)** | 192.168.2.102 | หัวใจหลัก | ควบคุมเพลาล้อ, Lidar, SLAM, Nav2 |
| **Jetson #2 (Upper)** | 192.168.2.103 | ประมวลผลภาพ | ZED Camera, VIO (Visual Odometry) |
| **Jetson #3 (Manipulator)** | 192.168.2.104 | ควบคุมแขนหุ่น | Manipulator control & perception |
| **Laptop (Local)** | - | ศูนย์ควบคุม | Rviz2 visualization, Remote control |

### การสื่อสาร
- ใช้ **SSH** แบบไม่ต้องใส่รหัสผ่าน (key-based authentication)
- ใช้ **ROS Domain ID = 44** เพื่อให้อุปกรณ์ทั้งหมดสื่อสารกัน
- ตั้ง `ROS_LOCALHOST_ONLY=0` เพื่ออนุญาตการสื่อสารข้างนอก localhost

---

## 📦 โครงสร้างแพ็คเจจหลัก (Packages)

### 1️⃣ **recon_bot_description** - แบบจำลองหุ่นยนต์
```
recon_bot_description/
├── robot/recon_bot.urdf.xacro       # โครงสร้างหุ่นยนต์ (XML format)
├── meshes/                           # ไฟล์ 3D model (.stl, .dae)
├── rviz/recon_botconfig.rviz        # ตั้งค่า Rviz
└── config/                          # ตั้งค่าต่างๆ
```
**ใช้สำหรับ**: กำหนดลักษณะทางเรขาคณิตของหุ่นยนต์ (ตำแหน่งล้อ, กล้อง, เซนเซอร์)

---

### 2️⃣ **recon_bot_bringup** - เรียกใช้ฮาร์ดแวร์
```
recon_bot_bringup/
├── launch/
│   ├── view_robot.launch.py         # Rviz ที่ Laptop
│   ├── rplidar_two.launch.py        # เปิด Lidar
│   └── launch_robot.launch.py       # เปิดฮาร์ดแวร์ทั้งหมด
├── config/
│   ├── twist_mux.yaml               # ตั้งค่ากำหนดเส้นทางคำสั่ง
│   └── ...
└── scripts/
    └── robot_monitor.py             # โปรแกรมตรวจสอบสถานะ
```
**ใช้สำหรับ**: เริ่มต้นฮาร์ดแวร์และควบคุมลำดับการทำงาน

---

### 3️⃣ **recon_bot_mecanum_control** - ควบคุมเพลาล้อ (Mecanum Wheels)
```
recon_bot_mecanum_control/
├── scripts/
│   ├── mecanum_controller.py        # ⭐ ตัวหลักควบคุมมอเตอร์ (520 บรรทัด)
│   ├── a_joy_cmd_vel.py             # อ่านจากจอยสติก → คำสั่ง Twist
│   ├── motor_test_node.py           # ทดสอบมอเตอร์
│   └── test_drive.py                # ทดสอบการเคลื่อนที่
└── launch/
    └── mecanum_control.launch.py    # เปิดระบบควบคุม
```

**Mecanum Controller (mecanum_controller.py) - เครื่องหลัก:**
- รับคำสั่ง `/cmd_vel` (Twist) จาก Nav2 หรือจอยสติก
- คำนวณ **ค่า kinematics** (สูตรคณิตศาสตร์) → แปลงเป็นคำสั่งมอเตอร์ 4 ล้อ
- อ่านตำแหน่งเพลาล้อ (encoder feedback) → ส่งกลับ odometry (ตำแหน่งปัจจุบัน)
- ตัวแปรสำคัญ:
  - `wheel_radius = 0.062 m` - รัศมีล้อ
  - `lx = 0.245 m` - ระยะห่างครึ่งระยะของล้อข้าง
  - `ly = 0.2 m` - ระยะห่างครึ่งระยะของล้อหน้าหลัง
  - มอเตอร์ ID: 1 (RL), 2 (RR), 3 (FR), 4 (FL)
- ความเร็วสูงสุด: 0.4 m/s (เชิงเส้น), 1.2 rad/s (เชิงมุม)

**กลไก Twist Mux**: เลือกคำสั่งอย่างไรตามลำดับความสำคัญ
```
ลำดับความสำคัญ: Joystick > Keyboard > Nav2
```

---

### 4️⃣ **recon_bot_slam** - สร้างแผนที่และระบุตำแหน่ง
```
recon_bot_slam/
├── launch/
│   ├── vio_slam.launch.py           # ⭐ SLAM พร้อม VIO (ใช้บ่อย)
│   ├── mecanum_joy_slam.launch.py   # SLAM + จอยสติก
│   ├── laptop_slam.launch.py        # SLAM ที่ Laptop
│   └── rtabmap_custom.launch.py     # RTABMap backend
├── config/
│   ├── ekf_vio.yaml                 # ⭐ ตั้งค่า EKF (Kalman Filter)
│   ├── slam_params_online_sync.yaml # ตั้งค่า Slam Toolbox
│   └── ...
├── scripts/
│   └── lidar_merge.py               # รวม Lidar scans เป็น point cloud
└── rviz/                            # ตั้งค่า Rviz สำหรับ SLAM
```

**คำศัพท์ที่สำคัญ:**
- **SLAM (Simultaneous Localization and Mapping)**: สร้างแผนที่ + ระบุตำแหน่งหุ่นยนต์พร้อมๆ กัน
- **VIO (Visual Odometry)**: ติดตามการเคลื่อนที่จากกล้อง (ZED)
- **EKF (Extended Kalman Filter)**: ฟิวชั่น (รวม) ข้อมูลจาก:
  - Wheel Odometry (จากเพลาล้อ)
  - Visual Odometry (จากกล้อง)
  - IMU (accelerometer, gyroscope)
  
  → ออกมาเป็น **Odometry ที่แม่นยำ**

---

### 5️⃣ **recon_bot_navigation** - นำทางอัตโนมัติ
```
recon_bot_navigation/
├── launch/
│   ├── vio_navigation.launch.py     # ⭐ นำทางด้วย VIO (ใช้บ่อย)
│   └── nav2_custom.launch.py        # ตั้งค่า Nav2 โดยตรง
├── config/
│   └── nav2_params.yaml             # ตั้งค่า Nav2
├── maps/                            # แผนที่ที่สร้างไว้
├── docs/
│   └── experiment_design.md         # การทดสอบหุ่นยนต์
└── rviz/                            # ตั้งค่า Rviz สำหรับนำทาง
```

**Nav2 (Navigation 2)**: Stack สำหรับนำทางอัตโนมัติ
- จ่ายคำสั่ง `/cmd_vel` ไปยัง Twist Mux
- ใช้แผนที่ + TF tree (ระบบพิกัด) → วางแผนเส้นทาง
- หลีกเลี่ยงสิ่งกีดขวาง (obstacle avoidance)

---

### 6️⃣ **recon_bot_aruco_pose_estimator** - ตรวจจับหมายเหตุ
```
recon_bot_aruco_pose_estimator/
├── scripts/                         # โค้ด detection
├── config/                          # ตั้งค่าขนาด marker
├── launch/                          # เปิดระบบ
└── rviz/                           # แสดงผล
```
**ใช้สำหรับ**: 
- ตรวจจับ ArUco markers (รหัส QR) ในภาพ
- ระบุตำแหน่ง (pose estimation)
- ใช้สำหรับการปรับ coordinate frame

---

### 7️⃣ **แพ็คเจจอื่นๆ**
| Package | ใช้สำหรับ |
|---------|---------|
| `recon_bot_test` | ทดสอบ และ validation |
| `recon_bot_commnunication` | สื่อสารระหว่าง robots (WIP) |
| `microRos_bot` | Micro-ROS สำหรับ embedded |
| `dynamixel_hardware` | Interface กับ Dynamixel servos |
| `zed-ros2-wrapper` | ZED camera driver |
| `apriltag`, `apriltag_ros` | AprilTag detection |
| `rplidar_ros` | RPLidar driver |

---

## 🔄 ขั้นตอนการทำงาน (Data Flow)

```
┌─────────────────────────────────────────────────────────────┐
│                    INPUT (ข้อมูลเข้า)                        │
└─────────────────────────────────────────────────────────────┘
                           ↓
        ┌────────────────┬─────────────┬─────────────┐
        ↓                ↓             ↓             ↓
   RPLidar 2D        ZED Camera      Dynamixel    IMU (ZED)
   Laser Scans       RGB-D frames    Wheel Encoder
                     
        ↓                ↓             ↓             ↓
   ┌────────────────────────────────────────────────┐
   │      SENSOR FUSION (EKF Node)                  │
   │  Lidar Odometry + VIO + IMU                   │
   │              ↓↓↓                              │
   │  Output: Filtered Odometry (/odom)            │
   └────────────────────────────────────────────────┘
                        ↓
   ┌────────────────────────────────────────────────┐
   │  PROCESSING LAYER (Jetson #1)                 │
   │  ┌──────────────────────────────────────────┐ │
   │  │ SLAM (Slam Toolbox + Lidar)             │ │
   │  │ → Map + Robot Pose in Map                │ │
   │  └──────────────────────────────────────────┘ │
   │  ┌──────────────────────────────────────────┐ │
   │  │ NAVIGATION (Nav2)                        │ │
   │  │ → Goal + Map → Path Planning             │ │
   │  │ → Output: /cmd_vel                       │ │
   │  └──────────────────────────────────────────┘ │
   └────────────────────────────────────────────────┘
                        ↓
   ┌────────────────────────────────────────────────┐
   │  CONTROL ARBITRATION (Twist Mux)              │
   │  Joystick /cmd_vel (priority 1)              │
   │  Keyboard  /cmd_vel (priority 2)             │
   │  Nav2      /cmd_vel (priority 3)             │
   │                                               │
   │  Output: /cmd_vel_mux (คำสั่งที่เลือก)      │
   └────────────────────────────────────────────────┘
                        ↓
   ┌────────────────────────────────────────────────┐
   │  MOTOR CONTROL (Mecanum Controller)            │
   │  /cmd_vel_mux → Kinematics → 4 Motor Commands │
   │  ω_FL, ω_FR, ω_RL, ω_RR                      │
   └────────────────────────────────────────────────┘
                        ↓
   ┌────────────────────────────────────────────────┐
   │  ACTUATORS (Dynamixel Motors)                 │
   │  ล้อหน้าซ้าย, หน้าขวา, หลังซ้าย, หลังขวา      │
   └────────────────────────────────────────────────┘
                        ↓
   ┌────────────────────────────────────────────────┐
   │  OUTPUT (Laptop - Rviz2 Visualization)        │
   │  - 3D Model ของหุ่นยนต์                       │
   │  - Lidar Point Cloud                         │
   │  - Planned Path                              │
   │  - Robot Pose                                │
   │  - Camera Feeds                              │
   └────────────────────────────────────────────────┘
```

---

## 🎮 โหมดการใช้งานหลัก

### โหมดที่ 1: **Mapping Mode** (สร้างแผนที่)
**สถานการณ์**: วันแรกเข้าสถานที่ใหม่ต้องสร้างแผนที่

```bash
# ที่ Jetson #1
ros2 launch recon_bot_slam vio_slam.launch.py
```

**ทำงาน**:
1. เปิด Lidar, ZED Camera, Motor control
2. Slam Toolbox เริ่มสร้างแผนที่
3. ควบคุมหุ่นยนต์ด้วย Joystick เพื่อสำรวจ
4. → ออกมาเป็นไฟล์แผนที่ (`.pgm` + `.yaml`)

---

### โหมดที่ 2: **Navigation Mode** (นำทางอัตโนมัติ)
**สถานการณ์**: มีแผนที่แล้ว ต้องให้หุ่นยนต์ไปยังจุดหมายอัตโนมัติ

```bash
# ที่ Jetson #1
ros2 launch recon_bot_navigation vio_navigation.launch.py
```

**ทำงาน**:
1. เปิด Lidar, ZED Camera, Motor control
2. EKF + Nav2 เริ่มทำงาน
3. โหลดแผนที่ที่บันทึกไว้
4. Localization: หาตำแหน่งของหุ่นยนต์บนแผนที่
5. รอคำสั่งจาก Rviz ที่ Laptop (ตั้งจุด goal)
6. → Nav2 วางแผน path + หุ่นยนต์เคลื่อนที่อย่างอิสระ

---

### โหมดที่ 3: **Visualization Only** (ดูผลเท่านั้น)
**ที่ Laptop**:

```bash
# ที่ Laptop (ที่บ้าน/สำนักงาน)
ros2 launch recon_bot_bringup view_robot.launch.py
```

**ทำงาน**:
- เปิด Rviz2 เท่านั้น
- ไม่ต้องฮาร์ดแวร์
- มองเห็นหุ่นยนต์ทำงานแบบ real-time

---

## 🛠️ เซ็นเซอร์และอุปกรณ์

| ส่วนประกอบ | รุ่น/ชนิด | ที่ตั้ง | ข้อมูลที่ส่ง |
|-----------|----------|-------|-----------|
| **Lidar** | RPLidar A2 | ด้านบนหุ่น | Laser scans (2D) |
| **กล้อง** | Stereolabs ZED2 | Jetson #2 | RGB-D, depth, IMU, VIO |
| **มอเตอร์** | Dynamixel (4 ตัว) | 4 มุม | Wheel position/velocity |
| **Joystick** | Standard USB | Laptop/PC | Control input |

---

## ⚙️ ตัวแปร Parameters สำคัญ

### Mecanum Controller Parameters
```yaml
wheel_radius: 0.062 m           # รัศมีล้อ
lx: 0.245 m                     # ระยะห่างครึ่ง (x-axis)
ly: 0.2 m                       # ระยะห่างครึ่ง (y-axis)
max_linear_speed: 0.4 m/s       # ความเร็วเชิงเส้นสูงสุด
max_angular_speed: 1.2 rad/s    # ความเร็วเชิงมุมสูงสุด
cmd_vel_timeout: 1.0 s          # หยุดการเคลื่อนที่หากไม่มีคำสั่งเกิน 1 วิ
```

### Motor IDs
```
ID 1: Right Rear (RR) - หลังขวา
ID 2: Right Left (RL) - หลังซ้าย
ID 3: Front Right (FR) - หน้าขวา
ID 4: Front Left (FL) - หน้าซ้าย
```

### ROS 2 Configuration
```bash
ROS_DOMAIN_ID=44                # ID ของ network (ทุก Jetson ต้องเหมือน)
ROS_LOCALHOST_ONLY=0            # อนุญาต remote communication
ROS_DISTRO=foxy หรือ humble     # เวอร์ชัน ROS
```

---

## 🧪 การทดสอบ

**Experiment Design**: `/src/recon_bot_navigation/docs/experiment_design.md`

ทดสอบการเคลื่อนที่:
1. **Strafing Test**: เคลื่อนข้าง (ทดสอบ Mecanum)
2. **Navigation Test**: เคลื่อนไปยังจุดต่างๆ อัตโนมัติ
3. **SLAM Test**: สร้างแผนที่และระบิตำแหน่ง

---

## 📝 Troubleshooting

### ปัญหา: หุ่นยนต์ไม่ขยับ
- ตรวจสอบว่า `/cmd_vel_mux` มี data
- ตรวจสอบ Motor IDs ตรงหรือไม่
- ดู logs ของ `mecanum_controller.py`

### ปัญหา: แผนที่ไม่ถูกต้อง
- ขับเคลื่อนช้าลง (SLAM ต้องเวลา)
- ลองใหม่ใน mapping mode
- ตรวจสอบ Lidar ว่าอยู่ตำแหน่งสูงเหมาะสม

### ปัญหา: Jetson เชื่อมต่อไม่ได้
- ตรวจสอบ IP addresses
- ทดสอบ SSH: `ssh user@ip`
- ตรวจสอบ ROS_DOMAIN_ID ตรงกันหรือไม่

---

## 🚀 ชุดคำสั่งสำคัญ

```bash
# Build ทั้งระบบ
cd ~/Ros2_Directory/recon_ws
colcon build --symlink-install

# Source setup
source install/setup.bash

# ตรวจสอบ ROS nodes
ros2 node list

# ตรวจสอบ topics
ros2 topic list

# ดู message content (real-time)
ros2 topic echo /cmd_vel

# เรียกใช้ launch file
ros2 launch recon_bot_slam vio_slam.launch.py
ros2 launch recon_bot_navigation vio_navigation.launch.py
ros2 launch recon_bot_bringup view_robot.launch.py

# Kill process
pkill -f ros2
```

---

## 📞 สรุปสำหรับ Antigravity (AI Assistant)

**เมื่อผู้ใช้ (Luke) มาถามเกี่ยวกับโครงการนี้:**

1. ✅ โครงการนี้คือระบบหุ่นยนต์ Mecanum wheel บน ROS 2
2. ✅ มี 3 Jetson + 1 Laptop ทำงานแบบกระจาย
3. ✅ ใช้ SLAM สร้างแผนที่ และ Nav2 สำหรับนำทางอัตโนมัติ
4. ✅ ใช้ EKF รวมข้อมูลจาก Lidar, ZED Camera, Wheel Encoders, IMU
5. ✅ Mecanum Controller คำนวณ kinematics + ควบคุม 4 มอเตอร์
6. ✅ ใช้ Twist Mux เลือกคำสั่ง (Joystick > Keyboard > Nav2)
7. ✅ ตอบผู้ใช้เป็นภาษาไทยเสมอ
8. ✅ หากมีคำถามเกี่ยวกับส่วนใหม่ ให้ดูในแพ็คเจจที่เกี่ยวข้อง

---

**สร้างโดย**: Antigravity (AI Assistant)  
**วันที่**: 17 ธันวาคม 2025  
**สถานะ**: ✅ พร้อมใช้งาน
