# Recon Bot - Mecanum Wheel Mobile Robot

**Recon Bot** is a ROS 2-based mobile robot featuring **Mecanum wheels** for holonomic (omni-directional) motion. It is designed for autonomous navigation, mapping (SLAM), and exploration using a distributed computing architecture.

## 🚀 Features
*   **Holonomic Motion**: Capable of moving forward, sideways (strafing), and rotating simultaneously.
*   **Distributed Architecture**:
    *   **Jetson #1**: Robot Core (Control, EKF, SLAM, Lidar).
    *   **Jetson #2**: Vision Processing (ZED Camera, VIO).
    *   **Laptop**: Visualization (Rviz) and Command Center.
*   **Sensor Fusion**: Uses **Extended Kalman Filter (EKF)** to fuse Wheel Odometry, IMU, and Visual Odometry (VIO) for robust localization.
*   **Advanced Navigation**: Powered by **Nav2** with custom parameters optimized for mecanum wheels.
*   **Safety**: Integrated **Twist Mux** for priority-based control (Joystick > Keyboard > Nav2).

## 🛠️ Hardware Setup
*   **Robot Base**: Custom Mecanum Wheel Chassis.
*   **Computers**: 2x NVIDIA Jetson (e.g., Orin Nano/NX), 1x Laptop.
*   **Sensors**:
    *   RPLidar (2D Lidar).
    *   Stereolabs ZED 2 Camera (Depth + VIO).
    *   IMU (Built-in ZED).
*   **Actuators**: Dynamixel Servos (or compatible DC motors with encoders).

## 📦 Installation
1.  **Clone the repository**:
    ```bash
    cd ~/Ros2_Directory/recon_ws/src
    git clone <your-repo-url> .
    ```
2.  **Install dependencies**:
    ```bash
    cd ~/Ros2_Directory/recon_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```
3.  **Build**:
    ```bash
    colcon build --symlink-install
    source install/setup.bash
    ```

## 🚦 Usage

### 1. Mapping Mode (Create a Map)
Run this on **Jetson #1** to start SLAM and create a new map.
```bash
ros2 launch recon_bot_slam vio_slam.launch.py
```
*   **Note**: Ensure `slam_toolbox` is uncommented in the launch file if it was disabled.

### 2. Navigation Mode (Autonomous)
Run this on **Jetson #1** to start EKF, Hardware, and Nav2 with an existing map.
```bash
ros2 launch recon_bot_navigation vio_navigation.launch.py
```

### 3. Vision Node
Run this on **Jetson #2** to handle camera and VIO.
```bash
ros2 launch zed_wrapper vio_zed_camera.launch.py
```

### 4. Visualization & Control
Run this on your **Laptop** to see what the robot sees.
```bash
ros2 launch recon_bot_bringup view_robot.launch.py
```

## 🧪 Experiment & Validation
We have designed a standard experimental procedure to validate the robot's holonomic capabilities.
*   See [Experiment Design](recon_bot_navigation/docs/experiment_design.md) for details on Strafing and Navigation tests.

## 📂 Package Structure
*   `recon_bot_bringup`: Launch files and configs for hardware.
*   `recon_bot_description`: URDF/Xacro robot models.
*   `recon_bot_mecanum_control`: Kinematics and motor controllers.
*   `recon_bot_navigation`: Nav2 configuration and launch files.
*   `recon_bot_slam`: SLAM and EKF configurations.
*   `recon_bot_aruco_pose_estimator`: Marker detection (Optional).

## 📝 License
[Your License Here]
