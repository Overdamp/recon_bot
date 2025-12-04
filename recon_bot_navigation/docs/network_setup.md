# Network Setup Guide for Distributed ROS 2

This guide details how to configure the network for the Recon Bot system, ensuring seamless communication between the two Jetson boards (LAN) and the Laptop (Wi-Fi).

## 1. Network Topology
*   **Router**: T3 Mesh A623 (Main Gateway & DHCP Server)
*   **Jetson #1 (Robot Core)**: Connected via **Ethernet (LAN)**
*   **Jetson #2 (Vision)**: Connected via **Ethernet (LAN)**
*   **Laptop (Control)**: Connected via **Wi-Fi (5GHz recommended)**

## 2. Router Configuration (T3 Mesh)
1.  **Connect**: Ensure all devices are connected to the T3 Mesh.
2.  **Check Isolation**: Log in to the router's admin page (usually `192.168.1.1` or similar).
    *   Ensure **"AP Isolation"** or **"Client Isolation"** is **DISABLED**. (This feature blocks Wi-Fi devices from talking to LAN devices).
3.  **DHCP Reservation (Recommended)**:
    *   In the router settings, find "DHCP Static Lease" or "Address Reservation".
    *   Fix the IP addresses for your Jetsons so they never change.
    *   *Example Plan*:
        *   **Jetson #1**: `192.168.1.101`
        *   **Jetson #2**: `192.168.1.102`
        *   **Laptop**: `192.168.1.103` (or let it be dynamic)

## 3. Device Configuration (On Each Machine)

### Step 3.1: Set Static IP (If not done on Router)
If you can't access router settings, set Static IP on Ubuntu:
1.  Open **Settings** -> **Network**.
2.  Click the **Gear icon** next to Wired (Jetson) or Wi-Fi (Laptop).
3.  Go to **IPv4** tab.
4.  Method: **Manual**.
5.  **Address**: `192.168.1.101` (Change for each device).
6.  **Netmask**: `255.255.255.0`.
7.  **Gateway**: `192.168.1.1` (Router IP).
8.  **DNS**: `8.8.8.8` (Google) or `192.168.1.1`.

### Step 3.2: Map Hostnames (/etc/hosts)
This allows you to ping by name (e.g., `ping jetson1`) instead of remembering IPs.
**Run this on ALL 3 devices (Jetson #1, #2, Laptop):**

1.  Edit the hosts file:
    ```bash
    sudo nano /etc/hosts
    ```
2.  Add these lines at the bottom (Use your ACTUAL IPs):
    ```text
    192.168.1.101   jetson1
    192.168.1.102   jetson2
    192.168.1.103   laptop
    ```
3.  Save and Exit (`Ctrl+O`, `Enter`, `Ctrl+X`).

### Step 3.3: Configure ROS 2 Environment
**Run this on ALL 3 devices:**

1.  Open `.bashrc`:
    ```bash
    nano ~/.bashrc
    ```
2.  Add these lines at the very bottom:
    ```bash
    # ROS 2 Network Config
    export ROS_DOMAIN_ID=44           # MUST be the same on all devices
    export ROS_LOCALHOST_ONLY=0       # Allow communication over network
    
    # [FIX] Force ROS to use the correct Network Interface
    # Replace with the ACTUAL IP of this machine (e.g., 10.61.2.21)
    export ROS_IP=10.61.2.XX 
    ```
3.  Save and Exit.
4.  Apply changes:
    ```bash
    source ~/.bashrc
    ```

### Important: Why ROS_IP?
If your Jetson has multiple interfaces (like `docker0`, `usb0`, `wlan0`), ROS 2 might try to send data via the wrong one (e.g., Docker IP) which the Laptop can't reach. Setting `ROS_IP` forces it to use the correct LAN IP.

## 4. Verification

### Test 1: Ping
From Laptop, try to ping the Jetsons:
```bash
ping jetson1
ping jetson2
```
*   **Success**: You see `64 bytes from ...`
*   **Fail**: Check IP addresses and Router Isolation settings.

### Test 2: ROS 2 Topic
1.  **On Jetson #1**: Run a dummy talker.
    ```bash
    ros2 run demo_nodes_cpp talker
    ```
2.  **On Laptop**: Check if you can see it.
    ```bash
    ros2 topic list
    ros2 topic echo /chatter
    ```
*   **Success**: You see the topic and data flowing.
*   **Fail**: Check `ROS_DOMAIN_ID` and Firewall.

## 5. Troubleshooting (Firewall)
If Ping works but ROS 2 doesn't, Ubuntu's firewall might be blocking ports.
**Run on all devices:**
```bash
sudo ufw allow from 192.168.1.0/24  # Allow all traffic from local network
# OR disable it temporarily to test
sudo ufw disable
```
