#!/usr/bin/env python3

from dynamixel_sdk import *  # ใช้ SDK จาก Robotis
import time

# --- พารามิเตอร์พื้นฐาน ---
DEVICENAME = '/dev/ttyUSB_DYNAMIXEL'
BAUDRATE = 1000000
PROTOCOL_VERSION = 2.0

ADDR_PRESENT_VELOCITY = 128
LEN_PRESENT_VELOCITY = 4

DXL_IDS = [1, 2, 3, 4]

# --- สร้าง handler ---
port_handler = PortHandler(DEVICENAME)
packet_handler = PacketHandler(PROTOCOL_VERSION)
group_sync_read = GroupSyncRead(port_handler, packet_handler, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY)

# --- เปิดพอร์ตและตั้งค่า baudrate ---
if not port_handler.openPort():
    print("❌ Failed to open port")
    quit()

if not port_handler.setBaudRate(BAUDRATE):
    print("❌ Failed to set baudrate")
    quit()

print("✅ Port opened and baudrate set.")

# --- เพิ่ม ID ทั้งหมดใน group sync read ---
for dxl_id in DXL_IDS:
    if not group_sync_read.addParam(dxl_id):
        print(f"❌ Failed to add ID {dxl_id} to GroupSyncRead")
        quit()

# --- อ่านค่า ---
if group_sync_read.txRxPacket() != COMM_SUCCESS:
    print(f"❌ GroupSyncRead failed: {packet_handler.getTxRxResult(group_sync_read.txRxPacket())}")
    quit()

# --- ดึงข้อมูลจากแต่ละ ID ---
for dxl_id in DXL_IDS:
    if group_sync_read.isAvailable(dxl_id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY):
        velocity = group_sync_read.getData(dxl_id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY)

        # แปลง signed 32-bit
        if velocity > (1 << 31):
            velocity -= (1 << 32)

        print(f"[ID:{dxl_id}] ✅ Present Velocity: {velocity}")
    else:
        print(f"[ID:{dxl_id}] ⚠️ Data not available")

# --- ปิดพอร์ต ---
port_handler.closePort()
