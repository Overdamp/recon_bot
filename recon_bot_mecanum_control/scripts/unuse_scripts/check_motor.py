#!/usr/bin/env python3
from dynamixel_sdk import *

# แก้ตรงนี้ถ้าใช้พอร์ตอื่น
DEVICENAME = '/dev/ttyUSB_DYNAMIXEL'
BAUDRATE = 57600  # หรือ 57600, 115200 แล้วแต่มอเตอร์
PROTOCOL_VERSION = 2.0

def main():
    portHandler = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    if not portHandler.openPort():
        print("❌ Failed to open port")
        return

    if not portHandler.setBaudRate(BAUDRATE):
        print("❌ Failed to set baudrate")
        return

    print(f"✅ Scanning motors on {DEVICENAME}...")

    for dxl_id in range(1, 253):  # Dynamixel ID range 1-252
        dxl_model_number, dxl_comm_result, dxl_error = packetHandler.ping(portHandler, dxl_id)

        if dxl_comm_result == COMM_SUCCESS:
            if dxl_error != 0:
                print(f"[ID:{dxl_id}] ⚠️ Error occurred: {packetHandler.getRxPacketError(dxl_error)}")
            else:
                print(f"[ID:{dxl_id}] 🎯 Motor detected, Model Number: {dxl_model_number}")
        elif dxl_comm_result != COMM_TX_FAIL:
            print(f"[ID:{dxl_id}] ❌ {packetHandler.getTxRxResult(dxl_comm_result)}")

    portHandler.closePort()

if __name__ == "__main__":
    main()
