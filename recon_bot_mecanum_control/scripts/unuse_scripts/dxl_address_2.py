#!/usr/bin/env python3
# dxl_address.py - For Dynamixel MX-106 with Protocol 2.0 (Velocity Control)

# ------------------------------
# Device and Protocol Settings
# ------------------------------
PROTOCOL_VERSION               = 2.0
BAUDRATE                       = 1000000
DEVICENAME                     = '/dev/ttyUSB_DYNAMIXEL'  # USB port (udev recommended)

# ------------------------------
# Dynamixel IDs (example setup)
# ------------------------------
DXL1_ID                        = 1   # Rear Right
DXL2_ID                        = 2   # Rear Left
DXL3_ID                        = 3   # Front Right
DXL4_ID                        = 4   # Front Left

# ------------------------------
# Torque Control
# ------------------------------
TORQUE_ENABLE                  = 1
TORQUE_DISABLE                 = 0

# ------------------------------
# Control Table Addresses (Protocol 2.0)
# ------------------------------
ADDR_MX_TORQUE_ENABLE          = 64
ADDR_MX_LED                    = 65
ADDR_MX_GOAL_VELOCITY          = 104
ADDR_MX_PRESENT_VELOCITY       = 128
ADDR_MX_CW_ANGLE_LIMIT         = 6
ADDR_MX_CCW_ANGLE_LIMIT        = 8
ADDR_MX_OPERATING_MODE         = 11
ADDR_MX_PROFILE_ACCELERATION   = 108
ADDR_MX_PROFILE_VELOCITY       = 112
ADDR_MX_POSITION_P_GAIN        = 84
ADDR_MX_POSITION_I_GAIN        = 82
ADDR_MX_POSITION_D_GAIN        = 80
ADDR_MX_VELOCITY_I_GAIN        = 76
ADDR_MX_VELOCITY_P_GAIN        = 78
ADDR_MX_PRESENT_POSITION       = 132
ADDR_MX_GOAL_POSITION          = 116
ADDR_MX_PRESENT_CURRENT        = 126
ADDR_MX_PRESENT_INPUT_VOLTAGE  = 144
ADDR_MX_PRESENT_TEMPERATURE    = 146

# ------------------------------
# Data Byte Length
# ------------------------------
LEN_MX_GOAL_VELOCITY           = 4
LEN_MX_PRESENT_VELOCITY        = 4
LEN_MX_PRESENT_POSITION        = 4
LEN_MX_GOAL_POSITION           = 4
LEN_MX_PRESENT_CURRENT         = 2
LEN_MX_PRESENT_VOLTAGE         = 2
LEN_MX_PRESENT_TEMPERATURE     = 1

# ------------------------------
# Miscellaneous
# ------------------------------
DXL_MOVING_STATUS_THRESHOLD    = 20   # Optional, not always used
DEBUG                          = True
