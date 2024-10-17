#!/usr/bin/python3.
import rclpy
from rclpy.node import Node
import numpy

import os
# from motor_start import *

class basic_movement(Node):
    def __init__(self):
        super().__init__('basic_movement_node')
        print('basic_movement_node')
        Lx = 0.198 # m
        Ly = 0.2043
        self.eqm = numpy.array([[1, -1, -(Lx+Ly)],[1,  1, (Lx+Ly)], [1,  1, -(Lx+Ly)], [1, -1, (Lx+Ly)]])
        self.r = 0.127/2
        self.Vx = float()
        self.Vy = float()
        self.Wz = float()
        self.A0 = float()
        self.A1 = float()
        self.A2 = float()
        self.A3 = float()
        self.robot_vel = 0
        self.a = 0
        self.movevel = 0
        self.robot_vel = numpy.array([[self.Vx], [self.Vy], [self.Wz]])
        self.base_vel = ((1/self.r)*self.eqm).dot((self.robot_vel))

    def vel(self):
        self.robot_vel = numpy.array([[self.Vx], [self.Vy], [self.Wz]])
        self.base_vel = ((1/self.r)*self.eqm).dot((self.robot_vel))

    def motor_start(self):
        print('motor_start')
        if os.name == 'nt':
            import msvcrt
            def getch():
                return msvcrt.getch().decode()
        else:
            import sys, tty, termios
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            def getch():
                try:
                    tty.setraw(sys.stdin.fileno())
                    ch = sys.stdin.read(1)
                finally:
                    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                return ch
            
        # DXL_MINIMUM_POSITION_VALUE  = 100           # Dynamixel will rotate between this value
        # DXL_MAXIMUM_POSITION_VALUE  = 4000 
        # dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE] 

        DXL_MINIMUM_SPEED_VALUE = 0       # Goal position
        DXL_MAXIMUM_SPEED_VALUE = 1023
        dxl_goal_speed = [DXL_MINIMUM_SPEED_VALUE ,DXL_MAXIMUM_SPEED_VALUE]

    def forward(self):
        print('forward')
        self.A0 = abs(int(self.base_vel[0])-1024)
        self.A1 = abs(int(self.base_vel[1]))
        self.A2 = abs(int(self.base_vel[2])-1024)
        self.A3 = abs(int(self.base_vel[3]))
        self.base_vel = (self.A0,self.A1,self.A2,self.A3)

    def backward(self):
        print('backward')
        self.A0 = abs(int(self.base_vel[0]))
        self.A1 = abs(int(self.base_vel[1])+1024)
        self.A2 = abs(int(self.base_vel[2]))
        self.A3 = abs(int(self.base_vel[3])+1024)
        self.base_vel = (self.A0,self.A1,self.A2,self.A3)

    def left(self):
        print('left')
        self.A0 = abs(int(self.base_vel[0]))
        self.A1 = abs(int(self.base_vel[1]))
        self.A2 = abs(int(self.base_vel[2])-1024)
        self.A3 = abs(int(self.base_vel[3])+1024)
        self.base_vel = (self.A0,self.A1,self.A2,self.A3)

    def right(self):
        print('right')
        self.A0 = abs(int(self.base_vel[0])-1024)
        self.A1 = abs(int(self.base_vel[1])+1024)
        self.A2 = abs(int(self.base_vel[2]))
        self.A3 = abs(int(self.base_vel[3]))
        self.base_vel = (self.A0,self.A1,self.A2,self.A3)

    def rotate_right(self): #All CCW
        print('rotate_right')
        self.A0 = abs(int(self.base_vel[0]))
        self.A1 = abs(int(self.base_vel[1]))
        self.A2 = abs(int(self.base_vel[2]))
        self.A3 = abs(int(self.base_vel[3]))
        self.base_vel = (self.A0,self.A1,self.A2,self.A3)

    def rotate_left(self):
        print('rotate_left')
        self.A0 = abs(int(self.base_vel[0])+1024)
        self.A1 = abs(int(self.base_vel[1])-1024)
        self.A2 = abs(int(self.base_vel[2])+1024)
        self.A3 = abs(int(self.base_vel[3])-1024)
        self.base_vel = (self.A0,self.A1,self.A2,self.A3)
        
    def RF(self):
        print('RF')
        self.A0 = abs(int(self.base_vel[0]))
        self.A1 = 0
        self.A2 = 0
        self.A3 = abs(int(self.base_vel[3])-1024)
        self.base_vel = (self.A0,self.A1,self.A2,self.A3)

    def RB(self):
        print('RB')
        self.A0 = 0
        self.A1 = abs(int(self.base_vel[1]))
        self.A2 = abs(int(self.base_vel[2]))
        self.A3 = 0
        self.base_vel = (self.A0,self.A1,self.A2,self.A3)

    def LF(self):
        print('LF')
        self.A0 = 0
        self.A1 = abs(int(self.base_vel[1]))
        self.A2 = abs(int(self.base_vel[2])-1024)
        self.A3 = 0
        self.base_vel = (self.A0,self.A1,self.A2,self.A3)

    def LB(self):
        print('LB')
        self.A0 = self.A0 = abs(int(self.base_vel[0]))
        self.A1 = 0
        self.A2 = 0
        self.A3 = abs(int(self.base_vel[0])+1024)
        self.base_vel = (self.A0,self.A1,self.A2,self.A3)

    def stop(self):
        print('stop')
        self.A0 = 0
        self.A1 = 0
        self.A2 = 0
        self.A3 = 0
        self.base_vel = (self.A0,self.A1,self.A2,self.A3)