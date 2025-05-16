#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import numpy
import time
import math

from sensor_msgs.msg import Joy

from a_sdk import basic_movement
from a_sdk import motor_start

class Joy_Base(Node):
    def __init__(self):
        super().__init__('base_node')
        self.sub_joy = self.create_subscription(Joy,'/joy',self.callback,10)

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
        self.base_vel = 0
        self.a = 0
        self.movevel = float()

    def right(self):
        self.Vx = 0
        self.Vy = 18 #17.95
        self.Wz = 0
        self.robot_vel = numpy.array([[self.Vx], [self.Vy], [self.Wz]])
        self.base_vel = ((1/self.r)*self.eqm).dot((self.robot_vel))
        basic_movement.right(self)
        self.move_wheel()

    def left(self):
        self.Vx = 0
        self.Vy = -18 #17.95
        self.Wz = 0
        self.robot_vel = numpy.array([[self.Vx], [self.Vy], [self.Wz]])
        self.base_vel = ((1/self.r)*self.eqm).dot((self.robot_vel))
        basic_movement.left(self)
        self.move_wheel()

    def right_rotate(self): #All CCW
        self.Vx = 0#-17.9541
        self.Vy = 0
        self.Wz = 18
        self.robot_vel = numpy.array([[self.Vx], [self.Vy], [self.Wz]])
        self.base_vel = ((1/self.r)*self.eqm).dot((self.robot_vel))
        basic_movement.rotate_right(self)
        self.move_wheel() 

    def left_rotate(self):
        self.Vx = 0#-17.9541
        self.Vy = 0
        self.Wz = -18
        self.robot_vel = numpy.array([[self.Vx], [self.Vy], [self.Wz]])
        self.base_vel = ((1/self.r)*self.eqm).dot((self.robot_vel))
        basic_movement.rotate_left(self)
        self.move_wheel()

    def backward_(self):
        self.Vx = 18#-17.9541
        self.Vy = 0
        self.Wz = 0
        self.robot_vel = numpy.array([[self.Vx], [self.Vy], [self.Wz]])
        self.base_vel = ((1/self.r)*self.eqm).dot((self.robot_vel))
        basic_movement.backward(self)
        self.move_wheel()

    def forward_(self):
        self.Vx = -18#-17.9541
        self.Vy = 0
        self.Wz = 0
        self.robot_vel = numpy.array([[self.Vx], [self.Vy], [self.Wz]])
        self.base_vel = ((1/self.r)*self.eqm).dot((self.robot_vel))
        basic_movement.forward(self)
        self.move_wheel()

    def stop_(self):
        self.Vx = 0
        self.Vy = 0
        self.Wz = 0
        self.robot_vel = numpy.array([[self.Vx], [self.Vy], [self.Wz]])
        self.base_vel = ((1/self.r)*self.eqm).dot((self.robot_vel))
        basic_movement.stop(self)
        self.move_wheel()

    def right_front(self):
        print("right_front")
        self.Vx = -18
        self.Vy = 18
        self.Wz = 0
        self.robot_vel = numpy.array([[self.Vx], [self.Vy], [self.Wz]])
        self.base_vel = ((1/self.r)*self.eqm).dot((self.robot_vel))
        basic_movement.RF(self)
        self.move_wheel()

    def right_back(self):
        print("right_back")
        self.Vx = 18
        self.Vy = 18
        self.Wz = 0
        self.robot_vel = numpy.array([[self.Vx], [self.Vy], [self.Wz]])
        self.base_vel = ((1/self.r)*self.eqm).dot((self.robot_vel))

    def left_front(self):
        print("left_front")
        self.Vx = -18
        self.Vy = -18
        self.Wz = 0
        self.robot_vel = numpy.array([[self.Vx], [self.Vy], [self.Wz]])
        self.base_vel = ((1/self.r)*self.eqm).dot((self.robot_vel))

    def left_back(self):
        print("left_back")
        self.Vx = 18
        self.Vy = -18
        self.Wz = 0
        self.robot_vel = numpy.array([[self.Vx], [self.Vy], [self.Wz]])
        self.base_vel = ((1/self.r)*self.eqm).dot((self.robot_vel))

    def move_wheel(self):
        print('move_wheel',self.base_vel)
        motor_start.packetHandler.write2ByteTxRx(motor_start.portHandler, motor_start.DXL2_ID, motor_start.ADDR_MX_MOVING_SPEED, (self.base_vel[1]))  ,motor_start.packetHandler.write2ByteTxRx(motor_start.portHandler, motor_start.DXL4_ID, motor_start.ADDR_MX_MOVING_SPEED, (self.base_vel[3]))
        motor_start.packetHandler.write1ByteTxRx(motor_start.portHandler, motor_start.DXL2_ID, motor_start.ADDR_GOAL_ACCELERATION, 10)               ,motor_start.packetHandler.write1ByteTxRx(motor_start.portHandler, motor_start.DXL4_ID, motor_start.ADDR_GOAL_ACCELERATION, 10)

        motor_start.packetHandler.write2ByteTxRx(motor_start.portHandler, motor_start.DXL3_ID, motor_start.ADDR_MX_MOVING_SPEED, (self.base_vel[2]))  ,motor_start.packetHandler.write2ByteTxRx(motor_start.portHandler, motor_start.DXL1_ID, motor_start.ADDR_MX_MOVING_SPEED, (self.base_vel[0]))
        motor_start.packetHandler.write1ByteTxRx(motor_start.portHandler, motor_start.DXL3_ID, motor_start.ADDR_GOAL_ACCELERATION, 10)               ,motor_start.packetHandler.write1ByteTxRx(motor_start.portHandler, motor_start.DXL1_ID, motor_start.ADDR_GOAL_ACCELERATION, 10)
        self.base_vel = [(self.base_vel[0]),(self.base_vel[1]),(self.base_vel[2]),(self.base_vel[3])]

    def pos_callback(self,msg):
        self.W1 = int(math.floor(self.base_vel[0]))  # 0-1023
        if(self.W1 > 1023):
            self.W1 = 1023
        elif(self.W1 < -1023):
            self.W1 = -1023

        self.W2 = int(math.floor(self.base_vel[1]))  # 1024-2046
        if(self.W2 > 1023):
            self.W2 = 1023
        elif(self.W2 < -1023):
            self.W2 = -1023

        self.W3 = int(math.floor(self.base_vel[2]))  # 0-1023
        if(self.W3 > 1023):
            self.W3 = 1023
        elif(self.W3 < -1023):
            self.W3 = -1023

        self.W4 = int(math.floor(self.base_vel[3]))  # 1024-2046
        if(self.W4 > 1023):
            self.W4 = 1023
        elif(self.W4 < -1023):
            self.W4 = -1023

        if self.W1 < 0 :
            self.W1 = abs(self.W1)+1023

        if self.W2 < 0 :
            self.W2 = abs(self.W2)+1023

        if self.W3 < 0 :
            self.W3 = abs(self.W3)+1023

        if self.W4 < 0 :
            self.W4 = abs(self.W4)+1023

    def callback(self,msg):
        self.LR = abs(msg.axes[0])
        self.FB = abs(msg.axes[1])
        self.base_vel = [0,0,0,0]
        if msg.buttons[5] != 1:
            if msg.axes[1]  > 0.05 and msg.axes[0] == 0.0:
                self.forward_()

            elif msg.axes[1] < -0.05 and msg.axes[0] == 0.0:
                self.backward_()
            
            elif msg.axes[1] == 0.0 and msg.axes[0] == 0.0:
                self.stop_()
            
            elif msg.axes[0] > 0.05 and msg.axes[1] == 0.0 and msg.buttons[11] == 0.0:
                self.left()
            
            elif msg.axes[0] < -0.05 and msg.axes[1] == 0.0 and msg.buttons[11] == 0.0:
                self.right()
            
            elif msg.buttons[11] == 1 and msg.axes[1] == 0.0 and msg.axes[0] > 0.05:
                self.left_rotate()
            
            elif msg.buttons[11] == 1 and msg.axes[1] == 0.0 and msg.axes[0] < -0.05:
                self.right_rotate()
            
            # ------------------------------------------------------------------------ #
            elif msg.axes[0] < -0.05 and msg.axes[1] > 0.05 :
                self.right_front()
            
            elif msg.axes[0] < -0.05 and msg.axes[1] < -0.05 : 
                self.right_back()
            
            elif msg.axes[0] > 0.05 and msg.axes[1] > 0.05 : 
                self.left_front()
            
            elif msg.axes[0] > 0.05 and msg.axes[1] < -0.05 : 
                self.left_back()
            

    def get_vel(self):
        self.velW1 = [0,0,0]
        self.velW2 = [0,0,0]
        self.velW3 = [0,0,0]
        self.velW4 = [0,0,0]
        self.velcalW1 = 0
        self.velcalW2 = 0
        self.velcalW3 = 0
        self.velcalW4 = 0

        self.velW1 = packetHandler.read2ByteTxRx(portHandler, DXL1_ID, ADDR_MX_PRESENT_SPEED)
        if self.velW1[0] > 1023:
            self.velcalW1 = (self.velW1[0]-1023)*-1*0.11*2*3.14/60
        else:
            self.velcalW1 = self.velW1[0]*0.11*2*3.14/60

        self.velW2 = packetHandler.read2ByteTxRx(portHandler, DXL2_ID, ADDR_MX_PRESENT_SPEED)
        if self.velW2[0] > 1023:
            self.velcalW2 = (self.velW2[0]-1023)*-1*0.11*2*3.14/60
        else:
            self.velcalW2 = self.velW2[0]*0.11*2*3.14/60

def main(args=None):
    rclpy.init(args=args)
    controller = Joy_Base()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()