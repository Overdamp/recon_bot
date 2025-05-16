#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from MobiControl import Mobility
import sys, os, yaml
from std_msgs.msg import Float32MultiArray, Float64MultiArray
from std_msgs.msg import String
from std_msgs.msg import Bool
import json
from rclpy import qos
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan
import time
import numpy as np

class MobilityNode(Node):
    def __init__(self):
        print("node int mobility")
        super().__init__('Mobility_node')
        self.MobiFunction = Mobility()
        Streaming = QoSProfile(durability=qos.QoSDurabilityPolicy.VOLATILE,
                           reliability=qos.QoSReliabilityPolicy.BEST_EFFORT, history=qos.QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.send_x = 0.0
        self.send_y = 0.0
        self.last_message_time = time.time()
        self.flag_timeout = False

        self.Lidar_distance = 0.0
        self.Lidar_xsign = 1
        self.Lidar_ysign = 1
        self.Lidar_stop_distance = 0.5
        self.Lidar_warn_distance = 0.8
        self.Lidar_status = "OK"

        self.current_vel = []
        self.mobi_status = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.send_velocity = Float32MultiArray()
        self.send_mobi_status = Float32MultiArray()
        self.send_to_lidar = Float32MultiArray()
        self.send_lidar_touser = Float32MultiArray()

        self.timer = self.create_timer(0.1, self.main)

        self.dxlvel_publisher = self.create_publisher(Float32MultiArray,'mobility/feedback/velocity',Streaming)
        self.status_publisher = self.create_publisher(Float32MultiArray,'mobility/feedback/status',Streaming)

        self.velo_subscription = self.create_subscription(String,'mobility/input/velocity',self.Drivedxl,Streaming)
        self.stop_subscription = self.create_subscription(String,'mobility/stop',self.Stopdxl,Streaming)

        #Lidars
        self.mobi_to_lidar_publisher = self.create_publisher(Float32MultiArray,'mobility/lidars',Streaming)
        self.lidar_subscription = self.create_subscription(Float32MultiArray,'/lidar/mobility',self.get_lidar ,Streaming)
      
    def main(self):
        current_time = time.time()

        # if current_time - self.last_message_time > 1.0:
        #     self.MobiFunction.SetWheelVelocity(Vx = 0.0, Vy = 0.0, Wz= 0.0)

        # else:

        self.check_obstacle_direct()

        #Set mobi direction in vx,vy,wz that recieve from callback
        self.MobiFunction.SetWheelVelocity(Vx = self.vx, Vy = self.vy, Wz= self.wz)

        self.current_vel = self.MobiFunction.GetVelocity() 
        self.mobi_status = self.MobiFunction.GetTempAndLoad()

        self.send_velocity.data = [self.current_vel[0][0], self.current_vel[1][0], self.current_vel[2][0]]
        self.send_mobi_status.data = self.mobi_status

        self.send_x = self.vx
        self.send_y = self.vy
        if (np.sign(self.vx), np.sign(self.vy)) == (1,0):
            self.send_y = 1
        elif (np.sign(self.vx), np.sign(self.vy)) == (-1,0):
            self.send_y = -1
        elif (np.sign(self.vx), np.sign(self.vy)) == (0,1):
            self.send_x = 1
        elif (np.sign(self.vx), np.sign(self.vy)) == (0,-1):
            self.send_x = -1

        self.send_to_lidar.data = [float(self.send_x), float(self.send_y)]
        self.dxlvel_publisher.publish(self.send_velocity)
        self.status_publisher.publish(self.send_mobi_status)

        if (np.sign(self.send_x), np.sign(self.send_y)) != (0,0):
            print('send')
            self.mobi_to_lidar_publisher.publish(self.send_to_lidar) #to lidar node


    def Drivedxl(self, msg:String):
        self.last_message_time = time.time()

        if self.flag_timeout == False:
            self.st = json.loads(msg.data)
            self.vx = self.st["mobi_vx"]
            self.vy = self.st["mobi_vy"]
            self.wz = self.st["mobi_w"]

            print("recieve vel: ", self.vx, " ", self.vy, " ", self.wz)

    def Stopdxl(self, msg:String):
        self.MobiFunction.SetWheelVelocity(Vx = 0.0, Vy = 0.0, Wz= 0.0)

    def get_lidar(self, msg:Float32MultiArray):
        self.Lidar_distance = msg.data[0]
        self.Lidar_xsign = msg.data[1]
        self.Lidar_ysign = msg.data[2]

    def check_obstacle_direct(self):
        obstacle_direct = (np.sign(self.Lidar_xsign), np.sign(self.Lidar_ysign))
        mobi_direct = (np.sign(self.send_x), np.sign(self.send_y))

        # print("obstacle_direction: ", obstacle_direct)

        if self.Lidar_distance < self.Lidar_stop_distance:
            self.Lidar_status = "stop"
            if mobi_direct == obstacle_direct:
                self.vx = 0.0
                self.vy = 0.0
                self.wz = 0.0

        elif self.Lidar_distance < self.Lidar_warn_distance:
            self.Lidar_status = "beware"
            if mobi_direct == obstacle_direct:
                pass
                # self.vx = self.vx/(1+((self.Lidar_warn_distance-self.Lidar_distance)))
                # self.vy = self.vy/(1+((self.Lidar_warn_distance-self.Lidar_distance)))
                # self.wz = self.wz/(1+((self.Lidar_warn_distance-self.Lidar_distance)))

        elif self.Lidar_distance > self.Lidar_warn_distance:
            self.Lidar_status = "OK"

        print("Lidar_status: ", self.Lidar_status, self.Lidar_distance)

def main(args=None):
    rclpy.init(args=args)
    node_object = MobilityNode()
    try:
        while rclpy.ok():
            rclpy.spin_once(node_object)
    except KeyboardInterrupt:
        print('repeater stopped cleanly')
    except BaseException:
        print('exception in repeater:', file=sys.stderr)
        raise
    finally:
        node_object.destroy_node()
        rclpy.shutdown() 


if __name__ == '__main__':
    main()