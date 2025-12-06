#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import tf_transformations

def create_pose(navigator, x, y, theta=0.0):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    
    # Convert theta to quaternion (we want fixed heading 0 for holonomic test)
    q = tf_transformations.quaternion_from_euler(0, 0, theta)
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    return pose

def main():
    rclpy.init()
    navigator = BasicNavigator()

    # Wait for Nav2 to be fully active
    # navigator.waitUntilNav2Active() 
    # Note: If running on laptop while Nav2 is on Jetson, this might timeout if discovery is slow.
    # You can comment it out if you are sure Nav2 is ready.

    # Define the Square Path (2.5m x 2.5m area, 1m square)
    # A(1,0) -> B(1,1) -> C(0,1) -> D(0,0)
    # Heading is ALWAYS 0.0 (Facing forward) to force strafing
    
    waypoints = []
    
    # Point A: (1.0, 0.0)
    waypoints.append(create_pose(navigator, 1.0, 0.0, 0.0))
    
    # Point B: (1.0, 1.0)
    waypoints.append(create_pose(navigator, 1.0, 1.0, 0.0))
    
    # Point C: (0.0, 1.0)
    waypoints.append(create_pose(navigator, 0.0, 1.0, 0.0))
    
    # Point D: (0.0, 0.0) - Back Home
    waypoints.append(create_pose(navigator, 0.0, 0.0, 0.0))

    print("Starting Square Path Test (Holonomic Mode)...")
    # navigator.followWaypoints(waypoints)
    
    # Alternatively, go to each point one by one to verify accuracy
    for i, wp in enumerate(waypoints):
        print(f"Going to Waypoint {i+1}: x={wp.pose.position.x}, y={wp.pose.position.y}")
        navigator.goToPose(wp)
        
        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            # print(f"Distance remaining: {feedback.distance_remaining:.2f}")
        
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f"Waypoint {i+1} Reached!")
        else:
            print(f"Waypoint {i+1} Failed!")
            break

    print("Test Complete!")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
