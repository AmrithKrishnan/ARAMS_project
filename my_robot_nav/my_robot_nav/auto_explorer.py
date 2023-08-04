#!/usr/bin/env python3

from math import sqrt
import sys
import rclpy
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

import random
import numpy as np

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Point, Quaternion

# Map Bounds
map_min_x = -2.5
map_max_x = 2.5
map_min_y = -2.5
map_max_y = 2.5

# Fixed goal positions
fixed_positions = [
    [x1, y1],  # Add the actual coordinates of the fixed goals here
    [x2, y2],
    [x3, y3],
    # Add more fixed goal positions as needed. i think total of 12 are needed for the task
]

success = True
goals_sent = 0

def main():
    global goals_sent

    rclpy.init()

    auto_chaos = rclpy.create_node('auto_goals')

    # create Action Client object with desired message type and action name
    nav_to_pose_client = ActionClient(auto_chaos, NavigateToPose, 'navigate_to_pose')

    # wait for action server to come up
    while not nav_to_pose_client.wait_for_server(timeout_sec=2.0):
        print("Server still not available; waiting...")

    while goals_sent < 3 and rclpy.ok():
        try:
            position, orientation = generate_random_goal()
            goal_handle = send_goal(position, orientation)
            check_result(goal_handle)
        except KeyboardInterrupt:
            print("Shutdown requested... complying...")
            break

    nav_to_pose_client.destroy()
    auto_chaos.destroy_node()
    rclpy.shutdown()

def send_goal(position, orientation):
    """Create action goal object and send to action server, check if goal accepted"""
    global auto_chaos
    global goals_sent

    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = "map"

    goal.pose.header.stamp = auto_chaos.get_clock().now().to_msg()
  
    goal.pose.pose.position = position
    goal.pose.pose.orientation = orientation

    print("Received new goal => X: " + str(goal.pose.pose.position.x) + " Y: " + str(goal.pose.pose.position.y))

    send_goal_future = nav_to_pose_client.send_goal_async(goal)
    rclpy.spin_until_future_complete(auto_chaos, send_goal_future)

    goal_handle = send_goal_future.result()

    if not goal_handle.accepted:
        print("Goal was rejected")
        nav_to_pose_client.destroy()
        auto_chaos.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    print("Goal Accepted!")
    goals_sent += 1

    return goal_handle

def check_result(goal_handle):
    """Check for task completion while blocking further execution"""
    get_result_future = goal_handle.get_result_async()

    rclpy.spin_until_future_complete(auto_chaos, get_result_future)

    status = get_result_future.result().status

    if status == GoalStatus.STATUS_SUCCEEDED:
        print("Reached Goal!!!")

def generate_random_goal():
    """Generate a random position and orientation for the goal"""
    global fixed_positions

    # Select a random fixed position
    position = Point()
    position.x, position.y = random.choice(fixed_positions)
    position.z = 0.0

    # Generate a random orientation
    orientation = Quaternion()
    orientation.w = round(random.uniform(-1.0, 1.0), 3)
    orientation.x = 0.0
    orientation.y = 0.0
    orientation.z = sqrt(1 - orientation.w * orientation.w)

    return position, orientation

if __name__ == '__main__':
    main()
