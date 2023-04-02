#!/usr/bin/env python3

import rospy
import rosbag
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult
from nav_msgs.msg import Odometry
import time

"""
Read the goals from rosbag

Parameters: 
bag_file: rosbag file, contains the /move_base_simple/goal message (position and orientation)

Returns:
goals: /move_base_simple/goal message (position and orientation)
"""
def read_goals_from_rosbag(bag_file):
    goals = []
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages():
            if topic == '/move_base_simple/goal':
                goals.append(msg)
    return goals

"""
Callback function: check if the robot reaches the goal

Parameters:
msg: subscribe the /move_base/result topic, check to the msg.status.status

Returns:
Publish a new goal if msg.status.status == 3
"""
def result_callback(msg):
    global goal_idx, goals, start_time
    if msg.status.status == 3: # succeeded
        rospy.sleep(1)
        print("Finished last goal! Publish next goal!")
        if goal_idx < len(goals):
            goal_pub.publish(goals[goal_idx])       # publish the goal
            print("publishing goals:", goal_idx, "!!!")
            goal_idx += 1           # update the goal index
            # print(goals[goal_idx])
        else:
            print("All goals reached!")
        start_time = time.time()    # update the start time

"""
Callback function: check if the robot stationay time is >10s

Parameters:
msg: subscribe the /odometry/filtered topic, check to the msg.pose.pose.position.x/y

Returns:
Publish a new goal if the stationary time more than 10 seconds 
"""
def odom_callback(msg):
    global start_time, dist, prev_x, prev_y, goal_idx, goals
    curr_time = time.time()         # update the current time
    x = msg.pose.pose.position.x    # update the current position x
    y = msg.pose.pose.position.y    # update the current position y
    dist = ((x - prev_x) ** 2 + (y - prev_y) ** 2) ** 0.5   
    if (curr_time - start_time) > 10 and dist < 0.000001:
        # rospy.sleep(3)
        print("Stay too long! Publish next goal!")
        if goal_idx < len(goals):
            goal_pub.publish(goals[goal_idx])       # publish the goal
            print("publishing goals:", goal_idx, "!!!")
            goal_idx += 1           # update the goal index
        else:
            rospy.loginfo("All goals reached!")
        start_time = time.time()    # update the start time
    prev_x = x
    prev_y = y

if __name__ == '__main__':
    prev_x = 0
    prev_y = 0
    dist = 0
    start_time = time.time()
    goal_idx = 0
    
    # initialize the node, publisher and subscriber
    rospy.init_node('goal_publisher')
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    result_sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, result_callback)
    odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, odom_callback)

    # read the bag file
    bag_file = 'path_v3.bag'
    goals = read_goals_from_rosbag(bag_file)
    print("reading goals!!!")
    
    rospy.sleep(3)
    print("sleep for 3s!!!")
            
    rospy.spin()
