#!/usr/bin/env python3

import rospy
import rosbag
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult
from nav_msgs.msg import Odometry
import time

def read_goals_from_rosbag(bag_file):
    goals = []
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages():
            if topic == '/move_base_simple/goal':
                goals.append(msg)
    return goals

def result_callback(msg):
    global goal_idx, goals, start_time
    if msg.status.status == 3: # SUCCEEDED
        rospy.sleep(3)
        print("Finished last goal! Publish next goal!")
        if goal_idx < len(goals):
            goal_pub.publish(goals[goal_idx])
            print("publishing goals:", goal_idx, "!!!")
            goal_idx += 1
            # print(goals[goal_idx])
        else:
            rospy.loginfo("All goals reached!")
        start_time = time.time()
            
def odom_callback(msg):
    global start_time, dist, prev_x, prev_y, goal_idx, goals
    curr_time = time.time()
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    if dist is None:
        dist = 0
    else:
        dist += ((x - prev_x) ** 2 + (y - prev_y) ** 2) ** 0.5
    if (curr_time - start_time) > 5 and dist < 0.3:
        rospy.sleep(3)
        print("Stay too long! Publish next goal!")
        if goal_idx < len(goals):
            goal_pub.publish(goals[goal_idx])
            print("publishing goals:", goal_idx, "!!!")
            goal_idx += 1
        else:
            rospy.loginfo("All goals reached!")
        start_time = curr_time
        dist = 0
    prev_x = x
    prev_y = y

if __name__ == '__main__':
    prev_x = 0
    prev_y = 0
    dist = None
    start_time = time.time()
    goal_idx = 0
    
    rospy.init_node('goal_publisher')
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    result_sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, result_callback)
    odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, odom_callback)

    bag_file = '/home/chifong/path_v3.bag'
    goals = read_goals_from_rosbag(bag_file)
    print("reading goals!!!")
    
    rospy.sleep(3)
    print("sleep for 3s!!!")
    
    # goal_pub.publish(goals[goal_idx])
    # print("publishing goals:", goal_idx, "!!!")
    # goal_idx += 1
            
    rospy.spin()


