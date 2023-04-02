#!/usr/bin/env python3

import rospy
import rosbag
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult

def read_goals_from_rosbag(bag_file):
    goals = []
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages():
            if topic == '/move_base_simple/goal':
                goals.append(msg)
    return goals

def result_callback(msg):
    global goal_idx, goals
    if msg.status.status == 3: # SUCCEEDED
        goal_idx += 1 
        if goal_idx < len(goals):
            goal_pub.publish(goals[goal_idx])
            print("publishing goals:", goal_idx, "!!!")
            # print(goals[goal_idx])
        else:
            rospy.loginfo("All goals reached!")

if __name__ == '__main__':
    rospy.init_node('goal_publisher')
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    result_sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, result_callback)

    bag_file = '/home/chifong/path_v3.bag'
    goals = read_goals_from_rosbag(bag_file)
    print("reading goals!!!")
    
    rospy.sleep(3)
    print("sleep for 3s!!!")
    
    goal_idx = 0

    rate = rospy.Rate(0.2) # 1 Hz
    while not rospy.is_shutdown():
        if goal_idx < len(goals):
            goal_pub.publish(goals[goal_idx])
            print("publishing goals:", goal_idx, "!!!")
            # print(goals[goal_idx])
            
        rate.sleep()

    rospy.spin()

