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


if __name__ == '__main__':

    bag_file = '/home/chifong/path_v3.bag'
    goals = read_goals_from_rosbag(bag_file)
    print("reading goals!!!")
    
    print(goals)


