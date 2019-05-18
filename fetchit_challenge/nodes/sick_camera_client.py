#! /usr/bin/env python

# Copyright 2019 Fetch Robotics Inc.
# Author(s): Niharika Arora

# Python
from __future__ import print_function

# ROS
import rospy
import actionlib
from fetchit_challenge.msg import SickCameraAction, SickCameraResult, SickCameraGoal

def sick_camera_client():
    client = actionlib.SimpleActionClient('sick_camera', SickCameraAction)
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = SickCameraGoal(trigger=0)
    client.send_goal(goal)
    client.wait_for_result()

    return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('sick_camera_client')
        result = sick_camera_client()
        rospy.sleep(1.0)
        print("done")
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)

