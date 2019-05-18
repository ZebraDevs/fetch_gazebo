#!/usr/bin/python

# Copyright 2019 Fetch Robotics Inc.
# Author(s): Niharika Arora

# Python
from __future__ import print_function
from datetime import datetime, timedelta
import socket
import sys
import time

# ROS
import rospy
import actionlib
from fetchit_challenge.msg import SickCameraAction, SickCameraResult, SickCameraGoal

class PIM60:
    """Interface for the PIm60 camera"""
    def __init__(self, ip, port=2116):
        self.buf = ''
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect( (ip, port) )
        self.sock.settimeout(1.0)

    def trigger(self):
        """Triggers the camera click"""
        self.send("TRIG")
        resp = self.recv()
        print(resp)

    def send(self, msg):
        """ Sends messages to device """
        print('SENT')
        self.sock.sendall(msg)

    def recv(self, timeout = 0.5):
        """ Received respose, will strip <STX> and <ETX> from response message """
        buf = self.buf
        while buf.find(chr(3)) < 0:
            buf += self.sock.recv(1000)

        start_idx = buf.find(chr(2))
        if start_idx < 0:
            print(" Could not find STX in buffer :", self.prettyMsg(buf))
        elif start_idx != 0:
            print(" Garbage data before STX :", self.prettyMsg(buf[:start_idx+1]))

        stop_idx = buf.find(chr(3))

        # Store any following messages in buffer
        self.buf = buf[stop_idx+1:]

        # Make sure remaining message has a STX at begining
        if len(self.buf) and (self.buf[0] != chr(2)):
            print(" Extra garbage after ETX :", self.prettyMsg(buf[stop_idx+1:]))
        if start_idx > stop_idx:
            print(" Found stop before start", start_idx, stop_idx)
        return buf[start_idx+1:stop_idx]


class SickCameraServer(object):
    _result = SickCameraResult()
    _wait_duration = 120
    _wait_until = datetime.now()

    def __init__(self):
        self._sick_camera = PIM60("192.168.100.103", 2116)
        self.server = actionlib.SimpleActionServer('sick_camera', SickCameraAction, self.callback, False)
        self.server.start()
        self.wait_duration = 5
        self.wait_until = datetime.now() - timedelta(seconds= self.wait_duration)


    def callback(self, goal):
        """Action server callback."""
        print("Received goal: " + str(goal))

        if goal.trigger == SickCameraGoal.TRIG:
            time_left = self.wait_until - datetime.now()
            if time_left.total_seconds() > 0.0:
                print("Sick Camera can click only every 5 seconds")
                self._result.success = False
                self._result.message = "Sick Camera can click only every 5 seconds " + str(self.wait_duration) +\
                        "s. Please wait " + str(time_left.total_seconds()) + "s."
                self.server.set_aborted(self._result)
            else:
                self.click()
                self.wait_until = datetime.now() + timedelta(seconds=self.wait_duration)
                self._result.success = True
                self._result.message = "Sick Camera took picture!."
                self.server.set_succeeded(self._result)
        else:
            self._result.success = False
            self._result.message = "Unknown goal type"
            self.server.set_aborted(self._result)

    def click(self):
        """Clicks the pitures!"""
        self._sick_camera.trigger()
            

if __name__ == "__main__":
    rospy.init_node('sick_camera_server')
    machine = SickCameraServer()
    rospy.spin()

