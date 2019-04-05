#!/usr/bin/python

# Copyright 2019 Fetch Robotics Inc.
# Author(s): Sarah Elliott

# Python
from __future__ import print_function
from datetime import datetime
from datetime import timedelta

# ROS
import rospy
import actionlib
from fetchit_challenge.msg import SchunkMachineAction, SchunkMachineResult, SchunkMachineGoal

# GPIO
try:
    import RPi.GPIO as GPIO
except ImportError:
    print("This script must be run on a Raspberry Pi. (RPi.GPIO import failed)")
    sys.exit(1)


class SchunkMachineServer(object):
    """Class for using Schunk Machine chuck."""
    _result = SchunkMachineResult()

    def __init__(self):
        """Specifies pin numbering schema and sets up GPIO channels"""
        # Setup GPIO
        GPIO.setwarnings(False)
        mode = GPIO.getmode()
        if mode is None:
            GPIO.setmode(GPIO.BOARD)
        elif mode == GPIO.BCM:
            GPIO.setup([], GPIO.OUT)
            GPIO.cleanup()
            GPIO.setmode(GPIO.BOARD)

        GPIO.setup(37, GPIO.OUT, initial=1)
        GPIO.setup(40, GPIO.OUT, initial=0)

        self._current_state = SchunkMachineGoal.OPEN
        # Minimum time for chuck to be closed
        self._lock_time = 120.0
        self.server = actionlib.SimpleActionServer('schunk_machine', SchunkMachineAction, self.callback, False)
        self.server.start()

    def callback(self, goal):
        """Action server callback."""
        print("Received goal: " + str(goal))
        if goal.state == self._current_state:
            self._result.success = True
            self._result.message = "Schunk Machine Chuck already in desired state."
            self.server.set_succeeded(self._result)
        elif goal.state == SchunkMachineGoal.CLOSE:
            self._lock_until = datetime.now() + timedelta(seconds=self._lock_time)
            self.close()
            self._result.success = True
            self._result.message = "Schunk Machine Chuck closed."
            self.server.set_succeeded(self._result)
            self._current_state = SchunkMachineGoal.CLOSE
        elif goal.state == SchunkMachineGoal.OPEN:
            time_left = self._lock_until - datetime.now()
            if time_left.total_seconds() > 0.0:
                self._result.success = False
                self._result.message = "Schunk Machine Chuck must be closed for at least " + str(self._lock_time) +\
                        "s. Please wait " + str(time_left.total_seconds()) + "s."
                self.server.set_aborted(self._result)
            else:
                self.open()
                self._result.success = True
                self._result.message = "Schunk Machine Chuck open."
                self.server.set_succeeded(self._result)
                self._current_state = SchunkMachineGoal.OPEN
        else:
            self._result.success = False
            self._result.message = "Unknown goal type"
            self.server.set_aborted(self._result)

    def open(self):
        """Set Pi pins to open chuck."""
        GPIO.output(40, 0)
        GPIO.output(37, 1)
            
    def close(self):
        """Set Pi pins to close chuck."""
        GPIO.output(40, 1)
        GPIO.output(37, 0)


if __name__ == "__main__":
    rospy.init_node('schunk_machine_server')
    machine = SchunkMachineServer()
    rospy.spin()
