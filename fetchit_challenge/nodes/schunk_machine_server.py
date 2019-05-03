#!/usr/bin/python

# Copyright 2019 Fetch Robotics Inc.
# Author(s): Sarah Elliott

# Python
from __future__ import print_function
from datetime import datetime
from datetime import timedelta

# ROS
import rospy
import sys
import actionlib
from fetchit_challenge.msg import SchunkMachineAction, SchunkMachineResult, SchunkMachineGoal
from shunk_machine_controlers import ShunkMachineControlersReal
from shunk_machine_controlers_sim import ShunkMachineControlersSim


class SchunkMachineServer(object):
    """Class for using Schunk Machine chuck."""
    _result = SchunkMachineResult()

    def __init__(self, is_simulated = True):
        rospy.loginfo("Starting SchunkMachineServer...")
        # We Initialise the Real or Simulated Control object
        if is_simulated:
            self._shunk_controlers_object = ShunkMachineControlersSim()
        else:
            self._shunk_controlers_object = ShunkMachineControlersReal()

        self._current_state = SchunkMachineGoal.OPEN
        # Minimum time for chuck to be closed
        self._lock_time = 120.0
        self._lock_until = datetime.now()
        self.server = actionlib.SimpleActionServer('schunk_machine', SchunkMachineAction, self.callback, False)
        self.server.start()
        rospy.loginfo("SchunkMachineServer...READY")

    def callback(self, goal):
        """Action server callback."""
        rospy.loginfo("Received goal: " + str(goal))
        if goal.state == self._current_state:
            self._result.success = True
            self._result.message = "Schunk Machine Chuck already in desired state."
            self.server.set_succeeded(self._result)
        elif goal.state == SchunkMachineGoal.CLOSE:
            self._shunk_controlers_object.close_chuck()
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
                self._shunk_controlers_object.open_chuck()
                self._result.success = True
                self._result.message = "Schunk Machine Chuck open."
                self.server.set_succeeded(self._result)
                self._current_state = SchunkMachineGoal.OPEN
        elif goal.state == SchunkMachineGoal.START_OPERATION:
            self._lock_until = datetime.now() + timedelta(seconds=self._lock_time)
            self._shunk_controlers_object.start_operation()
            self._result.success = True
            self._result.message = "Schunk Machine Started Operation, please wait =="+str(self._lock_time)+" seconds"
            self.server.set_succeeded(self._result)
            self._current_state = SchunkMachineGoal.START_OPERATION

        elif goal.state == SchunkMachineGoal.END_OPERATION:
            time_left = self._lock_until - datetime.now()
            if time_left.total_seconds() > 0.0:
                self._result.success = False
                self._result.message = "Schunk Machine Chuck must be operating for at least " + str(self._lock_time) +\
                        "s. Please wait " + str(time_left.total_seconds()) + "s."
                self.server.set_aborted(self._result)
            else:
                self._shunk_controlers_object.end_operation()
                self._result.success = True
                self._result.message = "Schunk Machine operation ended open."
                self.server.set_succeeded(self._result)
                self._current_state = SchunkMachineGoal.END_OPERATION

        else:
            self._result.success = False
            self._result.message = "Unknown goal type"
            self.server.set_aborted(self._result)

        rospy.loginfo("Message: " + str(self._result.message))

if __name__ == "__main__":
    rospy.init_node('schunk_machine_server')


    if len(sys.argv) > 1:
        simulated = sys.argv[1]
        is_simulation = simulated == "simulated"
        machine = SchunkMachineServer(is_simulated=True)
        rospy.spin()
    else:
        rospy.logerr("Use: python fetchit_challenge schunk_chine_server.py simulated/real")
