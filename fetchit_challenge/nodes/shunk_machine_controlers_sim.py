#!/usr/bin/python
import rospy
from std_msgs.msg import Float64

class ShunkMachineControlersSim(object):

    def __init__(self, open_chuck_value = 0.0, close_chuck_value = 0.01, chuck_operation_velocity = 10.0):
        self.setup(open_chuck_value, close_chuck_value, chuck_operation_velocity)
        self.wait_publishers_to_be_ready()
        self.init_position()

    def setup(self, open_chuck_value, close_chuck_value, chuck_operation_velocity):

        self._open_chuck_value = open_chuck_value
        self._close_chuck_value = close_chuck_value
        self._chuck_operation_velocity = chuck_operation_velocity

        self.pub_chuck_velocity = rospy.Publisher(
            '/schunk_machine/lathe_joint_velocity_controller/command',
            Float64,
            queue_size=1)

        self.pub_chuck_gripper1_position = rospy.Publisher(
            '/schunk_machine/lathe_gripper1_joint_position_controller/command',
            Float64,
            queue_size=1)
        self.pub_chuck_gripper2_position = rospy.Publisher(
            '/schunk_machine/lathe_gripper2_joint_position_controller/command',
            Float64,
            queue_size=1)

        self.pub_chuck_gripper3_position = rospy.Publisher(
            '/schunk_machine/lathe_gripper3_joint_position_controller/command',
            Float64,
            queue_size=1)

    def init_position(self):
        self.open_chuck()

    def wait_publishers_to_be_ready(self):
        publishers_ready = False

        rate_wait = rospy.Rate(10)
        while not publishers_ready:
            chuck_velocity_num = self.pub_chuck_velocity.get_num_connections()
            chuck_gripper1_num = self.pub_chuck_gripper1_position.get_num_connections()
            chuck_gripper2_num = self.pub_chuck_gripper2_position.get_num_connections()
            chuck_gripper3_num = self.pub_chuck_gripper3_position.get_num_connections()
            publishers_ready = (chuck_velocity_num > 0) and (chuck_gripper1_num > 0) and (chuck_gripper2_num > 0) and (chuck_gripper3_num > 0)
            rate_wait.sleep()

    def open_chuck(self):
        gripper1_pos_msg = Float64()
        gripper1_pos_msg.data = self._open_chuck_value

        gripper2_pos_msg = Float64()
        gripper2_pos_msg.data = self._open_chuck_value

        gripper3_pos_msg = Float64()
        gripper3_pos_msg.data = self._open_chuck_value

        # Publish Joint Position
        self.pub_chuck_gripper1_position.publish(gripper1_pos_msg)
        self.pub_chuck_gripper2_position.publish(gripper2_pos_msg)
        self.pub_chuck_gripper3_position.publish(gripper3_pos_msg)

    def close_chuck(self):

        gripper1_pos_msg = Float64()
        gripper1_pos_msg.data = self._close_chuck_value

        gripper2_pos_msg = Float64()
        gripper2_pos_msg.data = self._close_chuck_value

        gripper3_pos_msg = Float64()
        gripper3_pos_msg.data = self._close_chuck_value

        # Publish Joint Position
        self.pub_chuck_gripper1_position.publish(gripper1_pos_msg)
        self.pub_chuck_gripper2_position.publish(gripper2_pos_msg)
        self.pub_chuck_gripper3_position.publish(gripper3_pos_msg)

    def start_operation(self):
        chuck_velocity_msg = Float64()
        chuck_velocity_msg.data = self._chuck_operation_velocity

        # Publish Joint Position
        self.pub_chuck_velocity.publish(chuck_velocity_msg)

    def end_operation(self):
        chuck_velocity_msg = Float64()
        chuck_velocity_msg.data = 0.0

        # Publish Joint Position
        self.pub_chuck_velocity.publish(chuck_velocity_msg)