#!/usr/bin/env python3
import numpy as np
import os
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, WheelsCmdStamped
from std_msgs.msg import Header, Float32

class OdometryNode(DTROS):

    def __init__(self, node_name):
        """Wheel Encoder Node
        This implements basic functionality with the wheel encoders.
        """

        # Initialize the DTROS parent class
        super(OdometryNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.veh_name = rospy.get_namespace().strip("/")
        self.left_distance = 0.0
        self.prev_left = 0.0
        self.right_distance = 0.0
        self.prev_right = 0.0

        # Get static parameters
        self._radius = rospy.get_param(f'{self.veh_name}/kinematics_node/radius', 100)

        # Subscribing to the wheel encoders
        self.sub_encoder_ticks_left = rospy.Subscriber(f'{self.veh_name}/left_wheel_encoder_node/tick',WheelEncoderStamped,callback=self.cb_encoder_data,callback_args='left')
        self.sub_encoder_ticks_right = rospy.Subscriber(f'{self.veh_name}/right_wheel_encoder_node/tick',WheelEncoderStamped,callback=self.cb_encoder_data,callback_args='right')
        self.sub_executed_commands = rospy.Subscriber(f'{self.veh_name}/wheels_driver_node/wheels_cmd_executed',WheelsCmdStamped,callback=self.cb_executed_commands)

        # Publishers
        self.pub_integrated_distance_left = rospy.Publisher(f'{self.veh_name}/left_wheel_distance_traveled',Float32,queue_size=1)
        self.pub_integrated_distance_right = rospy.Publisher(f'{self.veh_name}/right_wheel_distance_traveled',Float32,queue_size=1)

        self.log("Initialized")

    def cb_encoder_data(self, msg, wheel):
        """ Update encoder distance information from ticks.
        """
        # Retrieve encoder data
        ticks = msg.data
        resolution = msg.resolution

        # Compute total distance measured by the encoder
        total_dist = (2 * np.pi * self._radius) * (ticks/resolution)

        if wheel == 'left':
            self.left_distance += total_dist - self.prev_left
            self.pub_integrated_distance_left.publish(self.left_distance)
            self.prev_left = total_dist

        elif wheel == 'right':
            self.right_distance += total_dist - self.prev_right
            self.pub_integrated_distance_right.publish(self.right_distance)
            self.prev_right = total_dist


    def cb_executed_commands(self, msg):
        """ Use the executed commands to determine the direction of travel of each wheel.
        """
        pass

if __name__ == '__main__':
    node = OdometryNode(node_name='my_odometry_node')
    # Keep it spinning to keep the node alive
    rospy.spin()
    rospy.loginfo("wheel_encoder_node is up and running...")