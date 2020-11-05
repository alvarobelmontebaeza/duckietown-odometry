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
        self.veh_name = "/" + os.environ['VEHICLE_NAME']
        self.left_distance = 0.0
        self.prev_left = 0.0
        self.right_distance = 0.0
        self.prev_right = 0.0
        self.first_message_left = True
        self.first_message_right = True
        self.initial_ticks_left = 0.0
        self.initial_ticks_right = 0.0

        # Get static parameters
        self._radius = rospy.get_param(f'{self.veh_name}/kinematics_node/radius', 100)
        self._resolution = 135

        # Subscribing to the wheel encoders
        self.sub_encoder_ticks_left = rospy.Subscriber(f'{self.veh_name}/left_wheel_encoder_node/tick',WheelEncoderStamped,callback=self.cb_encoder_data,callback_args='left')
        self.sub_encoder_ticks_right = rospy.Subscriber(f'{self.veh_name}/right_wheel_encoder_node/tick',WheelEncoderStamped,callback=self.cb_encoder_data,callback_args='right')

        # Publishers
        self.pub_integrated_distance_left = rospy.Publisher(f'{self.veh_name}/left_wheel_distance_traveled',Float32,queue_size=1)
        self.pub_integrated_distance_right = rospy.Publisher(f'{self.veh_name}/right_wheel_distance_traveled',Float32,queue_size=1)

        self.log("Initialized")

    def cb_encoder_data(self, msg, wheel):
        """ Update encoder distance information from ticks.
        """
        # Retrieve encoder data
        ticks = msg.data

        # Check if it's the first received message and store initial encoder value
        if wheel == 'left' and self.first_message_left == True:
            self.first_message_left = False
            self.initial_ticks_left = ticks
        if wheel == 'right' and self.first_message_right ==True:
            self.first_message_right = False
            self.initial_ticks_right = ticks

        # Compute total distance traveled by the wheel
        if wheel == 'left':
            rel_ticks = ticks - self.initial_ticks_left
            diff_ticks = np.abs(rel_ticks - self.prev_left)
            dist = (2 * np.pi * self._radius) * (diff_ticks/self._resolution)
            # Accumulate distance and publish it
            self.left_distance += dist
            self.pub_integrated_distance_left.publish(self.left_distance)
            self.prev_left = rel_ticks
            self.log("LEFT WHEEL DISTANCE: %s" % self.left_distance)

        elif wheel == 'right':
            rel_ticks = ticks - self.initial_ticks_right
            diff_ticks = np.abs(rel_ticks - self.prev_right)
            dist = (2 * np.pi * self._radius) * (diff_ticks/self._resolution)
            # Accumulate distance and publish it
            self.right_distance += dist
            self.pub_integrated_distance_right.publish(self.right_distance)
            self.prev_right = rel_ticks
            self.log("RIGHT WHEEL DISTANCE: %s" % self.right_distance)


if __name__ == '__main__':
    node = OdometryNode(node_name='my_odometry_node')
    # Keep it spinning to keep the node alive
    rospy.loginfo("wheel_encoder_node is up and running...")
    rospy.spin()