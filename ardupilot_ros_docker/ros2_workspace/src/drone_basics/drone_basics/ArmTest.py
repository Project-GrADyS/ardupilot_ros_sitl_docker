#! /usr/bin/env python3

__all__ = (
    'long',
    'int',
    'arming',
    'set_home',
    'takeoff',
    'land',
    'trigger_control',
)

import sys
import os
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir) # add current directory to PYTHONPATH

from ArdupilotVehicle import ArdupilotVehicle

import rclpy
from rclpy.node import Node

class ArmTest(Node, ArdupilotVehicle):
    def __init__(self):
        super().__init__('arm_node')
        self.get_logger().info('INFO: Setting up MAVROS stream rate')
        self.setMavrosStreamRate()
        self.setMode("GUIDED")# custom modes: http://wiki.ros.org/mavros/CustomModes
        self.setArm(True)


def main(args=None):
    rclpy.init(args=args)


    arm_test = ArmTest()
    rclpy.spin(arm_test)
    arm_test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
