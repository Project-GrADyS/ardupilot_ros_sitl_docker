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

from ArdupilotModules.Copter import Copter

import rclpy
from rclpy.node import Node

class ArmTest(Copter):
    def __init__(self, node_name, SYSID):
        super().__init__(node_name, SYSID)
        state = self.getState()
        state.setMavrosStreamRate()
        state.setMode("GUIDED")# custom modes: http://wiki.ros.org/mavros/CustomModes
        state.setArm(True)


def main(args=None):
    rclpy.init(args=args)

    arm_test = ArmTest('arm_client', 25)

    try:
        arm_test.get_logger().warning('Beginning client, shut down with CTRL-C')
        rclpy.spin(arm_test)
    except KeyboardInterrupt:
        arm_test.get_logger().info('Keyboard interrupt, shutting down.\n')
    finally:
        arm_test.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()