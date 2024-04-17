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
import rclpy
from rclpy.node import Node
from mavros_msgs.srv import SetMode, CommandLong, CommandInt, CommandBool, CommandHome, CommandTOL, CommandTriggerControl, CommandTriggerInterval

class ArmTest(Node):
    def __init__(self):
        super().__init__('arm_node')
        self.srv_set_mode = self.create_client(SetMode, "mavros/set_mode")
        while not self.srv_set_mode.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('WARN: set_mode service not available!')
        self.srv_arm = self.create_client(CommandBool, "/mavros/cmd/arming")
        while not self.srv_arm.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('WARN: arm service not available!')

        # create reposition request
        msg_flight_mode = SetMode.Request(base_mode=0, custom_mode='GUIDED')
        status = self.srv_set_mode.call_async(msg_flight_mode)
        print(status)

        msg_arming = CommandBool.Request(value=True)
        status = self.srv_arm.call_async(msg_arming)
        print(status)

def main(args=None):
    rclpy.init(args=args)
    arm_test = ArmTest()
    rclpy.spin(arm_test)
    arm_test.destroy_node()
    rclpy.shutdown() 	



if __name__ == '__main__':
    main()
