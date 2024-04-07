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
from mavros_msgs.srv import WaypointPush
from mavros_msgs.msg import Waypoint
from mavros_msgs.msg import WaypointList
from mavros_msgs.srv import SetMode, CommandLong, CommandInt, CommandBool, CommandHome, CommandTOL, CommandTriggerControl, CommandTriggerInterval

from sensor_msgs.msg import Image

class ArmTest(Node):
    def __init__(self):
        super().__init__('arm_node')
        srv_cmd_int = self.create_client(SetMode, "mavros/set_mode")
        srv_cmd_int.wait_for_service()

        # create reposition request
        command = SetMode.Request()
        #command.frame = 3 # global, relative altitude
        #command.command = 216 # https://mavlink.io/en/messages/common.html#MAV_CMD_DO_REPOSITION
        #command.current = 2 # indicate move to message in guided mode
        #command.x = int(46.6127 * 1e7)
        #command.y = int(14.2652 * 1e7)
        #command.z = 50.0 # altitude above ground
        command.base_mode = 216
        command.custom_mode = "GUIDED"
        print(command)

        future_command = srv_cmd_int.call_async(command)
        #if future_command.result().success:
        #    self.get_logger().info("Successfully send move to command to vehicle.")
        #else:
        #    self.get_logger().warn("Failed to send move to command to vehicle!")    

def main(args=None):
    rclpy.init(args=args)
    arm_test = ArmTest()
    rclpy.spin(arm_test)
    arm_test.destroy_node()
    rclpy.shutdown() 	



if __name__ == '__main__':
    main()
