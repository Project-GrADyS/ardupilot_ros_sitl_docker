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



import rclpy
from rclpy.node import Node
from mavros_msgs.srv import WaypointPush
from mavros_msgs.msg import Waypoint
from mavros_msgs.msg import WaypointList
from mavros_msgs.srv import SetMode, CommandLong, CommandInt, CommandBool, CommandHome, CommandTOL, CommandTriggerControl, CommandTriggerInterval

from sensor_msgs.msg import Image

class ArmTest(Node):
    def __init__(self):
        super().__init__('simple_pub_sub')
        #self.publisher = self.create_publisher(SetMode, '/mavros/SetMode', 10)
        #self.subscription = self.create_subscription(
        #    Image, '/image_raw', self.img_callback, 10)
        self.f_mode_client = self.create_client(SetMode, '/mavros/mission/FlightMode')
        while not self.f_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetMode.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.ckli
    

def main(args=None):
    rclpy.init(args=args)
    arm_test = ArmTest()
    rclpy.spin(arm_test)
    arm_test.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()