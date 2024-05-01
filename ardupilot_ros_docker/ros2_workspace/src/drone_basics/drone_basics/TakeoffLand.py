
import sys, os
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

from ArdupilotVehicle import ArdupilotVehicle
import rclpy
from rclpy.node import Node
#from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

ALT = 30.0

class TakeoffLand(Node, ArdupilotVehicle):
    def __init__(self):
        super().__init__('takeoff_land_node')
        self.timerIsOn = False
        self.land_timer = None
        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.setMavrosStreamRate()
        self.position_subscriber = self.create_subscription(
            Float64,
            '/mavros/global_position/rel_alt',
            self.listener_callback,
            qos_profile)
        

        #self.waitConnect()
        self.setMode("GUIDED")
        #self.waitStartGuided()
        self.setArm(True)
        self.setTakeoff(altitude=ALT)
        #self.get_logger().info(f'altitude = {self.position_subscriber}')

    def checkAlt(self, alt):
        self.get_logger().info('alt data: "%f"' % alt)
        if alt >= 0.95 * ALT and not self.timerIsOn:
            self.timerIsOn = True
            self.land_timer = self.create_timer(8.0, self.activate_land)  # Configura o temporizador para chamar a função que ativa o modo LAND após 5 segundos

    def activate_land(self):
         self.get_logger().info('Activating LAND mode')
         self.setMode("LAND")
         self.destroy_timer(self.land_timer)

    def listener_callback(self, msg):
        self.checkAlt(msg.data)


def main(args=None):
    rclpy.init(args=args)

    takeoff_land = TakeoffLand()
#rclpy.ok():
    try:
        takeoff_land.get_logger().info('Beginning client, shut down with CTRL-C')
        rclpy.spin(takeoff_land)
    except KeyboardInterrupt:
        takeoff_land.get_logger().info('Keyboard interrupt, shutting down.\n')
    takeoff_land.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()