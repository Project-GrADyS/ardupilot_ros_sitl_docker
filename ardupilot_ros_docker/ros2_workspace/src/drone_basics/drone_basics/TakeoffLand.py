
import sys
import os
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)
import rclpy
from rclpy.node import Node
from ArdupilotModules.Copter import Copter

ALT = 15.0
PERIOD = 6
FREQUENCY = round(1/PERIOD, 2)

class TakeoffLand(Copter):
    def __init__(self, node_name: str, SYSID: int):
        super().__init__(node_name, SYSID)
        self.timerIsOn = False
        self.land_timer = None
        
        self.check_alt_callback = self.create_timer(FREQUENCY, self.checkAlt)
        self.state_callback = self.create_timer(FREQUENCY, self.armVehicle)  # update every 0.02 seconds

    def armVehicle(self):
        state = self.getState()
        mode = state.getMode()
        armed = state.isArmed()
        self.get_logger().info(f'ARMED = {armed}')
        if not armed and mode == "GUIDED":
            state.setArm(True)
            state.setTakeoff(altitude=ALT)

    def checkAlt(self):
        alt = self.getLocalPosition().getPose().getPosePosition().getZ()
        #self.get_logger().info(f'alt data: {alt}')
        pos = self.getGlobalPosition().getLocal().getPosePosition()
        self.get_logger().info(f'POSITION: \n \
                               X: {pos.getX()}\n \
                               Y: {pos.getY()}\n \
                               Z: {pos.getZ()}\n')
        if alt >= 0.95 * ALT and not self.timerIsOn:
            self.timerIsOn = True
            self.land_timer = self.create_timer(8.0, self.activate_land)  # Sets the timer to call the function that activates LAND mode after 8 seconds

    def activate_land(self):
        self.get_logger().info('Activating LAND mode')
        self.getState().setMode("LAND")
        self.destroy_timer(self.land_timer)
        self.timerIsOn = False

def main(args=None):
    rclpy.init(args=args)

    drone = TakeoffLand('takeoff_land_node', 25)
    drone.getState().setMavrosStreamRate(PERIOD)
    drone.getState().setMode("GUIDED")
    rate = drone.create_rate(PERIOD)

    try:
        while rclpy.ok():
            drone.get_logger().warning('Beginning client, shut down with CTRL-C')
            rclpy.spin(drone)
            rate.sleep()
    except KeyboardInterrupt:
        drone.get_logger().info('Keyboard interrupt, shutting down.\n')
    finally:
        drone.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
