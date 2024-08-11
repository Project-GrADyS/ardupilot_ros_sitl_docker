import sys
import os
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)
import rclpy
from rclpy.node import Node
from ArdupilotModules.Copter import Copter

# Constants
ALT = 4.0

class Line(Copter):
    """
    Line class inherits from Copter to implement a mission sequence for a drone.
    """
    def __init__(self, node_name: str, SYSID: int):
        super().__init__(node_name, SYSID)
        self.timerIsOn = False
        self.mission_timer = None
        self.mission = self.getMission()
        self.waypoint_index = 0
        self.goals = [
            [0, 40, ALT, 0], [40, 40, ALT, -90], [40, 0, ALT, -180],
            [0, 0, ALT, 90], [0, 0, ALT, 0] 
        ]
        self.mission.initialize_local_frame()
        
        # Create timers for callbacks
        self.check_alt_callback = self.create_timer(0.15, self.check_alt)
        self.state_callback = self.create_timer(0.15, self.arm_vehicle)

    def arm_vehicle(self):
        """
        Arms the vehicle and initiates takeoff if not already armed.
        """
        state = self.getState()
        mode = state.getMode()
        armed = state.isArmed()
        self.get_logger().info(f'ARMED TEST = {armed}')
        if not armed and mode == "GUIDED":
            state.setArm(True)
            state.setTakeoff(altitude=ALT)

    def check_alt(self):
        """
        Checks the altitude and starts the mission if the desired altitude is reached.
        """
        alt = self.getLocalPosition().getPose().getPosePosition().getZ()
        pos = self.getGlobalPosition().getLocal().getPosePosition()
        self.get_logger().info(f'GLOCALPOSITION: \n \
                               X: {pos.getX()}\n \
                               Y: {pos.getY()}\n \
                               Z: {pos.getZ()}\n')
        self.get_logger().info(f'LLOCALPOSITION: \n \
                               X: {self.getLocalPosition().getPose().getPosePosition().getX()}\n \
                               Y: {self.getLocalPosition().getPose().getPosePosition().getY()}\n \
                               Z: {self.getLocalPosition().getPose().getPosePosition().getZ()}\n')
        self.get_logger().info(f'enu_2_pos: \n \
                               X: {self.mission.enu_2_local().x}\n \
                               Y: {self.mission.enu_2_local().y}\n \
                               Z: {self.mission.enu_2_local().z}\n')
        if alt >= 0.95 * ALT and not self.timerIsOn:
            self.timerIsOn = True
            self.get_logger().info('RODOU 0')
            self.mission_timer = self.create_timer(0.15, self.run_mission)

    def activate_land(self):
        """
        Activates the LAND mode to land the vehicle.
        """
        self.get_logger().info('Activating LAND mode')
        self.getState().setMode("LAND")

    def run_mission(self):
        """
        Executes the mission by setting waypoints and lands the vehicle after completing the mission.
        """
        if self.waypoint_index < len(self.goals):
            goal = self.goals[self.waypoint_index]
            self.mission.set_destination(x=goal[0], y=goal[1], z=goal[2], psi=goal[3])
            if self.mission.check_waypoint_reached():
                self.waypoint_index += 1
        else:
            self.destroy_timer(self.mission_timer)
            self.timerIsOn = False
            self.activate_land()

def main(args=None):
    rclpy.init(args=args)

    drone = Line('line_node', 25)
    drone.getState().setMavrosStreamRate(6)
    drone.getState().setMode("GUIDED")
    rate = drone.create_rate(6)
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
