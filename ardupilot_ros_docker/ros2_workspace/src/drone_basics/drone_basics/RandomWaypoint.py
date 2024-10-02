import sys
import os
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)
import rclpy
from rclpy.node import Node
from ArdupilotModules.Copter import Copter

import random
import math

# Constants
ALT = 5.0
PERIOD = 6
FREQUENCY = round(1/PERIOD, 2)
#range fence limits
DISTANCE_LOWER_LIMIT = 0
DISTANCE_UPPER_LIMIT = 100

class RandomWaypoint(Copter):
    """
    Line class inherits from Copter to implement a mission sequence for a drone.
    """
    def __init__(self, node_name: str, SYSID: int):
        super().__init__(node_name, SYSID)
        self.timerIsOn = False
        self.mission_timer = None
        self.mission = self.getMission()

        #Initial random route
        self.random_value = (random.randint(DISTANCE_LOWER_LIMIT, DISTANCE_UPPER_LIMIT), 
                             random.randint(DISTANCE_LOWER_LIMIT, DISTANCE_UPPER_LIMIT))
        self.calc_direction()

        self.mission.initialize_local_frame()
        
        # Create timers for callbacks
        self.check_alt_callback = self.create_timer(FREQUENCY, self.check_alt)
        self.state_callback = self.create_timer(FREQUENCY, self.arm_vehicle)

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
        self.get_logger().info(f'POSITION: \n \
                               X: {pos.getX()}\n \
                               Y: {pos.getY()}\n \
                               Z: {pos.getZ()}\n')
        if alt >= 0.95 * ALT and not self.timerIsOn:
            self.timerIsOn = True
            self.mission_timer = self.create_timer(FREQUENCY, self.run_mission)

    def calc_direction(self) -> float:
        """
        Calculates the direction of the destination in relation to the origin with reference to the y-axis signal values ​​compatible with those entered in the set_destination function
        """
        current_pos = self.getGlobalPosition().getLocal().getPosePosition()

        origin = (current_pos.getX(), current_pos.getY())
        destination = (self.random_value[0], self.random_value[1])

        delta_x = destination[0] - origin[0]
        delta_x = -delta_x
        delta_y = destination[1] - origin[1]
        
        # Calculates the angle in radians
        theta_rad = math.atan2(delta_x, delta_y)

        # Converts the angle to degrees and snaps to the y-axis reference
        theta_deg = math.degrees(theta_rad)
        self.destination_psi = theta_deg

    def run_mission(self):
        """
        Executes the mission by setting waypoints
        """
        self.mission.set_destination(x=self.random_value[0], 
                                     y=self.random_value[1], 
                                     z=ALT, 
                                     psi=self.destination_psi)
        print(f"VALORES_ALEATORIOS: X={self.random_value[0]} Y={self.random_value[1]}")
        if self.mission.check_waypoint_reached(pos_tol=0.6):
            self.random_value = (random.randint(DISTANCE_LOWER_LIMIT, DISTANCE_UPPER_LIMIT), 
                                 random.randint(DISTANCE_LOWER_LIMIT, DISTANCE_UPPER_LIMIT))
            self.calc_direction()


def main(args=None):
    rclpy.init(args=args)

    drone = RandomWaypoint('random_waypoint_node', 25)
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
