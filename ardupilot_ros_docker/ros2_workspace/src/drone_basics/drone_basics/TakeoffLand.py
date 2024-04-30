
import sys, os
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

from ArdupilotVehicle import ArdupilotVehicle

import rclpy
from rclpy.node import Node

class TakeoffLand(Node, ArdupilotVehicle):
    def __init__(self):
        super().__init__('takeoff_land_node')
        self.waitConnect()
        self.waitStartGuided()
        self.setTakeoff(altitude=10)
        