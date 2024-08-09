# base: https://github.com/autonomous-drone-project/drone_codes_cpp/blob/main/include/gnc_functions.hpp

import sys, os
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

from abc import ABC, abstractmethod


from rclpy.node import Node
from .VehicleState import VehicleState
from .GlobalPosition import GlobalPosition
from .LocalPosition import LocalPosition
from .Mission import Mission

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# usar metodos abstratos para definir os tipos de mensagem

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

class ArdupilotVehicle(Node):
    def __init__(self, node_name: str, SYSID: int):
        super().__init__(node_name)
        
        self._sysid = SYSID
        self.__state = VehicleState(self)
        self.__global_position = GlobalPosition(self)
        self.__local_position = LocalPosition(self)
        self.__mission = Mission(self)

    def getSYSID(self) -> int:
        return self._sysid
    def getState(self) -> VehicleState:
        return self.__state
    def getGlobalPosition(self) -> GlobalPosition:
        return self.__global_position
    def getLocalPosition(self) -> LocalPosition:
        return self.__local_position
    def getMission(self) -> Mission:
        return self.__mission