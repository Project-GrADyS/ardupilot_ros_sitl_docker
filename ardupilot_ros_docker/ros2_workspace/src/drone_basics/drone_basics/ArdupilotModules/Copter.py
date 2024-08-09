
from .ArdupilotVehicle import ArdupilotVehicle

class Copter(ArdupilotVehicle):
    def __init__(self, node_name: str, SYSID: int):
        super().__init__(node_name, SYSID)
        pass