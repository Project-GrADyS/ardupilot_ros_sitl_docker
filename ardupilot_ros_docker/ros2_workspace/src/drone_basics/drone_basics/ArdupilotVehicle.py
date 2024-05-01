# base: https://github.com/autonomous-drone-project/drone_codes_cpp/blob/main/include/gnc_functions.hpp

from abc import ABC, abstractmethod
import rclpy

from rclpy.node import Node
from mavros_msgs.srv import SetMode, CommandLong, CommandInt, CommandBool, CommandHome, CommandTOL, CommandTriggerControl, CommandTriggerInterval, StreamRate

# usar metodos abstratos para definir os tipos de mensagem

class ArdupilotVehicle():
    def __init__(self, SYSID: int):
        self.sysid = SYSID

    def setMavrosStreamRate(self, id: int=0, rate: int=10, isOn: bool=True):
        self.get_logger().info('INFO: Setting up MAVROS stream rate')
        self.srv_stream_rate = self.create_client(StreamRate, "/mavros/set_stream_rate")
        while not self.srv_stream_rate.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning('WARN: waiting for the stream_rate service to become available!')
        self.msg_stream_rate = StreamRate.Request(stream_id=id, message_rate=rate, on_off=isOn)
        self.status = self.srv_stream_rate.call_async(self.msg_stream_rate)
        self.get_logger().info(f'response:\n {self.status}')

# ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 10, on_off: true}"
# requester: making request: mavros_msgs.srv.StreamRate_Request(stream_id=0, message_rate=10, on_off=True)

    def setArm(self, state: bool):
        self.srv_arm = self.create_client(CommandBool, "/mavros/cmd/arming")
        while not self.srv_arm.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning('WARN: Waiting for the arm service to become available!')
        self.msg_arming = CommandBool.Request(value=state)
        self.status = self.srv_arm.call_async(self.msg_arming)
        self.get_logger().info(f'response:\n {self.status}')

    def setMode(self, mode: str):
        self.srv_set_mode = self.create_client(SetMode, "mavros/set_mode")
        while not self.srv_set_mode.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning('WARN: Waiting for the set_mode service to become available!')
        self.msg_flight_mode = SetMode.Request(base_mode=0, custom_mode=mode)
        self.status = self.srv_set_mode.call_async(self.msg_flight_mode)
        self.get_logger().info(f'response:\n {self.status}')

    def goToPosition():
        pass

    def setTakeoff(self, altitude: float, min_pitch: float=0.0, yaw: float=0.0, latitude: float=0.0, longitude: float=0.0):
        self.srv_takeoff = self.create_client(CommandTOL, "/mavros/cmd/takeoff")
        while not self.srv_takeoff.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning('WARN: Waiting for the takeoff service to become available!')
        self.msg_takeoff = CommandTOL.Request(min_pitch=min_pitch, yaw=yaw, latitude=latitude, longitude=longitude, altitude=altitude)
        self.status = self.srv_takeoff.call_async(self.msg_takeoff)
        self.get_logger().debug(f'response:\n {self.status}')


    def checkWaypointReached():
        pass

    def setSpeed():
        pass

    def setYaw():
        pass




    # getters
    def getPos(self, data: Node):
        return Node

    def getArmedState(self):
        pass

    def getCurrentPosition():
        pass

    def getFlightMode(self):
        pass

    


    def waitConnect(self):
        self.get_logger().info('Waiting for FCU connect')

    def waitStartGuided(self):
        while(self.getFlightMode != "GUIDED"):
            self.get_logger().info('Waiting for GUIDED mode')

    def initLocalFrame():
        pass

#This function is called at the beginning of a program and will start of the communication links to the FCU. The function requires the program's ros nodehandle as an input
    def initPublisherSubscriber():
        pass
