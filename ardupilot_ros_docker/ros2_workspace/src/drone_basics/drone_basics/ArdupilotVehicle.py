# base: https://github.com/autonomous-drone-project/drone_codes_cpp/blob/main/include/gnc_functions.hpp

from abc import ABC, abstractmethod
import rclpy

from rclpy.node import Node
from mavros_msgs.srv import SetMode, CommandLong, CommandInt, CommandBool, CommandHome, CommandTOL, CommandTriggerControl, CommandTriggerInterval

# usar metodos abstratos para definir os tipos de mensagem

class ArdupilotVehicle():
    def __init__(self, SYSID: int):
        self.sysid = SYSID

    def setArm(self, state: bool):
        self.srv_arm = self.create_client(CommandBool, "/mavros/cmd/arming")
        while not self.srv_arm.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('WARN: arm service not available!')

        self.msg_arming = CommandBool.Request(value=state)
        status = self.srv_arm.call_async(self.msg_arming)

    def setMode(self, mode: str):
        self.srv_set_mode = self.create_client(SetMode, "mavros/set_mode")
        while not self.srv_set_mode.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('WARN: set_mode service not available!')
        self.msg_flight_mode = SetMode.Request(base_mode=0, custom_mode=mode)
        status = self.srv_set_mode.call_async(self.msg_flight_mode)

    def goToPosition():
        pass

    def setTakeoff(takeoffAlt: float):
        pass

    def checkWaypointReached():
        pass

    def setSpeed():
        pass

    def setYaw():
        pass




    # getters
    def getPos():
        pass

    def getArmedState():
        pass

    def getCurrentPosition():
        pass

    


    def waitConnect():
        pass

    def waitStart():
        pass

    def initLocalFrame():
        pass

#This function is called at the beginning of a program and will start of the communication links to the FCU. The function requires the program's ros nodehandle as an input
    def initPublisherSubscriber():
        pass
