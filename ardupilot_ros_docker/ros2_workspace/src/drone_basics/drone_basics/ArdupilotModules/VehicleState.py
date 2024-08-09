import sys, os
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

from ArdupilotModules import ArdupilotVehicle
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandLong, CommandInt, CommandBool, CommandHome, CommandTOL, CommandTriggerControl, CommandTriggerInterval, StreamRate
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)


class VehicleState():
    def __init__(self, vehicle: ArdupilotVehicle) -> None:
        self.__vehicle = vehicle
        self.__connected = False
        self.__armed = False
        self.__guided = False
        self.__mode = "ALTHOLD"
        self.__manual_input = False
        self.__system_status = 0
    
        #subscription config
        self.__flightModeState = self.__vehicle.create_subscription(
            State,
            '/mavros/state',
            self.__callbackUpdateAtributes,
            qos_profile)
        self.__connected = False
        self.__armed = False
        self.__guided = False
        self.__mode = "GUIDED"
        self.__manual_input = False
        self.__system_status = 0



    #callback
    def __callbackUpdateAtributes(self, msg):
        self.__connected = msg.connected
        self.__armed = msg.armed
        self.__guided = msg.guided
        self.__mode = msg.mode
        self.__manual_input = msg.manual_input
        self.__system_status = msg.system_status

        #var clients
        self.__srv_stream_rate = None
        self.__srv_arm = None
        self.__srv_set_mode = None
        self.__srv_takeoff = None



    # getters
    def isConnected(self) -> bool:
        return self.__connected
    def isArmed(self) -> bool:
        return self.__armed
    def isGuided(self) -> bool:
        return self.__guided
    def getMode(self) -> str:
        return self.__mode
    def isManualInput(self) -> bool:
        return self.__manual_input
    def getSystemStatus(self) -> int:
        return self.__system_status

    # setters
    def setMavrosStreamRate(self, rate: int=10, id: int=0, isOn: bool=True) -> None:
        self.__vehicle.get_logger().info('INFO: Setting up MAVROS stream rate')
        self.__srv_stream_rate = self.__vehicle.create_client(StreamRate, "/mavros/set_stream_rate")
        while not self.__srv_stream_rate.wait_for_service(timeout_sec=1.0):
            self.__vehicle.get_logger().warning('WARN: waiting for the stream_rate service to become available!')
        self.msg_stream_rate = StreamRate.Request(stream_id=id, message_rate=rate, on_off=isOn)
        self.status = self.__srv_stream_rate.call_async(self.msg_stream_rate)
        self.__vehicle.get_logger().info(f'response:\n {self.status}')

    def setArm(self, enabled: bool) -> None:
        self.__srv_arm = self.__vehicle.create_client(CommandBool, "/mavros/cmd/arming")
        while not self.__srv_arm.wait_for_service(timeout_sec=1.0):
            self.__vehicle.get_logger().warning('WARN: Waiting for the arm service to become available!')
        self.msg_arming = CommandBool.Request(value=enabled)
        self.status = self.__srv_arm.call_async(self.msg_arming)
        #self.get_logger().info(f'response:\n {self.status}')

    def setMode(self, mode: str) -> None:
        self.__srv_set_mode = self.__vehicle.create_client(SetMode, "mavros/set_mode")
        while not self.__srv_set_mode.wait_for_service(timeout_sec=1.0):
            self.__vehicle.get_logger().warning('WARN: Waiting for the set_mode service to become available!')
        self.msg_flight_mode = SetMode.Request(base_mode=0, custom_mode=mode)
        self.status = self.__srv_set_mode.call_async(self.msg_flight_mode)
        self.__vehicle.get_logger().info(f'response:\n {self.status}')

    def setTakeoff(self, altitude: float, min_pitch: float=0.0, yaw: float=0.0, latitude: float=0.0, longitude: float=0.0):
        self.__srv_takeoff = self.__vehicle.create_client(CommandTOL, "/mavros/cmd/takeoff")
        while not self.__srv_takeoff.wait_for_service(timeout_sec=1.0):
            self.__vehicle.get_logger().warning('WARN: Waiting for the takeoff service to become available!')
        self.msg_takeoff = CommandTOL.Request(min_pitch=min_pitch, yaw=yaw, latitude=latitude, longitude=longitude, altitude=altitude)
        self.status = self.__srv_takeoff.call_async(self.msg_takeoff)
        self.__vehicle.get_logger().debug(f'response:\n {self.status}')