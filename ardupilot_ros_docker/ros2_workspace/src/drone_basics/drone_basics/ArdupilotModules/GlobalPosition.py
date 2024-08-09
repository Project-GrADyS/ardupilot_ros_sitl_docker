import sys, os
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64, UInt32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from ArdupilotModules import ArdupilotVehicle
from PositionModel import *

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

class CompassHdg():
    def __init__(self, vehicle: ArdupilotVehicle):
        self.__vehicle = vehicle

        #subscription config
        self.__compass_subscriber = self.__vehicle.create_subscription(
            Float64,
            '/mavros/global_position/compass_hdg',
            self.__callbackUpdateAtributes,
            qos_profile)
        self.__data = 0.0
    
    #callback
    def __callbackUpdateAtributes(self, msg):
        self.__data = msg.data
    def getData(self) -> float:
        return self.__data


class Global():
    def __init__(self, vehicle: ArdupilotVehicle):
        self.__vehicle = vehicle
        self.__cordinate = Coordinate()

        #subscription config
        self.__global_subscriber = self.__vehicle.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.__callbackUpdateAtributes,
            qos_profile)
    
    #callback
    def __callbackUpdateAtributes(self, msg):
        self.__cordinate.setCoordinate(msg.latitude, msg.longitude, msg.altitude)
    
    #getters
    def getCoordinate(self) -> Coordinate:
        return self.__cordinate
    
    ##faltando covariance

class Local():
    def __init__(self, vehicle: ArdupilotVehicle):
        self.__vehicle = vehicle
        self.__pose_position = Position()
        self.__pose_orientation = Orientation()
        # faltando implementar covariance
        self.__twist_linear = Position()
        self.__twist_angular = Position()

    
        #subscription config
        self.__local_subscriber = self.__vehicle.create_subscription(
            Odometry,
            '/mavros/global_position/local',
            self.__callbackUpdateAtributes,
            qos_profile)
            
    
    #callback
    def __callbackUpdateAtributes(self, msg):
        self.__pose_position.setPosition(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
        self.__pose_orientation.setPosition(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        self.__twist_linear.setPosition(msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z)
        self.__twist_angular.setPosition(msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z)

    # getters
    def getPosePosition(self) -> Position:
        return self.__pose_position
    def getPoseOrientation(self) -> Orientation:
        return self.__pose_orientation
    def getTwistLinear(self) -> Position:
        return self.__twist_linear
    def getTwistAngular(self) -> Position:
        return self.__twist_angular


class GpsVel():
    def __init__(self, vehicle: ArdupilotVehicle):
        self.__vehicle = vehicle
        self.__twist_linear = Position()
        self.__twist_angular = Position()

        #subscription config
        self.__gps_vel_subscriber = self.__vehicle.create_subscription(
            TwistStamped,
            '/mavros/global_position/raw/gps_vel',
            self.__callbackUpdateAtributes,
            qos_profile)

    #callback
    def __callbackUpdateAtributes(self, msg):
        self.__twist_linear.setPosition(msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z)
        self.__twist_linear.setPosition(msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z)

    def getTwistLinear(self) -> Position:
        return self.__twist_linear
    def getTwistAngular(self) -> Position:
        return self.__twist_angular

class Satellites():
    def __init__(self, vehicle: ArdupilotVehicle):
        self.__vehicle = vehicle

        #subscription config
        self.__compass_subscriber = self.__vehicle.create_subscription(
            UInt32,
            '/mavros/global_position/raw/satellites',
            self.__callbackUpdateAtributes,
            qos_profile)
        self.__data = 0
    
    #callback
    def __callbackUpdateAtributes(self, msg):
        self.__data = msg.data
    def getData(self) -> int:
        return self.__data
    



class GlobalPosition():
    def __init__(self, vehicle: ArdupilotVehicle):
        self.__vehicle = vehicle
        self.__compass_hdg = CompassHdg(self.__vehicle)
        self.__global = Global(self.__vehicle)
        self.__local = Local(self.__vehicle)
        self.__gps_vel = GpsVel(self.__vehicle)
        self.__satellites = Satellites(self.__vehicle)

    def getCompassHdg(self) -> CompassHdg:
        return self.__compass_hdg
    def getGlobal(self) -> Global:
        return self.__global
    def getLocal(self) -> Local:
        return self.__local
    def getGpsVel(self) -> GpsVel:
        return self.__gps_vel
    def getSatellites(self) -> Satellites:
        return self.__satellites