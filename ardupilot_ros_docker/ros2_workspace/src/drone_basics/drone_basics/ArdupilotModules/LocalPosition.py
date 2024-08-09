import sys, os
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from ArdupilotModules import ArdupilotVehicle
from PositionModel import *

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)


class Odom():
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
            '/mavros/local_position/odom',
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
    

class Pose():
    def __init__(self, vehicle: ArdupilotVehicle):
        self.__vehicle = vehicle
        self.__pose_position = Position()
        self.__pose_orientation = Orientation()
        # faltando implementar covariance
        
        #subscription config
        self.__local_subscriber = self.__vehicle.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.__callbackUpdateAtributes,
            qos_profile)

    #callback
    def __callbackUpdateAtributes(self, msg):
        self.__pose_position.setPosition(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        self.__pose_orientation.setPosition(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)

    # getters
    def getPosePosition(self) -> Position:
        return self.__pose_position
    def getPoseOrientation(self) -> Orientation:
        return self.__pose_orientation

class VelocityBody():
    def __init__(self, vehicle: ArdupilotVehicle):
        self.__vehicle = vehicle
        self.__twist_linear = Position()
        self.__twist_angular = Position()

        #subscription config
        self.__local_subscriber = self.__vehicle.create_subscription(
            TwistStamped,
            '/mavros/local_position/velocity_body',
            self.__callbackUpdateAtributes,
            qos_profile)
    
    #callback
    def __callbackUpdateAtributes(self, msg):
        self.__twist_linear.setPosition(msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z)
        self.__twist_angular.setPosition(msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z)

    # getters
    def getTwistLinear(self) -> Position:
        return self.__twist_linear
    def getTwistAngular(self) -> Position:
        return self.__twist_angular

class VelocityLocal():
    def __init__(self, vehicle: ArdupilotVehicle):
        self.__vehicle = vehicle
        self.__twist_linear = Position()
        self.__twist_angular = Position()

        #subscription config
        self.__local_subscriber = self.__vehicle.create_subscription(
            TwistStamped,
            '/mavros/local_position/velocity_local',
            self.__callbackUpdateAtributes,
            qos_profile)
    
     #callback
    def __callbackUpdateAtributes(self, msg):
        self.__twist_linear.setPosition(msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z)
        self.__twist_angular.setPosition(msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z)

    # getters
    def getTwistLinear(self) -> Position:
        return self.__twist_linear
    def getTwistAngular(self) -> Position:
        return self.__twist_angular

class Accel():
    def __init__(self, vehicle: ArdupilotVehicle):
        self.__vehicle = vehicle
        # IMPLEMENTAR 


class LocalPosition():
    def __init__(self, vehicle: ArdupilotVehicle):
        self.__vehicle = vehicle
        self.__odom = Odom(self.__vehicle)
        self.__pose = Pose(self.__vehicle)
        self.__velocity_body = VelocityBody(self.__vehicle)
        self.__velocity_local = VelocityLocal(self.__vehicle)
        self.__accel = Accel(self.__vehicle)

    def getOdom(self) -> Odom:
        return self.__odom
    def getPose(self) -> Pose:
        return self.__pose
    def getVelocityBody(self) -> VelocityBody:
        return self.__velocity_body
    def getVelocityLocal(self) -> VelocityLocal:
        return self.__velocity_local
    def getAccel(self) -> Accel:
        return self.__accel