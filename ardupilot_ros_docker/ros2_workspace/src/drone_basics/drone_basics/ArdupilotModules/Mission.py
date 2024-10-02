# see https://github.com/mavlink/mavros/issues/1718
# and https://github.com/mavlink/mavros/issues/1867
# about conversion: https://masoudir.github.io/mavros_tutorial/Chapter1_ArduRover_with_CLI/Step1_How_to_change_mode/

import sys, os
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

from math import atan2, pow, sqrt, degrees, radians, sin, cos

from ArdupilotModules import ArdupilotVehicle
import rclpy
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from mavros_msgs.srv import WaypointPush # http://docs.ros.org/en/noetic/api/mavros_msgs/html/srv/WaypointPush.html
from mavros_msgs.msg import Waypoint, State # http://docs.ros.org/en/hydro/api/mavros/html/msg/Waypoint.html
from nav_msgs.msg import Odometry
from ArdupilotModules.TextColours import *

qos_profile = QoSProfile(
    reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    depth=1
)

class Mission():
    def __init__(self, vehicle: ArdupilotVehicle):
        self.__vehicle = vehicle
        self.__waypoint_list = []
        self.addWaypoint(0, 16, float(0), float(0), float(0),
                          float("nan"), 49.00000, 7.00000, float(0),
                          False, True) # set waypoint 0 -> home position(to be ignored)
        self.__request = None
        self.__waypoints_loaded = False

        self.__current_state_g = State()
        self.__current_pose_g = Odometry()
        self.__correction_vector_g = Pose()
        self.__local_offset_pose_g = Point()
        self.__waypoint_g = PoseStamped()

        self.__current_heading_g = 0.0
        self.__local_offset_g = 0.0
        self.__correction_heading_g = 0.0
        self.__local_desired_heading_g = 0.0
        
        #client
        self.srv_waypoint_push_client = self.__vehicle.create_client(WaypointPush, '/mavros/mission/push')
        while not self.srv_waypoint_push_client.wait_for_service(timeout_sec=1.0):
            self.__vehicle.get_logger().warning('Waypoint Push service is not available. Looking for /mavros/mission/push')
        
        #publisher
        self.__local_pos_pub = self.__vehicle.create_publisher(PoseStamped, "mavros/setpoint_position/local", 10)
        

        self.__init_control_index = 0

    def initialize_local_frame(self):
        if(self.__init_control_index <= 0):
            self.__init_loop = self.__vehicle.create_timer(0.2, self.initialize_local_frame)
            self.__local_offset_g = 0.0
        """This function will create a local reference frame based on the starting location of the drone. This is typically done right before takeoff. This reference frame is what all of the the set destination commands will be in reference to."""

        q0, q1, q2, q3 = (
            self.__vehicle.getGlobalPosition().getLocal().getPoseOrientation().getW(),
            self.__vehicle.getGlobalPosition().getLocal().getPoseOrientation().getX(),
            self.__vehicle.getGlobalPosition().getLocal().getPoseOrientation().getY(),
            self.__vehicle.getGlobalPosition().getLocal().getPoseOrientation().getZ(),
        )

        psi = atan2((2 * (q0 * q3 + q1 * q2)),
                    (1 - 2 * (pow(q2, 2) + pow(q3, 2))))

        self.__local_offset_g += degrees(psi)
        self.__local_offset_pose_g.x += self.__vehicle.getGlobalPosition().getLocal().getPosePosition().getX()
        self.__local_offset_pose_g.y += self.__vehicle.getGlobalPosition().getLocal().getPosePosition().getY()
        self.__local_offset_pose_g.z += self.__vehicle.getGlobalPosition().getLocal().getPosePosition().getZ()
        self.__init_control_index += 1
        
        if(self.__init_control_index >=29):
            self.__local_offset_pose_g.x /= 30.0
            self.__local_offset_pose_g.y /= 30.0
            self.__local_offset_pose_g.z /= 30.0
            self.__local_offset_g /= 30.0
            self.__vehicle.get_logger().info(CBLUE2 + "Coordinate offset set" + CEND)
            self.__vehicle.get_logger().info(
                CGREEN2 + "The X-Axis is facing: {}".format(self.__local_offset_g) + CEND)
            self.__vehicle.destroy_timer(self.__init_loop)

    def enu_2_local(self) -> Point:
        x, y, z = (
            self.__vehicle.getGlobalPosition().getLocal().getPosePosition().getX(),
            self.__vehicle.getGlobalPosition().getLocal().getPosePosition().getY(),
            self.__vehicle.getGlobalPosition().getLocal().getPosePosition().getZ(),
        )

        current_pos_local = Point()

        current_pos_local.x = x * cos(radians((self.__local_offset_g - 90))) - y * sin(
            radians((self.__local_offset_g - 90))
        )

        current_pos_local.y = x * sin(radians((self.__local_offset_g - 90))) + y * cos(
            radians((self.__local_offset_g - 90))
        )

        current_pos_local.z = z

        return current_pos_local

    def check_waypoint_reached(self, pos_tol=0.3, head_tol=0.01) -> int:
        """This function checks if the waypoint is reached within given tolerance and returns an int of 1 or 0. This function can be used to check when to request the next waypoint in the mission.

        Args:
                pos_tol (float, optional): Position tolerance under which the drone must be with respect to its position in space. Defaults to 0.3.
                head_tol (float, optional): Heading or angle tolerance under which the drone must be with respect to its orientation in space. Defaults to 0.01.

        Returns:
                1 (int): Waypoint reached successfully.
                0 (int): Failed to reach Waypoint.
        """

        q0, q1, q2, q3 = (
            self.__vehicle.getLocalPosition().getPose().getPoseOrientation().getW(),
            self.__vehicle.getLocalPosition().getPose().getPoseOrientation().getX(),
            self.__vehicle.getLocalPosition().getPose().getPoseOrientation().getY(),
            self.__vehicle.getLocalPosition().getPose().getPoseOrientation().getZ(),
        )

        psi = atan2((2 * (q0 * q3 + q1 * q2)),
                    (1 - 2 * (pow(q2, 2) + pow(q3, 2))))
        
        self.__current_heading_g = degrees(psi) - self.__local_offset_g

        self.__local_pos_pub.publish(self.__waypoint_g)

        dx = abs(
            self.__waypoint_g.pose.position.x - self.__vehicle.getGlobalPosition().getLocal().getPosePosition().getX()
        )
        dy = abs(
            self.__waypoint_g.pose.position.y - self.__vehicle.getGlobalPosition().getLocal().getPosePosition().getY()
        )
        dz = abs(
            self.__waypoint_g.pose.position.z - self.__vehicle.getGlobalPosition().getLocal().getPosePosition().getZ()
        )

        dMag = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2))

        cosErr = cos(radians(self.__current_heading_g)) - cos(
            radians(self.__local_desired_heading_g)
        )

        sinErr = sin(radians(self.__current_heading_g)) - sin(
            radians(self.__local_desired_heading_g)
        )

        dHead = sqrt(pow(cosErr, 2) + pow(sinErr, 2))
        self.__vehicle.get_logger().warning(f'dMag = {dMag} pos_tol = {pos_tol} dHead = {dHead} head_tol = {head_tol}')
        if dMag < pos_tol and dHead < head_tol:
            return 1
        else:
            return 0
    
    def set_heading(self, heading):
        """This function is used to specify the drone's heading in the local reference frame. Psi is a counter clockwise rotation following the drone's reference frame defined by the x axis through the right side of the drone with the y axis through the front of the drone.

        Args:
                heading (Float): θ(degree) Heading angle of the drone.
        """
        self.__local_desired_heading_g = heading
        heading = heading + self.__correction_heading_g + self.__local_offset_g

        self.__vehicle.get_logger().warning("The desired heading is {}".format(self.__local_desired_heading_g))
        yaw = radians(heading)
        pitch = 0.0
        roll = 0.0

        cy = cos(yaw * 0.5)
        sy = sin(yaw * 0.5)

        cr = cos(roll * 0.5)
        sr = sin(roll * 0.5)

        cp = cos(pitch * 0.5)
        sp = sin(pitch * 0.5)

        qw = cy * cr * cp + sy * sr * sp
        qx = cy * sr * cp - sy * cr * sp
        qy = cy * cr * sp + sy * sr * cp
        qz = sy * cr * cp - cy * sr * sp

        self.__waypoint_g.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

    def set_destination(self, x, y, z, psi):
        """This function is used to command the drone to fly to a waypoint. These waypoints should be specified in the local reference frame. This is typically defined from the location the drone is launched. Psi is counter clockwise rotation following the drone's reference frame defined by the x axis through the right side of the drone with the y axis through the front of the drone.

        Args:
                x (Float): x(m) Distance with respect to your local frame.
                y (Float): y(m) Distance with respect to your local frame.
                z (Float): z(m) Distance with respect to your local frame.
                psi (Float): θ(degree) Heading angle of the drone.
        """
        self.set_heading(psi)

        theta = radians((self.__correction_heading_g + self.__local_offset_g - 90))

        Xlocal = x * cos(theta) - y * sin(theta)
        Ylocal = x * sin(theta) + y * cos(theta)
        Zlocal = z

        x = Xlocal + self.__correction_vector_g.position.x + self.__local_offset_pose_g.x

        y = Ylocal + self.__correction_vector_g.position.y + self.__local_offset_pose_g.y

        z = Zlocal + self.__correction_vector_g.position.z + self.__local_offset_pose_g.z

        self.__vehicle.get_logger().warning("Destination set to x:{} y:{} z:{} origin frame!!!!!!!!!".format(x, y, z))

        self.__waypoint_g.pose.position = Point(x=x, y=y, z=z)
        self.__local_pos_pub.publish(self.__waypoint_g)

    def addWaypoint(self, frame: int, command: int, param1: float, param2: float, param3: float, param4: float, x_lat: float, y_long: float, z_alt: float, is_current: bool=False, autocontinue: bool=True) -> None:
        waypoint = Waypoint()
        waypoint.frame = frame
        waypoint.command = command
        waypoint.param1 = param1
        waypoint.param2 = param2
        waypoint.param3 = param3
        waypoint.param4 = param4
        waypoint.x_lat = x_lat
        waypoint.y_long = y_long
        waypoint.z_alt = z_alt
        waypoint.is_current = is_current
        waypoint.autocontinue = autocontinue
        self.__waypoint_list.append(waypoint)
    
    def sendWaypointList(self) -> bool:
        if(not self.__waypoints_loaded):
            self.__request = WaypointPush.Request()
            self.__request.start_index = 0
            for waypoint in self.__waypoint_list:
                self.__request.waypoints.append(waypoint)
        self.__waypoints_loaded = True
        self.__vehicle.get_logger().info('Waypoint sent %s.' %(str(self.__waypoint_list)))
        self.__future = self.srv_waypoint_push_client.call_async(self.__request)
        """rclpy.spin_until_future_complete(self.__vehicle, self.__future)
        if self.__future.result() is not None:
            self.__vehicle.get_logger().info('Waypoints sent successfully.')  # Log de sucesso
            return True
        else:
            self.__vehicle.get_logger().error('Failed to send waypoints.')  # Log de falha
            return False """