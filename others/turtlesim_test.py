#!/usr/bin/env python 
# Set linear and angular values of Turtlesim's speed and turning. 

import rospy    # Needed to create a ROS node 
    # Message that moves base   
from turtlesim.msg import Pose

class ReadTurtlesim: 
        def __init__(self): 
                
                # ControlTurtlesim is the name of the node sent to the master
                rospy.init_node('TurtleSubcribe', anonymous=False) 
                # Message to screen 
                rospy.loginfo(" Press CTRL+c to stop TurtleBot") 
                # Keys CNTL + c will stop script 
                rospy.on_shutdown(self.shutdown) 
            # Publisher will send Twist message on topic 
            # /turtle1/cmd_vel 
                self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.callbackPose)
                rospy.spin()
        def callbackPose(self,msg):
                print("test")
                print(msg)
        

        def shutdown(self): 
                # You can stop turtlebot by publishing an empty Twist message 
                rospy.loginfo("Stopping Turtlesim") 
        # Give TurtleBot time to stop
        rospy.sleep(1) 


if __name__== "__main__": 
    try:
        ReadTurtlesim() 
    except: 
    	rospy.loginfo("End of the trip for Turtlesim") 

