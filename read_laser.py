#! /usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
rospy.init_node("read_laser")
pub = rospy.Publisher("/cmd_vel",Twist)
control = Twist()
def read(msg):
	regions = {
            'right':  min(min(msg.ranges[0:144]), 10),
            'fright': min(min(msg.ranges[145:288]), 10),
            'front':  min(min(msg.ranges[289:432]), 10),
            'fleft':  min(min(msg.ranges[433:576]), 10),
            'left':   min(min(msg.ranges[577:720]), 10),
        }
	print(regions['front'])        
	  

sub = rospy.Subscriber("/rrbot/laser/scan",LaserScan,read)
rospy.spin()