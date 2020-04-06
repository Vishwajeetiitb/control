#! /usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
rospy.init_node("avoid")
pub = rospy.Publisher("/cmd_vel",Twist)


def take_action(regions):
    control = Twist()
 
    
    state_description = ''
    
    if regions['front'] > 1 and regions['left'] > 0.5 and regions['right'] > 0.5:
        state_description = 'case 1 - nothing'
        linear_x = -0.5
        angular_z = 0
    elif regions['front'] < 1 and regions['left'] > 0.5 and regions['right'] > 0.5:
        state_description = 'case 2 - front'
        linear_x = 0
        angular_z = 0.8
    elif regions['front'] > 1 and regions['left'] > 0.5 and regions['right'] < 0.5:
        state_description = 'case 3 - fright'
        linear_x = 0
        angular_z = -0.8
    elif regions['front'] > 1 and regions['left'] < 0.5 and regions['right'] > 0.5:
        state_description = 'case 4 - fleft'
        linear_x = 0
        angular_z = 0.8
    elif regions['front'] < 1 and regions['left'] > 0.5 and regions['right'] < 0.5:
        state_description = 'case 5 - front and fright'
        linear_x = 0
        angular_z = -0.8
    elif regions['front'] < 1 and regions['left'] < 0.5 and regions['right'] > 0.5:
        state_description = 'case 6 - front and fleft'
        linear_x = 0
        angular_z = 0.8
    elif regions['front'] < 1 and regions['left'] < 0.5 and regions['right'] < 0.5:
        state_description = 'case 7 - front and fleft and fright'
        linear_x = 0
        angular_z = 0.8
    elif regions['front'] > 1 and regions['left'] < 0.5 and regions['right'] < 0.5:
        state_description = 'case 8 - fleft and fright'
        linear_x = -0.5
        angular_z = 0
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)

    rospy.loginfo(state_description)
    control.linear.x = -linear_x
    control.angular.z = angular_z
    pub.publish(control)


def read(msg):
	regions = {
            'right':  min(min(msg.ranges[0:144]), 10),
            'fright': min(min(msg.ranges[145:288]), 10),
            'front':  min(min(msg.ranges[289:432]), 10),
            'fleft':  min(min(msg.ranges[433:576]), 10),
            'left':   min(min(msg.ranges[577:720]), 10),
        }
	take_action(regions)
	   
	 

sub = rospy.Subscriber("/rrbot/laser/scan",LaserScan,read)
rospy.spin()