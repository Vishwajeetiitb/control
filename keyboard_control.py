#! /usr/bin/env python
import rospy
import keyboard  # using module keyboard
count = 1
import time
rospy.init_node("keyboard_control")
while True:
	 if keyboard.is_pressed('up'):
	 	print(count)
	 	count+=1
	 	time.sleep(0.1)