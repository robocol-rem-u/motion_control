#!/usr/bin/env python
import rospy
import roslib
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Bool
from rospy_tutorials.msg import Floats
import numpy as np


def talker():
	pub = rospy.Publisher('Robocol/MotionControl/ruta', numpy_msg(Floats),queue_size=10)
	pub2 = rospy.Publisher('Robocol/MotionControl/flag_hayRuta', Bool,queue_size=1)
		
	rospy.init_node('prueba_talker', anonymous=True)
	r = rospy.Rate(1000) # 10hz
	xd = 0
	while not rospy.is_shutdown():
		if xd == 0:
		#a = np.array([[1,2,3,4,5,6,7,8,9,10,11,12,13,14]], dtype=np.float32)
			a = np.array([1, 2, 3, 4, 5, 6,7,8,9,10,11,12], dtype=np.float32)
		#a = np.array([[1,2,3],[4,5,6],[7,8,9]])
			pub.publish(a)
			b = 1
			pub2.publish(b)
			
			xd = 1
		r.sleep()
	

if __name__ == '__main__':
	talker()









