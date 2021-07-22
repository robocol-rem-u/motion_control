#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
#from termcolor import colored

#ROBOCOL
pos_x_info_anterior = 0
pos_y_info_anterior = 0
theta_info_anterior = 0
cont = 0


def cmd_vel_callback(msg):
	vel_lin_x_info = msg.linear.x
	vel_ang_z_info = msg.angular.z
	print('vel. lin. x: ' + str(vel_lin_x_info) + ' | vel. ang. z: ' + str(vel_ang_z_info))

def rho_callback(msg):
	rho_info = msg.data
	print('rho: ' + str(rho_info))

def pos_callback(msg):
	global pos_x_info_anterior, pos_y_info_anterior, theta_info_anterior, cont

	pos_x_info = msg.linear.x
	pos_y_info = msg.linear.y
	theta_info = msg.angular.z

	if (abs(pos_x_info-pos_x_info_anterior) < 0.0001 and abs(pos_y_info-pos_y_info_anterior) < 0.001 and abs(theta_info-theta_info_anterior) < 0.001):
		#print colored('Detenido', 'red', attrs=['reverse', 'blink'])
		cont = cont + 1
		#print('Detenido')
	else:
		#print colored('En movimiento', 'green', attrs=['reverse', 'blink'])
		cont = 0
		#print('En movimiento')

	if (cont >= 1000):
		print('Detenido')
	

	pos_x_info_anterior = pos_x_info
	pos_y_info_anterior = pos_y_info
	theta_info_anterior = theta_info

	print('pos. x: ' + str(pos_x_info) + ' | pos. y: ' + str(pos_y_info) + ' | theta: ' + str(theta_info))


def info_status():
	rospy.init_node('status', anonymous=True)  # Inicia el nodo status

	rospy.Subscriber("Robocol/MotionControl/cmd_vel", Twist, cmd_vel_callback, tcp_nodelay=True)
	rospy.Subscriber("Robocol/MotionControl/rho", Float32, rho_callback, tcp_nodelay=True)
	rospy.Subscriber("Robocol/MotionControl/pos", Twist, pos_callback, tcp_nodelay=True)


	rate = rospy.Rate(10)


	print('Esperando comando...')
	while not rospy.is_shutdown():
		

		rate.sleep()
	#rate.sleep()


if __name__== '__main__':
	try:
		info_status()
	except rospy.ROSInterruptException:
		print('Nodo detenido')