#!/usr/bin/env python
import rospy
import sys
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
#from termcolor import colored

#ROBOCOL
pos_x_info_anterior = 0
pos_y_info_anterior = 0
theta_info_anterior = 0
cont = 0
vel_lin_x_info = 0
vel_ang_z_info = 0
rho_info = 0
pos_x_info, pos_y_info, theta_info = 0,0,0
pos_x_final_info, pos_y_final_info, theta_final_info, llegoAlaMeta = 0,0,0,0

def cmd_vel_callback(msg):
	global vel_lin_x_info, vel_ang_z_info
	vel_lin_x_info = msg.linear.x
	vel_ang_z_info = msg.angular.z

def rho_callback(msg):
	global rho_info
	rho_info = msg.data

def pos_callback(msg):
	global pos_x_info_anterior, pos_y_info_anterior, theta_info_anterior, cont, pos_x_info, pos_y_info, theta_info

	pos_x_info = msg.linear.x
	pos_y_info = msg.linear.y
	theta_info = msg.angular.z

	if (abs(pos_x_info-pos_x_info_anterior) < 0.01 and abs(pos_y_info-pos_y_info_anterior) < 0.01 and abs(theta_info-theta_info_anterior) < 0.01):
		cont = cont + 1
	else:
		cont = 0

	pos_x_info_anterior = pos_x_info
	pos_y_info_anterior = pos_y_info
	theta_info_anterior = theta_info


def pos_final_callback(msg):
	global pos_x_final_info, pos_y_final_info, theta_final_info, llegoAlaMeta
	pos_x_final_info = msg.linear.x
	pos_y_final_info = msg.linear.y
	theta_final_info = msg.angular.z
	llegoAlaMeta = msg.linear.z


def info_status():
	global vel_lin_x_info, vel_ang_z_info, rho_info, pos_x_info_anterior, pos_y_info_anterior, theta_info_anterior, cont, pos_x_info, pos_y_info, theta_info, pos_x_final_info, pos_y_final_info, theta_final_info, llegoAlaMeta

	rospy.init_node('status', anonymous=True)  # Inicia el nodo status

	rospy.Subscriber("Robocol/MotionControl/cmd_vel", Twist, cmd_vel_callback, tcp_nodelay=True)
	rospy.Subscriber("Robocol/MotionControl/rho", Float32, rho_callback, tcp_nodelay=True)
	rospy.Subscriber("Robocol/MotionControl/pos", Twist, pos_callback, tcp_nodelay=True)
	rospy.Subscriber("Robocol/MotionControl/pos_final", Twist, pos_final_callback, tcp_nodelay=True)

	rate = rospy.Rate(10)


	while not rospy.is_shutdown():
		mensaje = 'vel. lin. x: ' + str(round(vel_lin_x_info,3)) + ' | vel. ang. z: ' + str(round(vel_ang_z_info,3)) + '\n'
		mensaje = mensaje + 'rho: ' + str(round(rho_info,3)) + '\n'
		mensaje = mensaje + 'pos. x: ' + str(pos_x_info) + ' | pos. y: ' + str(pos_y_info) + ' | theta: ' + str(theta_info) + '\n'
		mensaje = mensaje + 'pos. final x: ' + str(pos_x_final_info) + ' | pos. final y: ' + str(pos_y_final_info) + ' | theta final: ' + str(theta_final_info) + '\n'
		
		#print('vel. lin. x: ' + str(vel_lin_x_info) + ' | vel. ang. z: ' + str(vel_ang_z_info))
		#print('rho: ' + str(rho_info))
		#print('pos. x: ' + str(pos_x_info) + ' | pos. y: ' + str(pos_y_info) + ' | theta: ' + str(theta_info))
		#print('pos. final x: ' + str(pos_x_final_info) + ' | pos. final y: ' + str(pos_y_final_info) + ' | theta final: ' + str(theta_final_info))

		if (llegoAlaMeta == 1):
			#print('YA LLEGO A LA META')
			mensaje = mensaje + 'YA LLEGO A LA META' + '\n'
		else:
			#print('No ha llegado')
			mensaje = mensaje + 'No ha llegado' + '\n'

		if (cont >= 1000):
			#print('Detenido')
			mensaje = mensaje + 'Detenido'
		else:
			#print('En movimiento')
			mensaje = mensaje + 'En movimiento'

		print(mensaje)
		
		#sys.stdout.write("\033[K") # Clear to the end of line
		#sys.stdout.write("\033[F") # Cursor up one line
		time.sleep(1)
		#sys.stdout.flush()
		#rate.sleep()
	#rate.sleep()


if __name__== '__main__':
	try:
		info_status()
	except rospy.ROSInterruptException:
		print('Nodo detenido')