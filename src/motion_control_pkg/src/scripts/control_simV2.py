#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String, Float32MultiArray, Float32
from geometry_msgs.msg import *
from std_msgs.msg import Bool
import sys
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion

import roslib
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

global pos_x, pos_y, theta, deltaX, deltaY
global rate, rho, auto, ruta, hayRuta

#RECIBE LAS INDICACIONES PARA HACER EL RECORRIDO Y MUEVE AL ROBOT - VERSION 4
#ROBOCOL

pos_x, pos_y, theta = 0, 0, 0

deltaX, deltaY = 0, 0

rho = 0
auto = 0
hayRuta = 1 #poner en cero cuando se vaya a probar cn planeacion
ruta = np.array([])

def position_callback(msg): #Me regresa la posicion en el marco inercial del robot
	global rate, pos_x, pos_y, theta

	pos_x = msg.pose.pose.position.x
	pos_y = msg.pose.pose.position.y
	pos_z = msg.pose.pose.position.z

	orC_x = msg.pose.pose.orientation.x
	orC_y = msg.pose.pose.orientation.y
	orC_z = msg.pose.pose.orientation.z
	orC_w = msg.pose.pose.orientation.w


	#orientation_q = msg.pose.pose.orientation
	orientation_list = [orC_x, orC_y, orC_z, orC_w]
	(roll, pitch,theta) = euler_from_quaternion(orientation_list)

def planeacion_callback(ruta):
	pass
	

def habilitarMov(msg): #Me indica si debo mover el robot autonomamente o no
	global auto
	auto = msg.data
	print('auto:' + str(auto))


def ruta_callback(msg):
	global ruta, hayRuta
	hayRuta = 1
	#a = np.zeros((3,3))
	print('msg.data y len:')
	print(msg.data)
	
	aux = msg.data
	ruta = aux.copy()

	ruta.resize(len(msg.data)/2, 2)
	print('ruta: ')
	print(ruta)
	print(len(ruta))


# def hayRuta_callback(msg): #Me indica si ya hay ruta o no
# 	global hayRuta
# 	hayRuta = msg.data
# 	print('hayRuta: ' + str(auto))

#Funcion principal de movimiento
def main_control():
	global pos_x, pos_y, theta, deltaX, deltaY, rho, rate, auto, hayRuta, ruta

	endPos = [0,0,0] #Posicion final por defecto

	rospy.init_node('control', anonymous=True) #Inicio nodo

	pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	rate = rospy.Rate(10) #10hz
	rospy.Subscriber("zed2/odom", Odometry, position_callback, tcp_nodelay=True)
	rospy.Subscriber('Robocol/MotionControl/flag_autonomo',Bool,habilitarMov, tcp_nodelay=True)
	rospy.Subscriber("Robocol/MotionControl/ruta", numpy_msg(Floats), ruta_callback, tcp_nodelay=True)
	#rospy.Subscriber('Robocol/MotionControl/flag_hayRuta',Bool,hayRuta_callback, tcp_nodelay=True)

	#ruta = np.array([[-0.3,-0.3], [-0.3,0.3]])
	ruta = np.array([[2.5,0.019],[1.5,1],[-0.035,0.0189]])

	vel_robot = Twist()

	rho = np.sqrt(endPos[0]**2 + endPos[1]**2)
	alpha = -theta + np.arctan2(endPos[1], endPos[0])
	beta = -theta - alpha

	K_rho = 0.15
	K_alpha = 0.4
	K_beta = -0.0


	while not rospy.is_shutdown():
		empezarDeNuevo = False
		v_vel = 0
		w_vel = 0
		vel_robot.linear.x = 0
		vel_robot.linear.y = 0
		vel_robot.angular.z = 0
		pub.publish(vel_robot)
		
		v_x = 0
		v_y = 0
		v_omega = 0

		print('Hay ruta?  ' + ('Si' if hayRuta == 1 else 'No'))
		
		print('Ruta: ' + str(ruta))

		while hayRuta == 1:

			for coord in ruta:
				coord_x = coord[0]
				coord_y = coord[1]
				auxtheta = np.arctan2(float(coord_y), float(coord_x))
				endPos = [float(coord_x), float(coord_y), float(auxtheta) ] # [X. Y, THETA]

				deltaX = endPos[0] - pos_x
				deltaY = endPos[1] - pos_y
				deltatheta = endPos[2] - theta

				rho = np.sqrt(deltaX**2 + deltaY**2)
				alpha = -theta + np.arctan2(deltaY, deltaX)

				beta = 0

				empezarDeNuevo = True
				while empezarDeNuevo == True:
					while abs(alpha) > 0.05:
						if auto == True:
							print('EndPos: ' + str(endPos[0]) + ' ' + str(endPos[1]) + ' ' + str(round(endPos[2],3)) + ' | rho: ' + str(round(rho,3)) + ' | ALFA: ' + str(round(alpha,3)) + ' | Pose: ' + str(round(pos_x,3)) + ', ' + str(round(pos_y,3)) + ', ' + str(round(theta,3)))
							deltaX = endPos[0] - pos_x
							deltaY = endPos[1] - pos_y
							deltatheta = endPos[2] - theta
							
							aux = theta
							if aux < 0:
								aux = np.pi*2 + aux

							#alpha = -aux + beta

							alpha = -theta + np.arctan2(deltaY, deltaX)
							K_alpha = 0.4 + 0.3 * np.exp(-alpha)

							w_vel = K_alpha*alpha + K_beta*beta

							v_omega = w_vel
							vel_robot.linear.x = 0
							vel_robot.linear.y = 0
							vel_robot.angular.z = v_omega
							pub.publish(vel_robot)
						else:
							print('Estamos en modo manual.')
							rate.sleep()

					K_alpha = 0.35
					empezarDeNuevo = False
					while rho > 0.05 and empezarDeNuevo == False:
						if auto == True:
							
							print('EndPos: ' + str(endPos[0]) + ' ' + str(endPos[1]) + ' ' + str(round(endPos[2],3)) + ' | RHO: ' + str(round(rho,3)) + ' | alfa: ' + str(round(alpha,3)) + ' | Pose: ' + str(round(pos_x,3)) + ', ' + str(round(pos_y,3)) + ', ' + str(round(theta,3)))#+ ' | v_omega: ' + str(round(v_omega,3)))

							deltaX = endPos[0] - pos_x
							deltaY = endPos[1] - pos_y
							deltatheta = endPos[2] - theta

							rho = np.sqrt(deltaX**2 + deltaY**2)
							alpha = -theta + np.arctan2(deltaY, deltaX)

							v_vel = K_rho*rho + 0.5 * np.exp(-rho)
							w_vel = K_alpha*alpha + K_beta*beta
							
							if alpha <= np.pi/2 and alpha > -np.pi/2:
								v_x = v_vel
								v_y = 0
								v_omega = w_vel
							else:# (np.abs(alpha) >= np.pi and np.abs(alpha) < 3*np.pi/2) or (alpha <= np.pi and alpha > np.pi/2):
								v_x = -v_vel
								v_y = 0
								v_omega = 0
								print('----------------Voy Hacia Atras--------------------V_X: ' + str(round(v_x, 3)))
							
							vel_robot.linear.x = v_x 
							vel_robot.linear.y = v_y
							vel_robot.angular.z = v_omega
							#vel_robot.angular.z = 0
							pub.publish(vel_robot)
							#rate.sleep()
						else:
							while auto == 1:
								#print('auto: ' + str(auto))
								print('Estamos en modo manual.')
								rate.sleep()
							empezarDeNuevo = True
			hayRuta = 0

			if auto == True:
				print("Ya llegue al destino")
				vel_robot.linear.x = v_x 
				vel_robot.linear.y = v_y
				vel_robot.angular.z = v_omega
				pub.publish(vel_robot)
			else:
				print('Estamos en modo manual.')

			rate.sleep()
		rate.sleep()
if __name__ == '__main__':
	main_control()	


