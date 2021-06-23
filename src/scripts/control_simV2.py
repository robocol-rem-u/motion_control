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

#RECIBE LAS INDICACIONES PARA HACER EL RECORRIDO
#ROBOCOL

pos_x, pos_y, theta = 0, 0, 0

deltaX, deltaY = 0, 0

rho = 0
auto = 0
hayRuta = 1
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

	#rate.sleep()

def planeacion_callback(ruta):
	pass
	

def habilitarMov(msg): #Me indica si debo mover el robot autonomamente o no
	global auto
	auto = msg.data
	print('auto:' + str(auto))


def ruta_callback(msg):
	global ruta
	#a = np.zeros((3,3))
	print('msg.data y len:')
	print(msg.data)
	
	aux = msg.data
	ruta = aux.copy()

	ruta.resize(len(msg.data)/2, 2)
	print('ruta: ')
	print(ruta)
	print(len(ruta))


def hayRuta_callback(msg): #Me indica si ya hay ruta o no
	global hayRuta
	hayRuta = msg.data
	print('hayRuta: ' + str(auto))


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
	rospy.Subscriber('Robocol/MotionControl/flag_hayRuta',Bool,hayRuta_callback, tcp_nodelay=True)


	#ruta = np.array([[0,0], [1,0], [1,1], [0,1], [0,0]])
	ruta = np.array([[2.5,0.019],[1.5,1],[-0.035169,0.018923]])

	#ruta = np.array([[5.302769,0], [4.203994,6.226416], [14.329459,-0.4288], [29.2159,5.30052], [12.103476,-11.714824]])


	vel_robot = Twist()

	rho = np.sqrt(endPos[0]**2 + endPos[1]**2)
	alpha = -theta + np.arctan2(endPos[1], endPos[0])
	beta = -theta - alpha

	K_rho = 0.15
	K_alpha = 0.35
	K_beta = -0.0

	hayRuta = 1

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
		print('hay ruta en while: ')
		print(hayRuta)

		print('ruta: ')
		print(ruta)
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
				beta = -deltatheta -alpha

				K_alpha = 0.4
				beta = endPos[2]
				beta = 0
				v_vel = 0
				v_x = 0
				v_y = 0
				empezarDeNuevo = True
				while empezarDeNuevo == True:
					while abs(alpha) > 0.05:
						if auto == True:
							print('Punto final: ' + str(endPos[0]) + ' ' + str(endPos[1]) + ' ' + str(endPos[2]))
							print('                                 alpha: ' + str(alpha))
							print('                                                       pos_x: ' + str(pos_x))
							print('                                                       pos_y: ' + str(pos_y))
							print('                                                       theta: ' + str(theta))
							deltaX = endPos[0] - pos_x
							deltaY = endPos[1] - pos_y
							deltatheta = endPos[2] - theta
							
							aux = theta
							if aux < 0:
								aux = np.pi*2 + aux

							alpha = -aux + beta
							#alpha = -aux + beta

							rho = np.sqrt(deltaX**2 + deltaY**2)
							alpha = -theta + np.arctan2(deltaY, deltaX)
							K_alpha = 0.4 + 0.3 * np.exp(-alpha)
							#beta = -deltatheta -alpha

							v_vel = K_rho*rho
							w_vel = K_alpha*alpha + K_beta*beta

							v_x = 0
							v_y = 0
							v_omega = w_vel
							vel_robot.linear.x = v_x 
							vel_robot.linear.y = v_y
							vel_robot.angular.z = v_omega
							pub.publish(vel_robot)
						else:
							print('auto: ' + str(auto))
							print('Estamos en modo manual.')
							rate.sleep()

					#beta = -theta - alpha	
					K_alpha = 0.35
					empezarDeNuevo = False
					while rho > 0.05 and empezarDeNuevo == False:
						if auto == True:
							print('Punto final: ' + str(endPos[0]) + ' ' + str(endPos[1]) + ' ' + str(endPos[2]))
							print('                                 rho: ' + str(rho))
							print('                                 alpha: ' + str(alpha))


							print('                                                       pos_x: ' + str(pos_x))
							print('                                                       pos_y: ' + str(pos_y))
							print('                                                       theta: ' + str(theta))

							deltaX = endPos[0] - pos_x
							deltaY = endPos[1] - pos_y
							deltatheta = endPos[2] - theta

							rho = np.sqrt(deltaX**2 + deltaY**2)
							alpha = -theta + np.arctan2(deltaY, deltaX)
							#beta = -deltatheta -alpha

							v_vel = K_rho*rho + 0.5 * np.exp(-rho)
							w_vel = K_alpha*alpha + K_beta*beta
							
							if alpha <= np.pi/2 and alpha > -np.pi/2:
								print('                                                       V_X: ' + str(v_x))
								v_x = v_vel
								v_y = 0
								v_omega = w_vel
							elif (np.abs(alpha) >= np.pi and np.abs(alpha) < 3*np.pi/2) or (alpha <= np.pi and alpha > np.pi/2):
								print('----------------------------------Voy Hacia Atras---------')
								print('-------------------------------------------------------V_X: ' + str(v_x))
								v_x = -v_vel
								v_y = 0
								v_omega = w_vel
							
							vel_robot.linear.x = v_x 
							vel_robot.linear.y = v_y
							#vel_robot.angular.z = v_omega
							vel_robot.angular.z = 0
							pub.publish(vel_robot)
							#rate.sleep()
						else:
							while auto == 1:
								print('auto: ' + str(auto))
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


