#!/usr/bin/env python
import rospy
import numpy as np
import sys
import cv2
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion

global pos_x, pos_y, theta
global pos_x_total, pos_y_total
global final

#GRAFICA EN TIEMPO REAL LA POSICION DEL ROBOT EN EL MAPA
#CAMBIAR LA DIRECCION DE LA IMAGEN (gridmap), SOLO SE PUEDE CORRER DESPUES DE CORRER PLANEACION
#ROBOCOL

pos_x, pos_y, theta = 0, 0, 0 #Donde se almacena la posicion que nos da Gazebo

#Variables para graficar
pos_x_total = [0]
pos_y_total = [0]

final = False #Variables para terminar plot


scriptDir = os.path.dirname(__file__)
ruta_img = scriptDir + "/mapafinal.png" #Utiliza el mapa que sale al correr "planeacion_a.py"
gridmap = cv2.imread(ruta_img,0) 

height, width = gridmap.shape

gridmap = cv2.cvtColor(gridmap, cv2.COLOR_GRAY2RGB)

def position_callback(msg): #Me regresa la posicion del robot
	global pos_x, pos_y, theta, pos_x_total, pos_y_total
	pos_x = msg.pose.pose.position.x
	pos_y = msg.pose.pose.position.y

	orC_x = msg.pose.pose.orientation.x
	orC_y = msg.pose.pose.orientation.y
	orC_z = msg.pose.pose.orientation.z
	orC_w = msg.pose.pose.orientation.w


	#orientation_q = msg.pose.pose.orientation
	orientation_list = [orC_x, orC_y, orC_z, orC_w]
	(roll, pitch,theta) = euler_from_quaternion(orientation_list)

	pos_x_total.append(pos_x)
	pos_y_total.append(pos_y)
	

#Funcion principal de movimiento
def main():
	global pos_x, pos_y, theta, final

	rospy.init_node('control', anonymous=True) #Inicio nodo

	#pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	rate = rospy.Rate(10) #10hz
	rospy.Subscriber("zed2/odom", Odometry, position_callback, tcp_nodelay=True)

	while not rospy.is_shutdown():
		pos_x_new = int(pos_x*10 + width/2)
		pos_y_new = int(pos_y*10 + height/2)
		## Make pixels row and column red
		print('POS X:', pos_x,'POS Y:', pos_y, 'POS X NUEVO:', pos_x_new,'POS Y NUEVO:', pos_y_new)

		#gridmap[pos_y_new,pos_x_new] = (255,0,0)
		image = cv2.circle(gridmap, (pos_x_new,pos_y_new), radius=1, color=(255, 0, 0), thickness=-1)
		cv2.namedWindow('image')
		cv2.resizeWindow('image',height/2, width/2)
		cv2.imshow('image',image)
		cv2.waitKey(1)
		#rate.sleep()
	final = True

if __name__ == '__main__':
	main()	


