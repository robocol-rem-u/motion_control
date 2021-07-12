#!/usr/bin/env python
import rospy
import cv2
import os 
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates

global pos_x, pos_y, pos_x_model, pos_y_model
global pos_x_total_zed2, pos_y_total_zed2, pos_x_total_model, pos_y_total_model
global final

#GRAFICA EN TIEMPO REAL LA POSICION DEL ROBOT EN EL MAPA
#CAMBIAR LA DIRECCION DE LA IMAGEN (gridmap), SOLO CORRER DESPUES DE CORRER PLANEACION
#ROBOCOL

pos_x, pos_y, pos_x_model, pos_y_model = 0, 0, 0, 0,

#Variables para acumular todas las posiciones
pos_x_total_zed2 = [0]
pos_y_total_zed2 = [0]
pos_x_total_model = [0]
pos_y_total_model = [0]

final = False #Variables para terminar plot


scriptDir = os.path.dirname(__file__)
ruta_img = scriptDir + "/mapafinal.png" #Utiliza el mapa que sale al correr "planeacion_a.py"
gridmap = cv2.imread(ruta_img,0) 

height, width = gridmap.shape

gridmap = cv2.cvtColor(gridmap, cv2.COLOR_GRAY2RGB)

def position_callback(msg): #Me regresa la posicion del robot segun Zed2
	global pos_x, pos_y, pos_x_total_zed2, pos_y_total_zed2
	pos_x = msg.pose.pose.position.x
	pos_y = msg.pose.pose.position.y

	pos_x_total_zed2.append(pos_x)
	pos_y_total_zed2.append(pos_y)


def model_callback(msg): #Me regresa la posicion del robot segun Model States
	global pos_x_model, pos_y_model, pos_x_total_model, pos_y_total_model
	msg_pose = msg.pose
	msg_pose_pos = msg_pose[1].position
	pos_x_model = msg_pose_pos.x
	pos_y_model = msg_pose_pos.y

	pos_x_total_model.append(pos_x_model)
	pos_y_total_model.append(pos_y_model)	

#Funcion principal grafica
def main():
	global pos_x, pos_y, final, pos_x_model, pos_y_model

	rospy.init_node('control', anonymous=True) #Inicio nodo

	#pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	rate = rospy.Rate(10) #10hz
	rospy.Subscriber("zed2/odom", Odometry, position_callback, tcp_nodelay=True)
	rospy.Subscriber("gazebo/model_states", ModelStates, model_callback, tcp_nodelay=True)

	print('Graficando...')

	while not rospy.is_shutdown():
		pos_x_new = int(pos_x*10 + width/2)
		pos_y_new = int(pos_y*10 + height/2)

		pos_x_new_model = int(pos_x_model*10 + width/2)
		pos_y_new_model = int(pos_y_model*10 + height/2)
		
		#print('POS X: ', round(pos_x,3),'POS Y: ', round(pos_y,3), 'Model_X: ', round(pos_x_model,3),'Model_Y: ', round(pos_y_model,3))

		image = cv2.circle(gridmap, (pos_x_new,pos_y_new), radius=1, color=(255, 0, 0), thickness=-1) #color=(Blue,Green,Red)
		image_model = cv2.circle(gridmap, (pos_x_new_model,pos_y_new_model), radius=1, color=(0, 0, 255), thickness=-1)
		img_final = cv2.resize(image + image_model, (500, 500), interpolation=cv2.INTER_AREA)
		cv2.namedWindow('Plot')
		cv2.resizeWindow('Plot',height/2, width/2)
		cv2.imshow('Plot',img_final)
		cv2.waitKey(1)

	final = True

if __name__ == '__main__':
	main()	


