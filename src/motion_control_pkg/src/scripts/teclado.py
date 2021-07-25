#!/usr/bin/env python3
import rospy
import sys
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from pynput.keyboard import Key, Listener, Controller

#MANEJA EL ROBOT CON EL TECLADO U OPRIMIENDO K CAMBIA AL CONTROL AUTOMATICO
#DESPUES DE CORRER ESTE CORRER EL NODO DE control_sim
#ROBOCOL

comando = Twist()
flag_autonomo = Bool()
lastpressed = ''
pub = None
pub_flagAuto = None
modo = 0
panic = False


def panic_callback(msg):
	global panic
	panic = msg.data

def on_press(key):
	"""Lee la tecla oprimida por el usuario y determina la accion que se debe realizar."""
	global comando, modo, flag_autonomo, pub_flagAuto, panic
	if modo == 1: #Manejo con Teclas
		if (key == Key.right) or (format(key) == "'d'"):
			comando.angular.z = -4
		elif (key == Key.left) or (format(key) == "'a'"):
			comando.angular.z = 4
		elif (key == Key.up) or (format(key) == "'w'"):
			comando.linear.x = 4
		elif (key == Key.down) or (format(key) == "'s'"):
			comando.linear.x = -4
		else:
			comando.linear.x = 0
			comando.angular.z = 0
		publicar(comando)

	if panic == True:
		print('Boton PANICO')
		sys.stdout.write("\033[K") # Clear to the end of line
		sys.stdout.write("\033[F") # Cursor up one line
		time.sleep(1)

	elif (format(key) == "'k'"):

		#0 es autonomo, 1 es control por flechas
		if modo == 0:
			modo = 1
			flag_autonomo = False
			print('\r modo: ' + ('Teleop..' if modo == 1 else 'Autonomo'))
			sys.stdout.write("\033[K") # Clear to the end of line
			sys.stdout.write("\033[F") # Cursor up one line
			time.sleep(1)
		else:
			modo = 0
			flag_autonomo = True
			print('\r modo: ' + ('Teleop..' if modo == 1 else 'Autonomo'))
			sys.stdout.write("\033[K") # Clear to the end of line
			sys.stdout.write("\033[F") # Cursor up one line
			time.sleep(1)

		pub_flagAuto.publish(flag_autonomo)

		
		return False

def on_release(key):
	""" Indica que se solto una tecla que estaba oprimida"""
	global lastpressed, comando
	lastpressed = ''
	comando.linear.x = 0
	comando.angular.z = 0
	publicar(comando)
	return False

def publicar(comando):
	""" Publica el comando para realizar el movimiento de la tortuga"""
	global pub, modo
	pub.publish(comando)
	#print(comando)
	#print(modo)


def rover_teleop():
	global pub, modo, pub_flagAuto, panic
	print('Starting rover_teleop node and publishing flag_autonomo...')
	rospy.init_node('rover_teleop', anonymous=True)  # Inicia el nodo teleop
	pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
	pub_flagAuto = rospy.Publisher('Robocol/MotionControl/flag_autonomo', Bool, queue_size=1)

	rospy.Subscriber('Robocol/MotionControl/flag_panic', Bool, panic_callback, tcp_nodelay=True)

	rate = rospy.Rate(10)
	vel_robot = Twist()

	#listener = Listener(on_press=on_press,on_release=on_release)
	#listener.start()
	#listener.join()
	print(' ')
	print('Para cambiar entre autonomo y manual oprima k')
	print('Esperando comando...')
	print(' ')
	while not rospy.is_shutdown():
		with Listener(on_press=on_press, on_release=on_release) as listener:
			listener.join()
		
		rate.sleep()
	#rate.sleep()


if __name__== '__main__':
	try:
		rover_teleop()
	except rospy.ROSInterruptException:
		print('Nodo detenido')