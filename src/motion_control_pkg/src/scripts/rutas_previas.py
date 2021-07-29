#!/usr/bin/env python3
import rospy
import time
import threading
import numpy 
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats



class Poses_Publish(object):
	def __init__(self):
		super(Poses_Publish, self).__init__()
		self.opciones = True
		#rutas planeadas previamente
		self.routes= [[7.31, 0,8.6,1.5,0.0], [7.19,7.55], [18.85,-3.59], [33.77,6.41], [13.22,-13.61],[21.01,13.21],[20.96,3.36], [20.40, -19.41], [14.77,6.89],[22.46,-10.36], [31.56, -18.81], [29.92,11.44], [32.79,-6.79], [2.04,-12.02], [7.63,13.24]]
		self.waypoints=[[12.19,8.73],[25.04,4.36],[28.62,-6.17],[11.63,-16.85],[7.64,-5.55],[27.48,-13.65]]
		print('Starting robocol_routes node...')
		rospy.init_node('robocol_routes')
		# Publishers
		print('Publishing in /Robocol/MotionControl/ruta (Floats)')
		self.pubroute = rospy.Publisher('/Robocol/MotionControl/ruta',numpy_msg(Floats),queue_size=10)
		rospy.on_shutdown(self.kill)
		print('')
		x = threading.Thread(target=self.thread_function)
		x.start()

	def thread_function(self):
		while self.opciones:
			print("Choose the Waypoint you want to go:")
			print(f" First  Waypoint: {self.waypoints[0]} ")
			print(f" Second Waypoint: {self.waypoints[1]} ")
			print(f" Third  Waypoint: {self.waypoints[2]} ")
			print(f" Fourth Waypoint: {self.waypoints[3]} ")
			print(f" Fifth  Waypoint: {self.waypoints[4]} ")
			print(f" Sixth  Waypoint: {self.waypoints[5]} ")
			op = (input(' > '))
			print(op)
			
			if (op) == "1" or (op) =="2" or (op) =="3" or (op) == "4" or (op) == "5" or (op) == "6":
				op = int(op)
				rout = self.routes[op-1]
				print(f"Route to follow: {rout}")
				self.pub_coords(rout)
			else:
				print(' COMMAND NOT RECOGNIZED.')
			print('')
		print('Closing thread...')

	def kill(self):
		print("\nKilling node...")
		self.opciones = False
		print('Press Enter to end...')
		
	def pub_coords(self,route_array):
		
		r = numpy.array (route_array,dtype = numpy.float32)
		self.pubroute.publish(r)
		print('*Route published.')
		
		

def main():
	try:
		poses_Publish = Poses_Publish()

		time.sleep(1)
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			
			rate.sleep()
	except rospy.ROSInterruptException:
		return
	except KeyboardInterrupt:
		return

if __name__ == '__main__':
	main()