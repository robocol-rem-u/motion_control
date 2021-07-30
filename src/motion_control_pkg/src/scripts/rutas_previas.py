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
		self.routes= [[7.2, -0.025, 7.5, 1.0250000000000001, 8.05, 7.175000000000001, 8.55, 8.075000000000001, 8.6, 8.875, 11.5, 9.275, 12.15, 9.075000000000001, 12.200000000000001, 8.725],[18.0, 8.725, 18.55, 8.275, 19.5, 7.825, 19.700000000000003, 7.775, 20.05, 7.7250000000000005, 21.1, 7.675000000000001, 21.400000000000002, 7.625, 24.1, 7.625, 24.200000000000003, 5.525, 25.05, 4.375],[28.1, 4.375, 28.6, 1.625, 28.6, -6.175000000000001],[28.450000000000003, -6.175000000000001, 28.400000000000002, -6.825, 19.85, -7.525, 19.700000000000003, -7.625, 18.35, -8.425, 17.650000000000002, -9.425, 17.5, -9.925, 17.400000000000002, -16.375, 13.850000000000001, -16.425, 13.350000000000001, -16.525000000000002, 12.3, -16.825, 11.65, -16.825],[11.65, -12.025, 11.600000000000001, -12.025, 10.100000000000001, -11.825000000000001, 9.950000000000001, -10.725000000000001, 9.35, -10.625, 9.3, -9.675, 9.200000000000001, -9.075000000000001, 8.450000000000001, -8.425, 8.35, -6.9750000000000005, 7.8500000000000005, -6.9750000000000005, 7.7, -6.2250000000000005, 7.65, -5.525],[12.350000381469727, -16.825000762939453, 12.75, -16.674999237060547, 13.949999809265137, -16.375, 23.600000381469727, -15.975000381469727, 24.100000381469727, -15.975000381469727, 24.600000381469727, -15.824999809265137, 25.75, -15.824999809265137, 25.799999237060547, -15.725000381469727, 25.899999618530273, -15.725000381469727, 26.200000762939453, -15.074999809265137, 27.0, -14.125, 27.5, -14.125, 27.5, -13.625]]
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