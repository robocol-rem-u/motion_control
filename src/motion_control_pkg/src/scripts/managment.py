#!/usr/bin/env python
import rospy
import time
import threading
from std_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *

#TIENE BOTON DE PANICO, DEPLOY PROBE Y RESET ODOMETRY
#ROBOCOL

class Poses_Publish(object):
    def __init__(self):
        super(Poses_Publish, self).__init__()
        
        self.opciones = True
        
        #self.glo_x,self.glo_y,self.glo_z = 0.0,0.0,0.0
        #self.ini_pose = [self.glo_x,self.glo_y,self.glo_z]
        #self.end_pose = [self.glo_x,self.glo_y,self.glo_z]

        #self.landmarks= [[7.31, 0], [7.19,7.55], [18.85,-3.59], [33.77,6.41], [13.22,-13.61],[21.01,13.21],[20.96,3.36], [20.40, -19.41], [14.77,6.89],[22.46,-10.36], [31.56, -18.81], [29.92,11.44], [32.79,-6.79], [2.04,-12.02], [7.63,13.24]]
        #self.waypoints=[[12.19,8.73],[25.04,4.36],[28.62,-6.17],[11.63,-16.85],[7.64,-5.55],[27.48,-13.65]]
        print('Starting robocol_managment node...')
        rospy.init_node('robocol_managment')
        # Publishers
        print('Publishing in cmd_vel')
        self.pubVel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        print('Publishing in zed2/reset_odometry')
        self.pubResetOdom = rospy.Publisher('zed2/reset_odometry', Empty, queue_size=1)
        print('Publishing in probe_deployment_unit/drop')
        self.pubProbe = rospy.Publisher('probe_deployment_unit/drop', Empty, queue_size=1)
        print('Publishing in Robocol/MotionControl/flag_autonomo')
        self.pub_flagAuto = rospy.Publisher('Robocol/MotionControl/flag_autonomo', Bool, queue_size=1)

        # # Subscribers
        print('Subscribing in probes_dropped\n')
        rospy.Subscriber('probe_deployment_unit/probes_dropped', UInt8, self.probe_callback)

        rospy.on_shutdown(self.kill)
        print('')
        x = threading.Thread(target=self.thread_function)
        x.start()

    def probe_callback(self,param):
        self.glo_x = 0

    def thread_function(self):
        while self.opciones:
            print("Choose an option:")
            print(" P: Panic Button")
            print(" D: To deploy probe.")
            print(" R: To reset odometry.")
            op = raw_input(' > ')
            print(op)
            if op == "P":
                print(' PANIK BUTTON')
                print('  Stopping Control Node...')
                flag_autonomo = False
                self.pub_flagAuto.publish(flag_autonomo)
                print('  Stopped Robot.')
                vel_robot = Twist()
                vel_robot.linear.x = 0
                vel_robot.linear.y = 0
                vel_robot.angular.z = 0
                self.pubVel.publish(vel_robot)
                
            elif op == "D":
                print(" Deplying probe")
                self.pubProbe.publish()
                

            elif op == "R":
                print(' Reseting Odometry...')
                self.pubResetOdom.publish()
            else:
                print(' COMMAND NOT RECOGNIZED.')
            print('')
        print('Closing thread...')

    def kill(self):
        print("\nKilling node...")
        self.opciones = False
        print('Press Enter to end...')
        

def main():
    try:
        poses_Publish = Poses_Publish()

        time.sleep(1)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # poses_Publish.pub_test()
            rate.sleep()
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main()