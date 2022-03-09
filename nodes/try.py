#!/usr/bin/env python2.7
import rospy
import math
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point, Pose
from tello_driver.msg import TelloStatus
# import threading
# import sys
# import tf.transformations as tftr
from numpy import *


class Tello:

    def __init__(self):
        # Creates a node with name 'tello_controller' and make sure it is a
        # unique node
        rospy.init_node('keyboard_controller', anonymous=True)

        self.takeoff_publisher = rospy.Publisher('/tello/takeoff', Empty, queue_size=1)
        self.land_publisher = rospy.Publisher('/tello/land', Empty, queue_size=1)
        self.emergency_publisher = rospy.Publisher('/tello/emerygency', Empty, queue_size=1)
        # self.flip_publisher = rospy.Publisher('/tello/flip', 

        self.velocity_publisher = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=5)

        # self.status_subscriber = rospy.Subscriber('/tello/status', TelloStatus, self.get_status)
        
        self.status = None

        # self.odom_subscriber = rospy.Subscriber('/tello/odom', Odometry, self.update_odom)

        # self.x = None
        # self.y = None
        # self.z = None
        # self.q = None
        # self.rpy = None
        # self.theta = None
        # self.theta_start = None

        # self.time_start = -1.0
        # self.start = Pose()

        self.rate = rospy.Rate(60)

        # return result_point
        
    def take_off(self):
        msg = Empty()
        self.takeoff_publisher.publish(msg)

    def land(self):
        msg = Empty()
        self.land_publisher.publish(msg)
        # self.file.close()
    
    def emergency(self):
        msg = Empty()
        self.emergency_publisher.publish(msg)
        
    def keyboard(self):
        # variable declaration
        x = 0
        y = 0
        z = 0
        yaw = 0
        stay = True
        key = ''
        
        # show the menu up
        print('Daftar pilihan perintah')
        print('\tW\t\t\t\tI')
        print('A\t\tD\t\tJ\t\tL')
        print('\tS\t\t\t\tK')
        print('Q\tT')
        
        print('Keterangan')
        print('W : Naik')
        print('S : Turun')
        print('A : Belok Kiri')
        print('D : Belok Kanan')
        print('I : Maju')
        print('K : Mundur')
        print('J : Kiri')
        print('L : Kanan')
        print('Q : Landing')
        print('E : Emergency Stop')
        command = raw_input('Silahkan masukan perintah : ')
        
        print('The command was : {}'.format(command))
        
        if command == 'w':
            z = 0.1
        elif command == 's':
            z = -0.1
        elif command == 'a':
            yaw = -15
        elif command == 'd':
            yaw = 15
        elif command == 'i':
            x = 0.1
        elif command == 'k':
            x = -0.1
        elif command == 'j':
            y = -0.1
        elif command == 'l':
            y = 0.1
        elif command == 'q':
            stay = False
        elif command == 'e':
            key = 'stop'
        
        return [x, y, z, yaw, stay, key]

if __name__ == '__main__':
    try:
        drone = Tello()
        rospy.Rate(1).sleep() # Setiing up a subscriber may take a while ...

        rospy.loginfo("Taking Off")
        drone.take_off()
        rospy.sleep(3)
        # print('next')
        # drone.set_start()
        # rospy.sleep(9)
        
        # stay = True
        cmd_vel = Twist()
        while True:
            values = drone.keyboard()
            
            # 
            cmd_vel.linear.x = values[0]
            cmd_vel.linear.y = values[1]
            cmd_vel.linear.z = values[2]
            #
            cmd_vel.angular.z = values[3]
            # stay or leave then landing
            stay = values[4]
            # special key
            key = values[5]
            
            # emergency stop
            if key == 'stop':
                rospy.loginfo("Emergency Stop")
                drone.emergency()
                rospy.sleep(1)
                break
            elif stay == False:
                break
            
            # send command to the drone
            rospy.loginfo("Go To Next Destination")
            drone.velocity_publisher.publish(cmd_vel)
            rospy.sleep(5)
            
        rospy.loginfo("Mission completed! The drone going to land")
        if not stay:
            drone.land()
            rospy.sleep(1)

    #rospy.spin()
    except rospy.ROSInterruptException:
        pass
        
        
        
        
