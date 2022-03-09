#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point, Pose
from tello_driver.msg import TelloStatus
import threading
import sys
import tf.transformations as tftr
from numpy import *

lock = threading.Lock()

H = 0.5  # m
#masx = [0.6, -0.6, -0.6, 0.6]
#masy = [0.6, 0.6, -0.6, -0.6]
masx = [0.3, -0,3]
masy = [0, 0]
E = 0.15
Ea = pi/12
V_MAX   = 0.5 # m/s 0.5
W_MAX   = 0.9 # rad/s 0.9

class Tello:

    def __init__(self):
        # Creates a node with name 'tello_controller' and make sure it is a
        # unique node
        rospy.init_node('tello_controller', anonymous=True)

        self.takeoff_publisher = rospy.Publisher('/tello/takeoff', Empty, queue_size=1)
        self.land_publisher = rospy.Publisher('/tello/land', Empty, queue_size=1)

        self.velocity_publisher = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=5)

        self.status_subscriber = rospy.Subscriber('/tello/status', TelloStatus, self.get_status)
        
        self.status = None

        self.odom_subscriber = rospy.Subscriber('/tello/odom', Odometry, self.update_odom)

        self.x = None
        self.y = None
        self.z = None
        self.q = None
        self.rpy = None
        self.theta = None
        self.theta_start = None

        self.time_start = -1.0
        self.start = Pose()

        self.file = open('laba6.txt', 'w')

        self.rate = rospy.Rate(60)              # this might be need to be decrease significantly

    def get_status(self, data):
        # Dron status callback function
        lock.acquire()

        self.status = data

        lock.release()

    def set_start(self):
        self.time_start = rospy.get_time()

        self.start.position.x = self.x
        self.start.position.y = self.y
        self.start.position.z = self.z

        self.start.orientation = self.q
        self.theta_start = tftr.euler_from_quaternion((self.q.x, self.q.y, self.q.z, self.q.w))[2]

    def update_odom(self, data):
        # Odometry callback function
        lock.acquire()

        self.x = round(data.pose.pose.position.x, 4)
        self.y = round(data.pose.pose.position.y, 4)
        self.z = round(data.pose.pose.position.z, 4)

        self.q = data.pose.pose.orientation
        self.rpy = tftr.euler_from_quaternion((self.q.x, self.q.y, self.q.z, self.q.w))  # roll pitch yaw
        self.theta = self.rpy[2]

        lock.release()

        # Log file
        if self.time_start != None:
            try:
                self.file.write( 'Time from start : {0} e_x : {1} e_y : {2} e_z : {3} e_a : {4} \n '.format(round((rospy.get_time()-self.time_start), 4), X - self.x, Y - (-self.y), H - (-self.z), ANGLE - (self.theta_start - self.theta_start) ) )
            except:
                pass

    def transform_point(self, point_to_transform):
        # x_world -> x_robot
        # y_world -> -y_robot
        # z_world -> -z_robot

        result_point = Point()

        result_point.x =   point_to_transform.x
        result_point.y = - point_to_transform.y
        result_point.z = - point_to_transform.z

        return result_point

    def saturation(self, vel_raw):
        # v_x
        if vel_raw.linear.x > V_MAX:
            vel_raw.linear.x = V_MAX
        elif vel_raw.linear.x < -V_MAX:
            vel_raw.linear.x = -V_MAX
        # v_y
        if vel_raw.linear.y > V_MAX:
            vel_raw.linear.y = V_MAX
        elif vel_raw.linear.y < -V_MAX:
            vel_raw.linear.y = -V_MAX
        # v_z
        if vel_raw.linear.z > V_MAX:
            vel_raw.linear.z = V_MAX
        elif vel_raw.linear.z < -V_MAX:
            vel_raw.linear.z = -V_MAX
        # w_x
        if vel_raw.angular.x > W_MAX:
            vel_raw.angular.x = W_MAX
        elif vel_raw.angular.x < -W_MAX:
            vel_raw.angular.x = -W_MAX
        # w_y
        if vel_raw.angular.y > W_MAX:
            vel_raw.angular.y = W_MAX
        elif vel_raw.angular.y < -W_MAX:
            vel_raw.angular.y = -W_MAX
        # w_z
        if vel_raw.angular.z > W_MAX:
            vel_raw.angular.z = W_MAX
        elif vel_raw.angular.z < -W_MAX:
            vel_raw.angular.z = -W_MAX

        return vel_raw

    def take_off(self):
        msg = Empty()
        self.takeoff_publisher.publish(msg)

    def land(self):
        msg = Empty()
        self.land_publisher.publish(msg)
        self.file.close()

    def go_to_point(self, goal_point):
        v_max = 0.5 #1.5
        kpa = 0.2 #0.9

        vel_msg = Twist()

        while True:

            x = self.x-self.start.position.x
            y = self.y-self.start.position.y
            z = self.z-self.start.position.z
            theta = self.theta - self.theta_start

            p = sqrt((goal_point.x - x)**2 + (goal_point.y - y)**2)
            a = math.atan2(goal_point.y - y, goal_point.x - x) - theta

            v_x = v_max * math.tanh(p) * math.cos(a)
            v_y = 0
            v_z = 0
            w_x = 0
            w_y = 0
            w_z = kpa * a + v_max * math.tanh(p) / p * math.sin(a) * math.cos(a)

            #velocity.linear.x = v_x * cos(a) - v_y * sin(a)
            #velocity.linear.y = v_y * sin(a) + v_y * cos(a)
            vel_msg.linear.x = v_x 
            vel_msg.linear.y = v_y
            vel_msg.linear.z = v_z

            vel_msg.angular.x = w_x
            vel_msg.angular.y = w_y
            vel_msg.angular.z = w_z

            print ('x1: {0}, x2 : {1}, y1 : {2} y2 : {3} t1 : {4} t2 : {5}'.format(self.x, self.start.position.x, self.y, self.start.position.y, self.theta*180/3.14, self.theta_start*180/3.14))
            print ('xg: {0}, yg : {1}, p : {2}'.format(goal_point.x, goal_point.y, p))
            print ('a: {0}, vx : {1}, vy : {2} x : {3} y : {4}'.format(a*180/3.14, v_x, v_y, x, y))
            self.rate.sleep()
            self.velocity_publisher.publish( self.saturation(vel_msg))
            if p < E:
                break




if __name__ == '__main__':
    f = open('lab6.txt', 'w')
    try:
        drone = Tello()
        rospy.Rate(1).sleep() # Setiing up a subscriber may take a while ...

        print('Taking off')
        drone.take_off()
        rospy.sleep(3)
        print('next')
        drone.set_start()
        rospy.sleep(9)
        print('land')
        for i in range(len(masx)):
            print(str(i))
            a = Point(masx[i], masy[i], 0)
            a = drone.transform_point(a)
            drone.go_to_point(a)
            rospy.sleep(1)
        drone.land()
        rospy.sleep(1)

    #rospy.spin()
    except rospy.ROSInterruptException:
        pass
