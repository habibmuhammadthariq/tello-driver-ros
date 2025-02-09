#!/usr/bin/env python

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
from geometry_msgs.msg import Twist, Point, Pose
import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

# addition
import math
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from tello_driver.msg import TelloStatus
from numpy import *

MAX_LIN_VEL = 0.6
MAX_ANG_VEL = 4


LIN_VEL_STEP_SIZE = 0.3
ANG_VEL_STEP_SIZE = 2

msg = """
Control Your Drone!
---------------------------
Moving around:
        w
   a    s    d
        x
        
        e

w/x : increase/decrease linear velocity (~ 0.4)
a/d : increase/decrease angular velocity (~ 4)

space key, s : force stop
q : landing
e : emergency stop
"""

e = """Communications Failed """

def getKey():
    key = raw_input('Silahkan masukan perintah : ')
    return key

def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

def checkLinearLimitVelocity(vel):
    vel = constrain(vel, -MAX_LIN_VEL, MAX_LIN_VEL)

    return vel

def checkAngularLimitVelocity(vel):
    vel = constrain(vel, -MAX_ANG_VEL, MAX_ANG_VEL)

    return vel

def takeoff(pub):
    msg = Empty()
    pub.publish(msg)

def land(pub):
    msg = Empty()
    pub.publish(msg)
    
def emergency(pub):
    msg = Empty()
    pub.publish(msg)

if __name__=="__main__":

    rospy.init_node('djitello_teleop')
    takeoff_pub = rospy.Publisher('/tello/takeoff', Empty, queue_size=1)
    land_pub = rospy.Publisher('/tello/land', Empty, queue_size=1)
    emergency_pub = rospy.Publisher('/tello/emergency', Empty, queue_size=1)
    cmd_vel_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=10)


    status = 0
    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    control_linear_vel  = 0.0
    control_angular_vel = 0.0

    try:
        # the drone taking off
        rospy.loginfo("The drone going to takeoff")
        takeoff(takeoff_pub)
        rospy.sleep(3)
        
        #show the keyboard menu up
        print(msg)
        while(1):
            key = getKey()
            if key == 'w' :
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel))
            elif key == 'x' :
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel))
            elif key == 'a' :
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel))
            elif key == 'd' :
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel))
            elif key == ' ' or key == 's' :
                target_linear_vel   = 0.0
                control_linear_vel  = 0.0
                target_angular_vel  = 0.0
                control_angular_vel = 0.0
                print(vels(target_linear_vel, target_angular_vel))
            else:
                if (key == 'q'):
                    rospy.loginfo("Mission completed! The drone going to land")
                    land(land_pub)
                    rospy.sleep(1)
                    break
                elif (key == 'e'):
                    rospy.loginfo("Emergency stop!! System will be dead soon")
                    emergency(emergency_pub)
                    rospy.sleep(1)
                    break
                    
            if status == 10 :
                print(msg)
                status = 0

            twist = Twist()

            control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
            twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

            control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

            cmd_vel_pub.publish(twist)
            rospy.sleep(5)

    except:
        rospy.loginfo(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        cmd_vel_pub.publish(twist)
        # addition from me
        rospy.loginfo("If the drone has taken off it will land")
        rospy.sleep(1)
        land(land_pub)
        rospy.sleep(1)

        
        
        
        
        
