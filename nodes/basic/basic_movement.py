#!/usr/bin/env python2
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
import traceback

def main():
    rospy.loginfo('Taking off')
    takeoff()

    rospy.loginfo('move forward')
    cmd_vel.linear.y = 1    # cmd_vel.y = 1 and sleep = 2 mean that the drone will move 25 cm/s forward
    cmd_vel.angular.z = 0.0
    pub_vel.publish(cmd_vel)
    rospy.sleep(2)

    rospy.loginfo('hover')
    cmd_vel.linear.y = 0.0
    cmd_vel.angular.z = 0.0
    pub_vel.publish(cmd_vel)
    rospy.sleep(1)

    rospy.loginfo('turn right')
    cmd_vel.linear.y = 0.0
    cmd_vel.angular.z = 1
    pub_vel.publish(cmd_vel)
    rospy.sleep(12)

    rospy.loginfo('hover')
    cmd_vel.linear.y = 0.0
    cmd_vel.angular.z = 0.0
    pub_vel.publish(cmd_vel)
    rospy.sleep(1)


def takeoff():
    pub_takeoff.publish(takeoff_msg) 
    rospy.sleep(2)
    
    # cmd_vel.linear.z = 0.75
    # pub_vel.publish(cmd_vel)
    #
    # rospy.sleep(3)
    #
    # cmd_vel.linear.z = 0
    # pub_vel.publish(cmd_vel)



if __name__ == '__main__':
    
    rospy.init_node('basic_movement')

    pub_takeoff = rospy.Publisher('/tello/takeoff',Empty,queue_size=1)

    pub_land = rospy.Publisher('/tello/land',Empty,queue_size=1)

    pub_vel = rospy.Publisher('/tello/cmd_vel',Twist,queue_size=1)

    takeoff_msg = Empty()
    cmd_vel = Twist()     

    try:

        cmd_vel.linear.z = 0.7
        pub_vel.publish(cmd_vel)

        rospy.sleep(3)

        cmd_vel.linear.z = 0
        pub_vel.publish(cmd_vel)

        main()

    except BaseException:
        traceback.print_exc()
    finally:
        rospy.loginfo('Landing')
        pub_land.publish(Empty())
