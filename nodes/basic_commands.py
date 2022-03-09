#!/usr/bin/env python2
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

class BasicCommand:
    def __init__(self):
        # start ROS node
        rospy.init_node('basic_commands')
        
        # load parameter
        # self.agent_mode_timeout_sec = rospy.get_param('~agent_mode_timeout_sec', 1.0)
        
        self.pub_takeoff = rospy.Publisher('takeoff', Empty, queue_size=1, latch=False)
        self.pub_land = rospy.Publisher('land', Empty, queue_size=1, latch=False)
        self.pub_emergency = rospy.Publisher('emergency', Empty, queue_size=1, latch=False)
        
    def takeoff(self):
        self.pub_takeoff.publish()
        
    def land(self):
        self.pub_land.publish()
    
    def emergency_stop(self):
        self.pub_emergency.publish()
    
    def spin(self):
        rospy.spin()
    
    def command(self):
        print("Command list")
        print("t : Takeoff")
        print("l : Landing")
        print("e : Emergency Stop")
        
        cmd = raw_input("Input command : ")
        
        if (cmd == 't'):
            self.takeoff()
        elif (cmd == 'l'):
            self.land()
        elif (cmd == 'e'):
            self.emergency_stop()
        else:
            print('hover')
    
if __name__ == '__main__':
    try:
        node = BasicCommand()
        node.command()
        node.spin()
    except rospy.ROSInterruptException:
        pass

