#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

class Robot:

    def __init__(self) -> None:
        self.pub = rospy.Publisher('/type', String, queue_size=5)

    def publish(self):
        self.pub.publish("Hi! I am robot")

if __name__ == '__main__':

    # init node
    rospy.init_node("Robot", anonymous=False)
    rospy.loginfo("[Robot]: Starting node")
    
    # init robot object
    robot = Robot()

    # setting rate
    rate = rospy.Rate(10)

    # loop till node is running
    while not rospy.is_shutdown():
        cmd = input("Do you want to publish [y/n] : ")
        
        # check if the user has selected yes
        if (cmd == 'y'):
            robot.publish()
            rospy.loginfo("[Robot]: Publishing")

        # if not then shutdown
        else:
            rospy.loginfo("[Robot]: Shutting down")
            rospy.signal_shutdown("User selected N")
            break

        rate.sleep()