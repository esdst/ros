#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

class Camera:

    def __init__(self) -> None:
        self.sub = rospy.Subscriber('/type', String, callback=self.callback)
        rospy.loginfo("[Camera]: Subscriber started")

    def callback(self, msg):
        rospy.loginfo("I hear you : %s", msg.data)


if __name__ == '__main__':

    #init node
    rospy.init_node("Camera", anonymous=False)
    rospy.loginfo("[Camera]: Started Node")

    # Create Camera Object
    camera = Camera()

    # running node indefinetly
    rospy.spin()

