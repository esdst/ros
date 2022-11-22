#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image

class Camera:

    def __init__(self) -> None:
        self.sub = rospy.Subscriber('/type', String, callback=self.callback)
        rospy.loginfo("[Camera]: Subscriber started")

        self.img_pub = rospy.Publisher('/camera/image', Image, queue_size=5)

    def callback(self, msg):
        rospy.loginfo("[Camera]: I hear you : %s", msg.data)

    def publish_img(self) -> None:
        img = Image()
        img.header.stamp = rospy.Time.now()
        self.img_pub.publish(img)
        #rospy.loginfo("[Camera]: Publishing Image")

if __name__ == '__main__':

    #init node
    rospy.init_node("Camera", anonymous=False)
    rospy.loginfo("[Camera]: Started Node")

    # Create Camera Object
    camera = Camera()

    # running node indefinetly
    # rospy.spin()

    # setting rate 10Hz
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        camera.publish_img()

        rate.sleep()

