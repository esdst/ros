#!/usr/bin/env python3

import rospy
import message_filters
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped

class ObjectDetection:
    
    def __init__(self) -> None:
        
        # subscribing to type of robot
        self.robot_type_sub = message_filters.Subscriber('/type', String)

        # subscribing to pose of robot
        self.robot_pose_sub = message_filters.Subscriber('/robot/pose',  PoseStamped)

        # subscribing to image from camera node
        self.camera_img_sub = message_filters.Subscriber('/camera/image', Image)

        # time synchronizer
        #time_synchronizer = message_filters.ApproximateTimeSynchronizer([self.robot_type_sub, self.camera_img_sub], 10, 0.1, allow_headerless=True)

        time_synchronizer = message_filters.ApproximateTimeSynchronizer([self.robot_pose_sub, self.camera_img_sub], 10, 0.1, allow_headerless=True)

        # register callback
        time_synchronizer.registerCallback(self.callback)

    #def callback(self, type_msg: String, img_msg: Image) -> None:
    def callback(self, pose_msg: PoseStamped, img_msg: Image) -> None:
        rospy.loginfo("[Object Detection]: received synced msgs")


if __name__ == '__main__':

    # init node
    rospy.init_node("object_detection", anonymous=True)
    rospy.loginfo("[Object Detection]: Init Node")

    # create object detection instance
    object_detection = ObjectDetection()

    rospy.spin()


