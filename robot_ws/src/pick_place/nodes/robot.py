#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from pick_place.msg import State
from enum import Enum
from pick_place.srv import GetCurrentRobotState, GetCurrentRobotStateResponse, GetCurrentRobotStateRequest

class RobotState(Enum):
    IDLE = 0
    ACTIVE = 1
    PAUSED = 2
    INTERRUPTED = 3


class Robot:

    def __init__(self) -> None:
        self.pub = rospy.Publisher('/type', String, queue_size=5)

        # creating pose publisher
        self.pose_pub = rospy.Publisher('/robot/pose', PoseStamped, queue_size=5)

        # creating a state publisher
        self.state_pub = rospy.Publisher('/robot/state', State, queue_size=5)

        # create a service GetCurrentRobotState
        self.service = rospy.Service("/get_current_robot_state", GetCurrentRobotState, self.service_callback)

    def service_callback(self, req: GetCurrentRobotStateRequest):
        res = GetCurrentRobotStateResponse()
        state_msg = State()
        state_msg.value = RobotState.PAUSED.value
        res.state = state_msg
        return res


    def publish(self):
        self.pub.publish("Hi! I am robot")

    def publish_pose(self) -> None:
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        self.pose_pub.publish(pose_stamped)
    
    def publish_state(self, robot_state: RobotState) -> None:
        state_msg = State()
        state_msg.value = robot_state.value
        self.state_pub.publish(state_msg)


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
        cmd = input("Do you want to publish [y/n/a/i] : ")
        
        # check if the user has selected yes
        if (cmd == 'y'):
            robot.publish()
            rospy.loginfo("[Robot]: Publishing")

            robot.publish_pose()
            rospy.loginfo("[Robot]: publishing pose")

        if (cmd == 'a'):
            robot.publish_state(robot_state=RobotState.ACTIVE)
        
        if (cmd == 'i'):
            robot.publish_state(robot_state=RobotState.IDLE)

        # if not then shutdown
        else:
            rospy.loginfo("[Robot]: Shutting down")
            rospy.signal_shutdown("User selected N")
            break

        rate.sleep()