#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose
from pick_place.msg import State
from enum import Enum
from pick_place.srv import GetCurrentRobotState, GetCurrentRobotStateResponse, GetCurrentRobotStateRequest
import moveit_commander
import sys
from moveit_commander import (
    RobotCommander,
    PlanningSceneInterface,
    MoveGroupCommander,
    MoveItCommanderException
)
import tf


class RobotState(Enum):
    IDLE = 0
    ACTIVE = 1
    PAUSED = 2
    INTERRUPTED = 3


class Robot:

    def __init__(self) -> None:
        # Move it
        moveit_commander.roscpp_initialize(sys.argv)    
        
        # Robot Commander
        self.robot = RobotCommander()

        # Planning Scene
        self.scene = PlanningSceneInterface()

        # Move group
        group_name = "panda_manipulator"
        self.move_group = MoveGroupCommander(group_name)

        # Transform Broadcaster
        self.tf_broadcaster = tf.TransformBroadcaster()

        # type publisher
        self.pub = rospy.Publisher('/type', String, queue_size=5)

        # creating pose publisher
        self.pose_pub = rospy.Publisher('/robot/pose', PoseStamped, queue_size=5)

        # creating a state publisher
        self.state_pub = rospy.Publisher('/robot/state', State, queue_size=5)

        # create a service GetCurrentRobotState
        self.service = rospy.Service("/get_current_robot_state", GetCurrentRobotState, self.service_callback)

    def go_to_pose(self):
        
        # pose goals
        pose_goal = Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = 0.4
        pose_goal.position.y = 0.1
        pose_goal.position.z = 0.2

        # visualize
        self.visualize_pose(pose=pose_goal)

        # move
        self.move_group.set_pose_target(pose=pose_goal)

        self.move_group.go(wait=True)

        self.move_group.stop()

    def visualize_pose(self, pose:Pose) -> None:

        self.tf_broadcaster.sendTransform((pose.position.x, pose.position.y, pose.position.z), 
                                           (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
                                           rospy.Time.now(),
                                           "goal",
                                           "panda_link0")



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

            # info
            rospy.loginfo("[MoveIt]: Go to Pose")
            robot.go_to_pose()


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