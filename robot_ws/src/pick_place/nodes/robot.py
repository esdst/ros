#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose, Point
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
from math import tau

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
    
    def go_to_joints(self):

        # get current joint angles
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -tau / 8 # 1/8 of a turn in opposite direction
        joint_goal[2] = 0
        joint_goal[3] = -tau / 4
        joint_goal[4] = 0
        joint_goal[5] = 0
        joint_goal[6] = 0

        # move
        self.move_group.go(joint_goal, wait=True)

        # stop
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

    def add_box(self, name: str, frame_id: str, size: tuple, position=Point) -> None:
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = frame_id
        
        # set the position and orientation
        pose_stamped.pose.position = position
        pose_stamped.pose.orientation.w = 1.0

        self.scene.add_box(name=name, pose=pose_stamped, size=size)

    def remove_box(self, name: str) -> None:
        self.scene.remove_world_object(name=name)

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
        cmd = input("Do you want to publish [y/n/a/i/f] : ")
        
        # check if the user has selected yes
        if (cmd == 'y'):
            robot.publish()
            rospy.loginfo("[Robot]: Publishing")

            robot.publish_pose()
            rospy.loginfo("[Robot]: publishing pose")

            # info
            rospy.loginfo("[MoveIt]: Go to Pose [IK]")
            robot.go_to_pose()


        if (cmd == 'a'):
            #robot.publish_state(robot_state=RobotState.ACTIVE)
            position = Point()
            position.x = 0.4
            position.y = 0.0
            position.z = 0.1

            robot.add_box("wall", "world", (0.1,0.2,0.6), position)

        if (cmd == 'r'):
            #robot.publish_state(robot_state=RobotState.IDLE)
            robot.remove_box("wall")

        if (cmd == 'i'):
            robot.publish_state(robot_state=RobotState.IDLE)

        # if not then shutdown
        if (cmd == 'n'):
            rospy.loginfo("[Robot]: Shutting down")
            rospy.signal_shutdown("User selected N")
            break
        
        if (cmd == 'f'):
            # info
            rospy.loginfo("[MoveIt]: Go to Joint [FK]")
            robot.go_to_joints()
        
        rate.sleep()