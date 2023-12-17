#!/usr/bin/env python3

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
import math
import random

class TurtleBot3Explore:
    def __init__(self):
        rospy.init_node('turtlebot3_explore_node', anonymous=True)

        # MoveBaseクライアントの初期化
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()

        # マップの中心付近から探索を始める
        self.current_pose = PoseStamped()
        self.current_pose.header.frame_id = 'map'
        self.current_pose.pose.position.x = 0.0
        self.current_pose.pose.position.y = 0.0
        self.current_pose.pose.position.z = 0.0
        self.current_pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

    def send_goal(self, pose):
        goal = MoveBaseGoal()
        goal.target_pose = pose

        self.move_base_client.send_goal(goal)
        self.move_base_client.wait_for_result()

    def generate_random_goal(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = random.uniform(-5.0, 5.0)
        goal.pose.position.y = random.uniform(-5.0, 5.0)
        goal.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

        return goal

    def explore(self):
        while not rospy.is_shutdown():
            # ランダムな位置に移動
            random_goal = self.generate_random_goal()
            self.send_goal(random_goal)

            # 移動が完了するまで待機
            status = self.move_base_client.get_state()
            while status not in [GoalStatus.SUCCEEDED, GoalStatus.ABORTED, GoalStatus.PREEMPTED]:
                status = self.move_base_client.get_state()
                rospy.sleep(1.0)

if __name__ == '__main__':
    explorer = TurtleBot3Explore()
    explorer.explore()
