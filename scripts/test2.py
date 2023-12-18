#!/usr/bin/env python3
# noetic、gazebo,move_baseで、障害物を迂回して動くプログラム課題
#
#
#
#pa#
import rospy
from geometry_msgs.msg import Twist         # move_base/goal
from geometry_msgs.msg import PoseStamped   # move_base_simple/goal
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal,MoveBaseResult
import actionlib
import math

class TurtleBotNavigator:
    def __init__(self):
        rospy.init_node('tbot3_navi', anonymous=True)
        self.num_avoid=0
        self.goal_result=MoveBaseResult()
        # パブリッシャー
        # self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # サブスクライバー
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/move_bese/result',MoveBaseResult, self.goal_result_callback)
        self.goal_pub = rospy.Publisher('move_base_simple/goal' , PoseStamped , queue_size=100)        

        # ナビゲーションのアクションクライアント
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()


    def goal_result_callback(self,msg):
        rospy.loginfo("result")
        self.goal_result=msg
        rospy.loginfo(self.goal_result)       
        if self.goal_result==3:
            rospy.loginf("GOAL!")
        # resultgoal=msg.



    def simgo_callback(self, msg):
        rospy.loginfo("simgo")
        pass 
        

    def scan_callback(self, msg):
        # 障害物が近くにあるかどうかを確認        #rospy.loginfo("scan")

        # 前方60度の範囲のデータを抽出
        front_data = msg.ranges[-30::30]   

        if min(front_data)< 0.5:
            self.avoid_obstacle()
        # else:
        #     move_cmd = Twist()
        #     move_cmd.linear.x = 0.1   # 前進停止
        #     move_cmd.angular.z = 0 #  回転（例）
        #     # パブリッシュ
        #     #self.cmd_vel_pub.publish(move_cmd)  
        
    def avoid_obstacle(self):
        # 障害物を避ける動作を実装
        #rospy.loginfo("avoid {}",num_avoid)
        # num_avoid = num_avoid + 1
        # move_cmd = Twist()
        # move_cmd.linear.x = 0.0   # 前進停止
        # move_cmd.angular.z = 0.1 #  回転（例）

        # # パブリッシュ
        # self.cmd_vel_pub.publish(move_cmd)
        pass

    def odom_callback(self, msg):
        rospy.loginfo("odom")
        # ロボットの位置を確認して目標地点を設定
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        goal_x = current_x + 0.2 # 2メートル前進
        goal_y = current_y 

        # self.move_to_goal(goal_x, goal_y)

    def move_to_goal(self, x, y):
        rospy.loginfo("move_to_goal")

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = x
        goal.pose.position.y = y        
        goal.pose.orientation.w = 1.0

        rospy.loginfo("Sending goal...")
        self.goal_pub.publish(goal)
        #while not self.goal_result==3:
            #rospy.loginfo(self.goal_result)
        #    pass
        rospy.loginfo("OK")
        # self.goal_pub.wait_for_result()

        # if self.goal_pub.get_state() == actionlib.GoalStatus.SUCCEEDED:
        #     rospy.loginfo("Goal reached!")

        # goal = MoveBaseGoal()
        # goal.target_pose.header.frame_id = 'map'
        # goal.target_pose.header.stamp = rospy.Time.now()

        # goal.target_pose.pose.position.x = x
        # goal.target_pose.pose.position.y = y
        # goal.target_pose.pose.orientation.w = 1.0

        # rospy.loginfo("Sending goal...")
        # self.move_base_client.send_goal(goal)
        # self.move_base_client.wait_for_result()

        # if self.move_base_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        #     rospy.loginfo("Goal reached!")
        #     # ロボットの位置を確認して目標地点を設定
        #     current_x = msg.pose.pose.position.x
        #     current_y = msg.pose.pose.position.y

        #     goal_x = current_x + 0.1 # 2メートル前進
        #     goal_y = current_y 

        #     self.move_to_goal(goal_x, goal_y)
          
        # else:
        #     rospy.loginfo("Failed to reach goal.")

    def navigate_forever(self):
        rate = rospy.Rate(50)  # 5Hz

        move_cmd = Twist()
        move_cmd.linear.x = 0.5   # 前進停止
        move_cmd.angular.z = 0 #  回転（例）
        # パブリッシュ
        # self.cmd_vel_pub.publish(move_cmd)  
        self.move_to_goal(2.0,1.0)

        while not rospy.is_shutdown():
            # self.move_to_goal(0.0, 0.0)  # 中央に戻る
            # self.move_to_goal(1.0, 1.0)  # 中央に戻る
            rate.sleep()

if __name__ == '__main__':
    try:
        navigator = TurtleBotNavigator()
        navigator.navigate_forever()
    except rospy.ROSInterruptException:
        pass
