#!/usr/bin/env python3
# noetic、gazebo,move_baseで、障害物を迂回して動くプログラム課題
#
#
#
#pa#
import time
import rospy
from geometry_msgs.msg import Twist                       # move_base/goal
from geometry_msgs.msg import PoseStamped                 # move_base_simple/goal
from geometry_msgs.msg import PoseWithCovarianceStamped   # move_base_simple/goal
from geometry_msgs.msg import Point, Quaternion
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal,MoveBaseResult
from actionlib_msgs.msg import GoalStatusArray
import actionlib
import math
import random
from random import randint, randrange, random, uniform

class TurtleBotNavigator:
    def __init__(self):
        rospy.init_node('tbot3_navi', anonymous=True)
        self.num_avoid   = 0
        self.odom_enable = 1
        self.target_set  = 0
        self.goal_status = 0
        self.new_goal_x  = 0
        self.new_goal_y  = 0
        self.new_goal_rot_w  = 0
        self.qx = 0
        self.qy = 0
        self.qz = 0
        self.qw = 0
        self.initpos     = Odometry()
        self.initpos_turtle = PoseWithCovarianceStamped()

        # パブリッシャー
        self.initpos_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)           # 初期位置の設定用
        self.goal_pub = rospy.Publisher('move_base_simple/goal' , PoseStamped , queue_size=100)                # 目標設定用 

        # サブスクライバー
        # rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)                                                 # 今の位置を取得
        rospy.Subscriber('/move_base/status',GoalStatusArray , self.goal_result_callback)                       # 到着判定
        
    
        # ナビゲーションのアクションクライアント
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()


    def goal_result_callback(self,msg):
        if self.target_set ==1:
            self.goal_result=GoalStatusArray()
            self.goal_result=msg      
            for goal_st in msg.status_list:
                #rospy.loginfo(goal_st.status) 
                self.goal_status = goal_st.status            # 最新ステータスを取得
            if self.goal_status==3:
                rospy.loginfo("GOAL! & Set Next Goal")
                self.goal_make()                
                self.move_to_goal(self.new_goal_x,self.new_goal_y,self.new_goal_rot_w)                                    
            if self.goal_status==4:
                rospy.loginfo("retry make goal")
                self.goal_make()                
                self.move_to_goal(self.new_goal_x,self.new_goal_y,self.new_goal_rot_w)                                    

    def goal_make(self):
        self.new_goal_x = 0.1*randint(-20,20)
        self.new_goal_y = 0.1*randint(-20,20)
        self.new_goal_rot_w = math.pi*0.1*randint(-20,20)     # yaw
        rospy.loginfo("new goal")
        rospy.loginfo(self.new_goal_x)
        rospy.loginfo(self.new_goal_y)
        rospy.loginfo(self.new_goal_rot_w)
        # 座標チェックをいれたいけど・・・

    def odom_callback(self, msg):
        if self.odom_enable==1 :
            #rospy.loginfo("odom")
            self.initpos=msg
            self.odom_enable = 0
            #rospy.loginfo(self.initpos)

    def quaternion_from_yaw(self, yaw):
        self.qw = math.cos(yaw / 2.0)
        self.qx = 0.0
        self.qy = 0.0
        self.qz = math.sin(yaw / 2.0)

    def move_to_goal(self, x, y, w):
        rospy.loginfo("move_to_goal")
        self.quaternion_from_yaw(w)
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = x
        goal.pose.position.y = y        
        goal.pose.orientation.x = self.qx
        goal.pose.orientation.y = self.qy
        goal.pose.orientation.z = self.qz
        goal.pose.orientation.w = self.qw                
        rospy.loginfo("Sending goal...")
        self.goal_pub.publish(goal)


    def navigate_forever(self):
        rospy.loginfo("Start Navi")
        rate = rospy.Rate(50)  # 5Hz
        while not self.odom_enable==0:
            time.sleep(1)
        rospy.loginfo("Set Initial Pose") # 

        self.initpos_turtle.header.stamp = rospy.Time.now()
        self.initpos_turtle.header.frame_id = "map"
        # self.initpos_turtle.pose.pose.position.x    = -2.0 
        # self.initpos_turtle.pose.pose.position.y    = -0.5
        # self.initpos_turtle.pose.pose.position.z    = 0.0
        # self.initpos_turtle.pose.pose.orientation.w = 1.0
        self.initpos_turtle.pose.pose.position.x    = self.initpos.pose.pose.position.x 
        self.initpos_turtle.pose.pose.position.y    = self.initpos.pose.pose.position.y 
        self.initpos_turtle.pose.pose.position.z    = self.initpos.pose.pose.position.z 
        self.initpos_turtle.pose.pose.orientation.w = self.initpos.pose.pose.orientation.w
        self.initpos_pub.publish(self.initpos_turtle)

        self.move_to_goal( 2.0, 1.0, 1.0 )     # 最初の目標
        self.target_set = 1
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    try:
        navigator = TurtleBotNavigator()
        navigator.navigate_forever()
    except rospy.ROSInterruptException:
        pass
