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
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal,MoveBaseResult
from actionlib_msgs.msg import GoalStatusArray
import actionlib
import math

class TurtleBotNavigator:
    def __init__(self):
        rospy.init_node('tbot3_navi', anonymous=True)
        self.num_avoid   = 0
        self.odom_enable = 1
        self.target_set  = 0
        self.goal_status = 0
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
                self.goal_status = goal_st.status                               # 最新ステータスを取得
            #rospy.loginfo(self.goal_result.status_list[0].status)              # index error がでる。最初は用意されていないようだ。
            #rospy.loginfo(self.goal_result.status_list[0].status)       
            if self.goal_status==3:
                rospy.loginfo("GOAL! & Set Next Goal")
                self.move_to_goal(-2.0,-1.0)                                    # ここで、有効なゴールを毎回設定する　乱数で？広さはわかる？有効ゴールか？
                time.sleep(1)
            # resultgoal=msg.



    # def simgo_callback(self, msg):
    #     rospy.loginfo("simgo")
    #     pass 
        

    # def scan_callback(self, msg):
    #     # 障害物が近くにあるかどうかを確認        #rospy.loginfo("scan")

    #     # 前方60度の範囲のデータを抽出
    #     front_data = msg.ranges[-30::30]   

    #     if min(front_data)< 0.5:
    #         self.avoid_obstacle()
    #     # else:
    #     #     move_cmd = Twist()
    #     #     move_cmd.linear.x = 0.1   # 前進停止
    #     #     move_cmd.angular.z = 0 #  回転（例）
    #     #     # パブリッシュ
    #     #     #self.cmd_vel_pub.publish(move_cmd)  
        
    # def avoid_obstacle(self):
    #     # 障害物を避ける動作を実装
    #     #rospy.loginfo("avoid {}",num_avoid)
    #     # num_avoid = num_avoid + 1
    #     # move_cmd = Twist()
    #     # move_cmd.linear.x = 0.0   # 前進停止
    #     # move_cmd.angular.z = 0.1 #  回転（例）

    #     # # パブリッシュ
    #     # self.cmd_vel_pub.publish(move_cmd)
    #     pass

    def odom_callback(self, msg):
        if self.odom_enable==1 :
            rospy.loginfo("odom")
            self.initpos=msg
            self.odom_enable = 0
            rospy.loginfo(self.initpos)

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
        # rospy.loginfo("OK")
        # self.goal_pub.wait_for_result()

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

        move_cmd = Twist()
        move_cmd.linear.x = 0.5   # 前進停止
        move_cmd.angular.z = 0 #  回転（例）
        # パブリッシュ
        # self.cmd_vel_pub.publish(move_cmd)  
        self.move_to_goal(2.0,1.0)
        self.target_set = 1
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
