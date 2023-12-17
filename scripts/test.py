#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def scan_callback(msg):
    # 障害物が近くにあるかどうかを確認
    if min(msg.ranges) < 1.0:
        avoid_obstacle()

def avoid_obstacle():
    # 障害物を避ける動作を実装
    move_cmd = Twist()
    move_cmd.linear.x = 0.0  # 前進停止
    move_cmd.angular.z = 0.5  # 回転（例）

    # パブリッシュ
    cmd_vel_pub.publish(move_cmd)

if __name__ == '__main__':
    # ノードの初期化
    rospy.init_node('obstacle_avoidance_node')

    # サブスクライバーのセットアップ
    rospy.Subscriber('/scan', LaserScan, scan_callback)

    # パブリッシャーのセットアップ
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    # メインループ
    rospy.spin()
