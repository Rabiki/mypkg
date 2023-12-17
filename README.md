# はじめてburgerが動いたバージョン
# move_baseを正しく設定できていないと思われる。
# 障害物があると勝手に迂回するための回転を開始するようにしているがmove_baseが自分でやるはず。
# 
# 起動順
# roscore
# roslaunch turtlebot3_gazebo turtlebot3_world.launch
# roslaunch turtlebot3_navigation turtlebot3_navigation.launch
# rosrun mypkg test2.py
# 
# 
# test.pyは、ただ命令で進むだけ
# test2.pyはmove_baseを起動している
