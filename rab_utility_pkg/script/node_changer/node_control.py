#!/usr/bin/env python
# -*- coding: utf-8 -*-

from subprocess import *

#公園内waypointの起動
call(['rosrun','rab_utility_pkg','park_wp'])

#公園から探索エリアまでのwaypoint起動
call(['rosrun','rab_utility_pkg','park2search_wp'])

#探索エリアのwaypoint起動
call(['rosrun','rab_utility_pkg','1st_search_wp'])

#探索エリアからロッテリアまでのwaypoint起動
call(['rosrun','rab_utility_pkg','search2lotteria_wp'])

#mapを切り替えてmove_base再起動
p=Popen(['roslaunch','nda_robot_2dnav move_base_lotteria2goal.launch'])

#ロッテリアから探索エリアまでのwaypoint起動
call(['rosrun','rab_utility_pkg','lotteria2search_wp'])

#探索エリアのwaypoint起動
call(['rosrun','rab_utility_pkg','2nd_search_wp'])

#探索エリアから横断歩道までのwaypoint起動
call(['rosrun','rab_utility_pkg','search2pedestrian_signal_wp'])

#ここに横断歩道プログラム

#折り返し地点のwaypoint起動
call(['rosrun','rab_utility_pkg','turnaround_point_wp'])

#横断歩道からゴールまでのwaypoint起動
call(['rosrun','rab_utility_pkg','pedestrian_signal2goal_wp'])