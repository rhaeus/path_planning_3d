#!/usr/bin/env python
from __future__ import print_function
import sys
import json 
import math
import numpy as np
 

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from cf_msgs.srv import DronePath, DronePathRequest, DronePathResponse 

from grid_map_3D import GridMap3D
from a_star_3D import AStar3D


global map_file_path
global map_resolution
global inflation_radius

def plan_path(req):
    global map_file_path
    global map_resolution
    global inflation_radius

    grid_map = GridMap3D(map_file_path, map_resolution, inflation_radius) 

    start_pose, goal_pose = req.start, req.goal

    start_index = grid_map.coord_to_grid_index((start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.position.z))
    # print("start_index: ", start_index)
    goal_index = grid_map.coord_to_grid_index((goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z))
    # print("goal_index: ", goal_index)

    astar = AStar3D(grid_map)

    print("planning path in 3D...")
    print("start: ", start_pose)
    print("goal:", goal_pose)
    path_indices = astar.plan(start_index, goal_index)
    # print("path", path_indices)
    
    if len(path_indices) <= 1:
        print("!no path found!")
    else:
        print("path found")
        # print("sparsening path...")
        # path_indices = astar.sparsen_path(path_indices)
        # print("sparse path", path_indices)

    print("planning done!")

    path = Path()
    path.header.stamp = rospy.Time.now()
    path.header.frame_id = 'map'
    path.header.seq = 0
    for i in range(len(path_indices)):
        temp = PoseStamped()
        temp.header.stamp = rospy.Time.now()
        temp.header.frame_id = 'map'
        temp.header.seq = i
        (x, y, z) = grid_map.grid_index_to_coord((path_indices[i]))
        # print("waypoint: ", x,y)
        temp.pose.position.x = x
        temp.pose.position.y = y
        temp.pose.position.z = z
        temp.pose.orientation.x = 0
        temp.pose.orientation.y = 0
        temp.pose.orientation.z = 0
        temp.pose.orientation.w = 1

        path.poses.append(temp)

        
    return DronePathResponse(path)

def path_server():
    rospy.init_node('path_server_3D')

    global map_file_path
    global map_resolution
    global inflation_radius

    map_file_path = rospy.get_param('~map_file_path')
    map_resolution = rospy.get_param('~map_resolution', 0.1)
    inflation_radius = rospy.get_param('~inflation_radius', 0.1)

    s = rospy.Service('drone_path_3D', DronePath, plan_path)
    print("Ready to plan 3D path with A*.")
    rospy.spin()



if __name__ == "__main__":
    path_server()

