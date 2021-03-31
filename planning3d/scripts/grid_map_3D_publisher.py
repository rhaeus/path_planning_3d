#!/usr/bin/env python

import rospy
from grid_map_3D import GridMap3D
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray



def publish_map(map):
    pub = rospy.Publisher('/cf1/grid_map_3D', Marker, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        pub.publish(map)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('grid_map_3D_publisher', anonymous=True)
    # load map
    map_path = rospy.get_param('~map_file_path')
    resolution = rospy.get_param('~map_resolution')
    inflation_radius = rospy.get_param('~inflation_radius')

    map = GridMap3D(map_path, resolution, inflation_radius)
    msg = map.get_ros_message()
    
    # publish map
    try:
        publish_map(msg)
    except rospy.ROSInterruptException:
        pass