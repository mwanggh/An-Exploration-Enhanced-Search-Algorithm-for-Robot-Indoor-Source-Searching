#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid
import tf
import numpy as np
from copy import deepcopy

class MapClient():
    def __init__(self, map_topic):
        self.raw_grid_data = None
        self.grid_data = None
        self.grid_metadata = None
        self.resolution = None
        self.width = None
        self.height = None
        self.origin = None
        self.origin_x = None
        self.origin_y = None
        self.map_frame = None
        self.tf_listener = tf.TransformListener()
        
        # init map subscriber
        self.map_sub = rospy.Subscriber(map_topic, OccupancyGrid, self.map_cb, queue_size=3)
        while self.raw_grid_data is None and self.grid_data is None and not rospy.is_shutdown():
            rospy.loginfo(f'Waiting for {str(self.map_sub.resolved_name)}')
            rospy.sleep(0.1)
        rospy.loginfo(f'Map client initialized')
        self.update_grid_map()
    
    def update_grid_map(self):
        data = deepcopy(self.raw_grid_data)
        self.grid_metadata = data.info
        self.grid_data = np.array(data.data, dtype=np.int8).reshape(data.info.height, data.info.width)
        self.resolution = self.grid_metadata.resolution
        self.width = self.grid_metadata.width
        self.height = self.grid_metadata.height
        self.origin_x = self.grid_metadata.origin.position.x
        self.origin_y = self.grid_metadata.origin.position.y
        self.origin = (self.origin_x, self.origin_y)
        self.map_frame = data.header.frame_id
        rospy.loginfo(f'Update grid map: width {self.width:.2f} height {self.height:.2f} origin_x {self.origin_x:.2f} origin_y {self.origin_y:.2f}')
        return data
    
    def map_cb(self, data):
        # rospy.loginfo("Got a full OccupancyGrid update")
        self.raw_grid_data = data
    
    def get_world_x_y(self, costmap_x, costmap_y):
        world_x = costmap_x * self.resolution + self.origin_x
        world_y = costmap_y * self.resolution + self.origin_y
        return world_x, world_y

    def get_costmap_x_y(self, world_x, world_y):
        costmap_x = int(round((world_x - self.origin_x) / self.resolution))
        costmap_y = int(round((world_y - self.origin_y) / self.resolution))
        return costmap_x, costmap_y
    
    def get_cost_from_world_x_y(self, x, y):
        cx, cy = self.get_costmap_x_y(x, y)
        return self.get_cost_from_costmap_x_y(cx, cy)

    def get_cost_from_costmap_x_y(self, x, y):
        if self.is_in_gridmap(x, y):
            # data comes in row-major order http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/OccupancyGrid.html
            # first index is the row, second index the column
            return self.grid_data[y][x]
        else:
            return -10
    
    def is_in_gridmap(self, x, y):
        # x -> width, y -> height
        if -1 < x < self.width and -1 < y < self.height:
            return True
        else:
            return False
        
    def get_max_cost_r(self, mx, my, mr):
        r = round(mr / self.resolution)
        x, y = self.get_costmap_x_y(mx, my)
        xmin = max(0, x - r)
        xmax = min(self.grid_data.shape[1] - 1, x + r)
        ymin = max(0, y - r)
        ymax = min(self.grid_data.shape[0] - 1, y + r)
        # print(mx, my, mr, x, y, r, xmin ,xmax, ymin, ymax)
        data = self.grid_data[ymin:ymax+1, xmin:xmax+1]
        return np.amax(data)
        