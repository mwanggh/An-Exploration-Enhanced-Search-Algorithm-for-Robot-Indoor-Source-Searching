#!/usr/bin/env python3

import rospy
from map_client import MapClient
from robot_client import RobotClient
import math
import random
import os
from timeit import default_timer as timer
import pandas as pd
from copy import deepcopy
import pickle
import numpy as np
from visualization_msgs.msg import Marker as VMarker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point as GPoint
import matplotlib
import matplotlib.cm as cm
from jsk_rviz_plugins.msg import OverlayText
from textwrap import dedent
from typing import List

class Frontier_Node():
    def __init__(self, x, y):
        self.x = x
        self.y = y

class Frontier():
    def __init__(self, obs_r=0.20):
        self.obs_r = obs_r
        self.FREE = 0
        self.UNKNOWN = -1
    
    def find(self, map_client:MapClient, mx, my):
        obs_r = round(self.obs_r / map_client.resolution)
        frontiers:List[Frontier_Node] = []
        q = []
        # visited = [[False for _ in range(map_client.height)] for _ in range(map_client.width)]
        visited = np.full((map_client.width, map_client.height), False, dtype=bool)
        
        dx = [0, 1, 0, -1]
        dy = [1, 0, -1, 0]
        
        sx, sy = map_client.get_costmap_x_y(mx, my)
        q.append((sx, sy))
        visited[sx, sy] = True
        
        while not len(q) == 0:
            node = q.pop(0)
            if node[0] < 0 or node[0] >= map_client.width or node[1] < 0 or node[1] >= map_client.height:
                continue
            if map_client.get_cost_from_costmap_x_y(node[0], node[1]) == self.UNKNOWN:
                continue
            xmin = max(0, node[0] - obs_r)
            xmax = min(map_client.width, node[0] + obs_r)
            ymin = max(0, node[1] - obs_r)
            ymax = min(map_client.height, node[1] + obs_r)
            data = map_client.grid_data[ymin:ymax, xmin:xmax]
            if np.amax(data) > self.FREE:
                continue
            
            for i in range(4):
                nx, ny = node[0] + dx[i], node[1] + dy[i]
                if nx < 0 or nx >= map_client.width or ny < 0 or ny >= map_client.height:
                    continue
                if not visited[nx, ny]:
                    visited[nx, ny] = True
                    if map_client.get_cost_from_costmap_x_y(nx, ny) == self.UNKNOWN:
                        frontiers.append(Frontier_Node(node[0], node[1]))
                    elif map_client.get_cost_from_costmap_x_y(nx, ny) == self.FREE:
                        q.append((nx, ny))
                    
        for node in frontiers:
            node.x, node.y = map_client.get_world_x_y(node.x, node.y)
        rospy.loginfo(f'Find frontiers: raw size {len(frontiers)}')
        return frontiers

class RRT_Node():
    def __init__(self, x, y, cost=0.0, parent_ind=None):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent = parent_ind

class RRT_Sample():
    def __init__(self, max_iter=200, sample_min_r=0.5, sample_max_r=3.0,  step=0.3, path_resolution=0.12, obs_r=0.20, near_r=0.5):
        self.max_iter = max_iter
        self.max_r = sample_max_r
        self.min_r = sample_min_r
        self.step = step
        self.path_res = path_resolution
        self.obs_r = obs_r
        self.near_r = near_r
        self.FREE = 0

    def steer(self, to_node:RRT_Node, from_node:RRT_Node):
        path_theta = math.atan2(to_node.y - from_node.y, to_node.x - from_node.x)
        x = from_node.x + self.step * math.cos(path_theta)
        y = from_node.y + self.step * math.sin(path_theta)
        dis = math.hypot(to_node.x - x, to_node.y - y)
        if dis <= self.step:
            return RRT_Node(to_node.x, to_node.y)
        return RRT_Node(x, y)

    def no_obs(self, to_node:RRT_Node, from_node:RRT_Node, map_client:MapClient):
        path_x = []
        path_y = []
        dis = math.hypot(to_node.x - from_node.x, to_node.y - from_node.y)
        n_expand = math.floor(dis / self.path_res)
        path_theta = math.atan2(to_node.y - from_node.y, to_node.x - from_node.x)
        for i in range(n_expand):
            path_x.append(from_node.x + i * self.path_res * math.cos(path_theta))
            path_y.append(from_node.y + i * self.path_res * math.sin(path_theta))
        path_x.append(to_node.x)
        path_y.append(to_node.y)
        for i in range(len(path_x)):
            x, y = map_client.get_costmap_x_y(path_x[i], path_y[i])
            obs_r = int(round(self.obs_r / map_client.resolution))
            xmin = int(max(0, x - obs_r))
            xmax = int(min(map_client.width, x + obs_r))
            ymin = int(max(0, y - obs_r))
            ymax = int(min(map_client.height, y + obs_r))
            data = map_client.grid_data[ymin:ymax, xmin:xmax]
            if np.amax(data) > self.FREE:
                return False
            if np.amin(data) < self.FREE:
                return False
        return True
    
    def sample(self, map_client:MapClient, start_x, start_y):
        start = RRT_Node(start_x, start_y)
        node_list = [start, ]
        for _ in range(self.max_iter):
            # sample new node
            random_theta = random.random() * math.pi * 2 - math.pi
            random_r = self.min_r + random.random() * (self.max_r - self.min_r)
            random_node_x = start.x + random_r * math.cos(random_theta)
            random_node_y = start.y + random_r * math.sin(random_theta)
            dist_list = [(i, math.hypot(node.x - random_node_x, node.y - random_node_y)) for i, node in enumerate(node_list)]
            nearest_node_info = min(dist_list, key=lambda x:x[1])
            nearest_node = node_list[nearest_node_info[0]]
            random_node = RRT_Node(random_node_x, random_node_y)
            new_node = self.steer(random_node, nearest_node)
            new_node.cost = nearest_node.cost + nearest_node_info[1]
            new_node.parent = nearest_node_info[0]
            
            # find near nodes
            if self.no_obs(new_node, nearest_node, map_client):
                dist_list = [(i, math.hypot(node.x - new_node.x, node.y - new_node.y)) for i, node in enumerate(node_list)]
                near_nodes = [item for item in dist_list if item[1] < self.near_r]
                if len(near_nodes) > 0:
                    # find new parent
                    candidate_parents = [(item[0], node_list[item[0]].cost + item[1]) for item in near_nodes \
                                         if node_list[item[0]].cost + item[1] < new_node.cost]
                    if len(candidate_parents) > 0:
                        new_parent_info = min(candidate_parents, key=lambda x:x[1])
                        new_parent = node_list[new_parent_info[0]]
                        new_node_temp = self.steer(new_node, new_parent)
                        if self.no_obs(new_node_temp, new_parent, map_client):
                            new_node = new_node_temp
                            new_node.cost = new_parent.cost + math.hypot(new_parent.x - new_node.x, new_parent.y - new_node.y)
                            new_node.parent = new_parent_info[0]
                    # rewire
                    for item in near_nodes:
                        near_node = node_list[item[0]]
                        cost_t = math.hypot(near_node.x - new_node.x, near_node.y - new_node.y) + new_node.cost
                        if cost_t < near_node.cost:
                            near_node.cost = cost_t
                            near_node.parent = len(node_list)
                node_list.append(new_node)
        return node_list

class Goal_Node():
    def __init__(self, x, y, yaw=None, u=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        # self.do_sample = do_sample
        self.u = u
            
class Banana():
    def __init__(self, mc:MapClient, rc:RobotClient, params):
        self.iteration = None
        self.mc = mc
        self.rc = rc
        self.params = params
        self.rrt_max_r = params['rrt_max_r']
        self.rrt_min_r = params['rrt_min_r']
        self.reach_waypoint_dis_th = params['reach_waypoint_dis_th']
        self.sigma = params['sigma']
        self.beta = params['beta']
        self.obs_r = params['obs_r']
        self.rrt_client = RRT_Sample(params['rrt_max_iter'], sample_min_r=self.rrt_min_r, sample_max_r=self.rrt_max_r, obs_r=self.obs_r)
        self.fontier_client = Frontier(obs_r=self.obs_r)
        self.do_sample = True
        self.goal:Goal_Node = None
        self.gas_hit = False
        self.wind_hit = False
        self.state_ups = 'Upstreaming'
        self.state_btk = 'Backtracking'
        self.state_dws = 'Downstreaming'
        self.state_epr = 'Exploration'
        self.state = self.state_epr
        self.G_epr:List[Goal_Node] = []
        self.G_dws:List[Goal_Node] = []
        self.G_ups:List[Goal_Node] = []
        self.G_dws_last:List[RRT_Node] = []
        self.rrt_nodes_raw:List[RRT_Node] = []
        self.frontiers_raw:List[Frontier_Node] = []
        rospy.loginfo(f'Agent initialized')
        
        self.cal_start_time = None
        self.cal_end_time = None
        self.data_log = []
        self.data_log_map = []
        self.data_targets = []
        self.data_rrt_nodes = []
        self.data_frontiers = []
        
        self.visualize_goal_pub = None
        self.visualize_rrt_tree_pub = None
        self.visualize_targets_pub = None
        self.visualize_text_pub = None
        self.visualize_source_pub = None
    
    def probability(self, x, y):
        # source lcoation estimator
        # Eq. (2)
        kQ = 4.0
        D = 1.0
        tau = 250
        V = self.rc.wind_speed
        phi = self.rc.yaw - self.rc.wind_direction if self.gas_hit else self.rc.yaw - self.rc.wind_direction + math.pi
        lam = math.sqrt(D * tau / (1 + V**2 * tau / (4 * D)))
        dis = math.sqrt(abs(x - self.rc.x)**2 + abs(y - self.rc.y)**2)
        dx = self.rc.x - x
        dy = self.rc.y - y
        
        # pa = kQ / (4 * math.pi * D * dis)
        pa = kQ / (4 * math.pi * D) # Modify the rrt_min_r parameter or use this if you don't want the navigation goal to be too close to the robot
        pb = math.exp(-dis / lam)
        pc = math.exp(-dx * V * math.cos(phi) / (2 * D))
        pd = math.exp(-dy * V * math.sin(phi) / (2 * D))
        p = pa * pb * pc * pd
        return p
    
    def check_reach_goal(self, x, y):
        return math.hypot(self.rc.x - x, self.rc.y - y) < self.reach_waypoint_dis_th
    
    def del_node_reached(self, lst:List[Goal_Node]):
        del_list = []
        for ind, node in enumerate(lst):
            if self.check_reach_goal(node.x, node.y):
                del_list.append(ind)
        for ind in sorted(del_list, reverse=True):
            rospy.loginfo(f'Reach target: index {ind} x {lst[ind].x:.2f} y {lst[ind].y:.2f}, del')
            del lst[ind]
    
    def observe(self, itration):
        self.iteration = itration
        # get measures
        self.mc.update_grid_map()
        self.rc.update_pose()
        self.rc.update_real_pose()
        self.rc.update_anemometer()
        self.rc.update_gas()
        
        self.cal_start_time = timer()
        
        self.gas_hit = self.rc.gas <= self.params['gas_sensor_hit_th']
        self.wind_hit = self.rc.wind_speed >= self.params['anemometer_speed_th']
        rospy.loginfo(f"gas_hit {self.gas_hit} wind_hit {self.wind_hit} gas_speed_th {self.params['gas_sensor_hit_th']} wind_th {self.params['anemometer_speed_th']}")
        
        if self.goal is None:
            self.goal = Goal_Node(self.rc.x, self.rc.y)
        self.del_node_reached(self.G_ups)
        self.del_node_reached(self.G_dws)
        
        # determine status (priority)
        self.do_sample = False
        if len(self.G_ups) > 0 or (self.gas_hit and self.wind_hit):
            self.state = self.state_btk
            if self.gas_hit and self.wind_hit:
                self.state = self.state_ups
            if len(self.G_ups) == 0:
                self.do_sample = True
            if self.check_reach_goal(self.goal.x, self.goal.y) and self.gas_hit and self.wind_hit:
                self.do_sample = True
            self.G_dws.clear()
            self.G_epr.clear()
        elif len(self.G_dws) > 0 or self.wind_hit:
            if len(self.G_dws) == 0: self.do_sample = True
            self.state = self.state_dws
            self.G_epr.clear()
        else:
            self.do_sample = True
            self.state = self.state_epr
        
        rospy.loginfo(f'State {self.state}')
        rospy.loginfo(f'Reach goal {self.check_reach_goal(self.goal.x, self.goal.y)} Sample {self.do_sample}')
        rospy.loginfo(f'Set size: G_ups {len(self.G_ups)} G_dws {len(self.G_dws)} G_epr {len(self.G_epr)}')
    
    def rrt_sample(self):
        node_list_raw = self.rrt_client.sample(self.mc, self.rc.x, self.rc.y)
        node_list = []
        for node in node_list_raw:
            do_append = True
            # delete nodes that are very close to the robot
            if math.hypot(node.x - self.rc.x, node.y - self.rc.y) < self.rrt_min_r:
                do_append = False
                continue
            # delete the nodes that was last sampled when the robot was in the Downstreaming state
            if self.state == self.state_dws and len(self.G_dws_last) > 0:
                for item in self.G_dws_last:
                    if math.hypot(node.x - item.x, node.y - item.y) < self.reach_waypoint_dis_th:
                        do_append = False
                        break
            if do_append:
                node_list.append(Goal_Node(node.x, node.y))
        self.rrt_nodes_raw = node_list_raw
        self.G_dws_last = deepcopy(node_list_raw)
        rospy.loginfo(f'RRT nodes: size {len(node_list)}')
        return node_list
    
    def estimate(self):
        # Eq. (5)
        if self.state == self.state_ups or self.state == self.state_btk:
            if self.do_sample:
                self.G_ups = self.rrt_sample()
            for node in self.G_ups:
                node.u = self.sigma * node.u + self.probability(node.x, node.y)
        # Eq. (9)
        if self.state == self.state_dws:
            if self.do_sample:
                self.G_dws = self.G_dws + self.rrt_sample()
            if len(self.G_dws) == 0:
                return
            j1_pro = [self.probability(item.x, item.y) for item in self.G_dws]
            j1_max = max(j1_pro)
            j1 = [j1_pro[i] / j1_max for i in range(len(j1_pro))]
            j2_dis = [math.hypot(self.rc.x - item.x, self.rc.y - item.y) for item in self.G_dws]
            j2_max = max(j2_dis)
            j2 = [j2_dis[i] / j2_max for i in range(len(j2_dis))]
            for ind, node in enumerate(self.G_dws):
                j = self.beta * j1[ind] + (1 - self.beta) * j2[ind]
                node.u = self.sigma * node.u + j
        # Eq. (20)
        if self.state == self.state_epr:
            if self.do_sample:
                self.frontiers_raw = self.fontier_client.find(self.mc, self.rc.x, self.rc.y)
                self.G_epr = [Goal_Node(item.x, item.y, u=math.hypot(item.x - self.rc.x, item.y - self.rc.y)) for item in self.frontiers_raw]
        rospy.loginfo(f'Set size: G_ups {len(self.G_ups)} G_dws {len(self.G_dws)} G_epr {len(self.G_epr)}')
    
    def generate_ramdom_goal(self):
        MAP_FREE = 0
        while True:
            random_theta = random.random() * math.pi * 2 - math.pi
            random_r = random.random() * self.rrt_max_r
            x = self.goal.x + random_r * math.cos(random_theta)
            y = self.goal.y + random_r * math.sin(random_theta)
            node_x, node_y = self.mc.get_costmap_x_y(x, y)
            if not (0 <= node_x < self.mc.width and 0 <= node_y < self.mc.height):
                continue
            if not self.mc.get_cost_from_costmap_x_y(node_x, node_y) == MAP_FREE:
                continue
            return Goal_Node(x, y)
    
    def evaluate(self):
        if self.state == self.state_ups or self.state == self.state_btk:
            if not len(self.G_ups) == 0:
                # Eq. (6)
                self.goal = max(self.G_ups, key=lambda x: x.u)
            else:
                self.state = self.state_epr
                rospy.loginfo(f'Empty G_ups -> Set state {self.state}')
        if self.state == self.state_dws:
            if not len(self.G_dws) == 0:
                # Eq. (10)
                self.goal = min(self.G_dws, key=lambda x: x.u)
            else:
                self.state = self.state_epr
                rospy.loginfo(f'Empty G_dws -> Set state {self.state}')
        if self.state == self.state_epr:
            if not len(self.G_epr) == 0:
                # Eq. (21)
                self.goal = min(self.G_epr, key=lambda x: x.u)
            else:
                # if both RRT sampling and frontier sampling fail, generate a random goal
                rospy.loginfo(f'Empty G_epr -> Set random goal')
                self.goal = self.generate_ramdom_goal()
                    
        rospy.loginfo(f'Set goal: x {self.goal.x:.2f} y {self.goal.y:.2f}')
    
    def navigate(self):
        self.rc.send_goal(self.goal.x, self.goal.y, self.goal.yaw)
        self.cal_end_time = timer()
    
    def in_motion(self):
        return not self.rc.move_base_done
    
    def visualize(self):
        if self.visualize_text_pub is None:
            self.visualize_text_pub = rospy.Publisher('v_text', OverlayText, queue_size=2)
        text = OverlayText()
        text.width = 580
        text.left = 10
        text.height = 400
        text.text_size = 25
        text.line_width = 1
        t = dedent(f'''
                    Iteration: {self.iteration}
                    Robot state: {self.state}
                    Valid airflow measure: {str(self.wind_hit)}
                    Valid concentration measure: {str(self.gas_hit)}
                    G_ups size: {len(self.G_ups)}
                    G_dws size: {len(self.G_dws)}
                    G_epr size: {len(self.G_epr)}
                    ''')
        text.text = t
        text.fg_color = ColorRGBA(1.0, 0.0, 0.0, 1.0)
        text.bg_color = ColorRGBA(0.0, 0.0, 0.0, 0.2)
        self.visualize_text_pub.publish(text)
        
        if self.visualize_source_pub is None:
            self.visualize_source_pub = rospy.Publisher('v_source', VMarker, queue_size=2)
        marker = VMarker()
        marker.ns="v_source"
        marker.id = 0
        marker.header.frame_id = self.mc.map_frame
        marker.header.stamp = rospy.get_rostime()
        marker.action = VMarker.ADD
        marker.type = VMarker.LINE_LIST
        marker.lifetime = rospy.Duration(1)
        shape_vertices = 4;
        angle_offset = 2* math.pi / shape_vertices;
        source_x = self.params['source_x']
        source_y = self.params['source_y']
        shape_r = 0.1
        for i in ([k for k in range(shape_vertices)] + [0, ]):
            point1 = GPoint()
            point1.x = source_x + shape_r * math.cos(angle_offset * i)
            point1.y = source_y + shape_r * math.sin(angle_offset * i)
            point1.z = 0.0
            marker.points.append(point1)
            point2 = GPoint()
            point2.x = source_x + shape_r * math.cos(angle_offset * (i+1))
            point2.y = source_y + shape_r * math.sin(angle_offset * (i+1))
            point2.z = 0.0
            marker.points.append(point2)
        marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)
        marker.pose.orientation.w = 1.0
        marker.scale = Vector3(0.02, 0.02, 0.02)
        self.visualize_source_pub.publish(marker)
        
        if self.visualize_targets_pub is None:
            self.visualize_targets_pub = rospy.Publisher('v_targets', VMarker, queue_size=2)
        marker = VMarker()
        marker.ns="targets_v"
        marker.id = 0
        marker.header.frame_id = self.mc.map_frame
        marker.header.stamp = rospy.get_rostime()
        marker.action = VMarker.ADD
        marker.type = VMarker.POINTS
        marker.lifetime = rospy.Duration(1)
        lst = None
        if self.state == self.state_ups or self.state == self.state_btk:
            lst = self.G_ups
        elif self.state == self.state_dws:
            lst = self.G_dws
        else:
            lst = self.G_epr
        if len(lst) == 0:
            return
        norm = matplotlib.colors.Normalize(vmin=min(lst, key=lambda x: x.u).u, vmax=max(lst, key=lambda x: x.u).u, clip=True)
        mapper = cm.ScalarMappable(norm=norm, cmap=cm.Blues)
        for i in range(len(lst)):
            point = GPoint()
            point.x = lst[i].x
            point.y = lst[i].y
            point.z = 0.0
            marker.points.append(point)
            color = mapper.to_rgba(lst[i].u)
            marker.colors.append(ColorRGBA(color[0], color[1], color[2], color[3]))
        marker.pose.orientation.w = 1.0
        marker.scale = Vector3(0.05, 0.05, 0.05)
        self.visualize_targets_pub.publish(marker)
        
        if self.visualize_goal_pub is None:
            self.visualize_goal_pub = rospy.Publisher('v_goal', VMarker, queue_size=2)
        marker = VMarker()
        marker.ns="goal_v"
        marker.id = 0
        marker.header.frame_id = self.mc.map_frame
        marker.header.stamp = rospy.get_rostime()
        marker.action = VMarker.ADD
        marker.type = VMarker.POINTS
        marker.lifetime = rospy.Duration(1)
        point = GPoint()
        point.x = self.goal.x
        point.y = self.goal.y
        point.z = 0.0
        marker.points.append(point)
        marker.colors.append(ColorRGBA(1.0, 0.0, 0.0, 1.0))
        marker.pose.orientation.w = 1.0
        marker.scale = Vector3(0.05, 0.05, 0.05)
        self.visualize_goal_pub.publish(marker)
        
        if self.visualize_rrt_tree_pub is None:
            self.visualize_rrt_tree_pub = rospy.Publisher('v_rrt_tree', VMarker, queue_size=2)
        if self.state == self.state_epr:
            return
        marker = VMarker()
        marker.ns="v_rrt_tree"
        marker.id = 0
        marker.header.frame_id = self.mc.map_frame
        marker.header.stamp = rospy.get_rostime()
        marker.action = VMarker.ADD
        marker.type = VMarker.LINE_LIST
        marker.lifetime = rospy.Duration(1)
        for node in self.rrt_nodes_raw:
            # print(f'x {node.x} y {node.y} parent_ind {node.parent}')
            if node.parent is None:
                continue
            parent_node = self.rrt_nodes_raw[node.parent]
            # print(f'parent ind {node.parent} x {parent_node.x} y {parent_node.y}')
            parent_point = GPoint()
            parent_point.x = parent_node.x
            parent_point.y = parent_node.y
            parent_point.z = 0.0
            marker.points.append(parent_point)
            point = GPoint()
            point.x = node.x
            point.y = node.y
            point.z = 0.0
            marker.points.append(point)
        marker.color = ColorRGBA(0.0, 1.0, 1.0, 1.0)
        marker.pose.orientation.w = 1.0
        marker.scale = Vector3(0.01, 0.01, 0.01)
        self.visualize_rrt_tree_pub.publish(marker)
        
    def record_data(self):
        # data_log_header = ['it', 'ros_time', 'state', 'cal_start_time', 'cal_end_time', 
        #                     'goal_x', 'goal_y', 'goal_yaw',
        #                     'robot_x', 'robot_y', 'robot_z', 'robot_yaw',
        #                     'robot_real_x', 'robot_real_y', 'robot_real_z', 'robot_real_yaw',
        #                     'gas', 'gas_hit', 'wind_speed', 'wind_hit', 'wind_dir',
        #                     'source_x', 'source_y', 'sample', 'G_ups_size', 'G_dws_size', 'G_epr_size']
        self.data_log.append([self.iteration, rospy.get_time(), self.state, self.cal_start_time, self.cal_end_time,
                              self.goal.x, self.goal.y, self.goal.yaw,
                              self.rc.x, self.rc.y, self.rc.z, self.rc.yaw,
                              self.rc.real_x, self.rc.real_y, self.rc.real_z, self.rc.real_yaw,
                              self.rc.gas, self.gas_hit, self.rc.wind_speed, self.wind_hit, self.rc.wind_direction,
                              self.params['source_x'], self.params['source_y'], self.do_sample,
                              len(self.G_ups), len(self.G_dws), len(self.G_epr)])
        self.data_log_map.append(deepcopy(self.mc.raw_grid_data))
        targets_2 = [(self.iteration, 2, item.x, item.y, item.yaw, item.u) for item in self.G_ups]
        targets_1 = [(self.iteration, 1, item.x, item.y, item.yaw, item.u) for item in self.G_dws]
        targets_0 = [(self.iteration, 0, item.x, item.y, item.yaw, item.u) for item in self.G_epr]
        self.data_targets = self.data_targets + targets_2 + targets_1 + targets_0
        frontiers_lst = [(self.iteration, item.x, item.y) for item in self.frontiers_raw]
        self.data_frontiers = self.data_frontiers + frontiers_lst
        rrt_nodes_lst = [(self.iteration, ind, item.x, item.y, item.parent, item.cost) for ind, item in enumerate(self.rrt_nodes_raw)]
        self.data_rrt_nodes = self.data_rrt_nodes + rrt_nodes_lst
        rospy.loginfo(f'Iter {self.iteration} cost time {self.cal_end_time - self.cal_start_time:.5f}')
    
    def save_data(self, file_path):
        data_log_header = ['it', 'ros_time', 'state', 'cal_start_time', 'cal_end_time', 
                            'goal_x', 'goal_y', 'goal_yaw',
                            'robot_x', 'robot_y', 'robot_z', 'robot_yaw',
                            'robot_real_x', 'robot_real_y', 'robot_real_z', 'robot_real_yaw',
                            'gas', 'gas_hit', 'wind_speed', 'wind_hit', 'wind_dir',
                            'source_x', 'source_y', 'sample', 'G_ups_size', 'G_dws_size', 'G_epr_size']
        data_file = os.path.join(file_path, 'data.csv')
        rospy.loginfo(f'Save data to {data_file}')
        df = pd.DataFrame(self.data_log, columns=data_log_header)
        df.to_csv(data_file, index=False, float_format='%.6f')
        
        map_file = os.path.join(file_path, 'map_pickle')
        rospy.loginfo(f'Save data to {map_file}')
        with open(map_file, 'wb') as f:
            pickle.dump(self.data_log_map, f)
        
        if len(self.data_targets) > 0:
            targets_file = os.path.join(file_path, 'targets.csv')
            rospy.loginfo(f'Save data to {targets_file}')
            df = pd.DataFrame(np.array(self.data_targets), columns=['it', 'level', 'x', 'y', 'yaw', 'u'])
            df.to_csv(targets_file, index=False, float_format='%.6f')
        
        if len(self.data_frontiers) > 0:
            frontiers_file = os.path.join(file_path, 'frontiers.csv')
            rospy.loginfo(f'Save data to {frontiers_file}')
            df = pd.DataFrame(np.array(self.data_frontiers), columns=['it', 'x', 'y'])
            df.to_csv(frontiers_file, index=False, float_format='%.6f')
        
        if len(self.data_rrt_nodes) > 0:
            rrt_file = os.path.join(file_path, 'rrt_nodes.csv')
            rospy.loginfo(f'Save data to {rrt_file}')
            df = pd.DataFrame(np.array(self.data_rrt_nodes), columns=['it', 'ind', 'x', 'y', 'parent', 'cost'])
            df.to_csv(rrt_file, index=False, float_format='%.6f')