#!/usr/bin/env python3

import rospy
import math
from map_client import MapClient
from robot_client import RobotClient
from banana import Banana
import os

class FinishCheck():
    def __init__(self, rc:RobotClient, params):
        self.rc = rc
        self.params = params
        self.robot_stuck_pose = None
        self.robot_stuck_last_time = None
        self.robot_stuck_difind_source_th = 0.2
        
    def check(self, it):
        dis = math.sqrt(abs(self.params['source_x'] - self.rc.real_x)**2 + abs(self.params['source_y'] - self.rc.real_y)**2)
        rospy.loginfo(f"Distance to soruce: dist {dis:.2f} find_threshold {self.params['find_source_th']:.2f}")
        # find source
        if  dis <= self.params['find_source_th']:
            rospy.loginfo('STOP MAIN: FIND_SOURCE')
            rospy.loginfo(f"source pose: x {self.params['source_x']:.2f} y {self.params['source_y']:.2f}")
            rospy.loginfo(f'robot pose: x {self.rc.real_x:.2f} y {self.rc.real_y:.2f}')
            return True
        # reach max iteration
        if it >= self.params['max_iter']:
            rospy.loginfo('STOP MAIN: REACH_MAX_ITER')
            rospy.loginfo(f"iter: {it} max_iter {self.params['max_iter']}")
            return True
        # robot stuck
        if self.robot_stuck_pose is None:
            self.robot_stuck_pose = (self.rc.real_x, self.rc.real_y)
            self.robot_stuck_last_time = rospy.get_time()
        else:
            dis = math.sqrt(abs(self.robot_stuck_pose[0] - self.rc.real_x)**2 + abs(self.robot_stuck_pose[1] - self.rc.real_y)**2)
            if dis > self.robot_stuck_difind_source_th:
                self.robot_stuck_pose = (self.rc.real_x, self.rc.real_y)
                self.robot_stuck_last_time = rospy.get_time()
        robot_stuck_d = rospy.get_time() - self.robot_stuck_last_time
        rospy.loginfo(f"Robot stuck time {robot_stuck_d:.2f} threshold {self.params['max_stuck_time']:.2f}")
        if robot_stuck_d > self.params['max_stuck_time']:
            rospy.loginfo('STOP MAIN: ROBOT_STUCK')
            rospy.loginfo(f"Robot stuck time {robot_stuck_d:.2f} threshold {self.params['max_stuck_time']:.2f}")
            return True
        return False

def get_param():
    params = {'run_id': rospy.get_param('/run_id'),
              'map_topic': rospy.get_param('~map_topic', '/map'),
              'robot_frame': rospy.get_param('~robot_frame', 'base_footprint'),
              'real_pose_topic': rospy.get_param('~robot_real_pose_topic'),
              'anemometer_topic': rospy.get_param('~anemometer_topic'),
              'anemometer_speed_th': float(rospy.get_param('~anemometer_speed_th', 0.2)),
              'gas_sensor_topic': rospy.get_param('~gas_sensor_topic'),
              'gas_sensor_hit_th': float(rospy.get_param('~gas_sensor_hit_th', 61500)),
              'sensor_window': int(rospy.get_param('~sensor_window', 4)),
              'source_x': float(rospy.get_param('~source_x')),
              'source_y': float(rospy.get_param('~source_y')),
              'find_source_th': float(rospy.get_param('~find_source_th', 0.5)),
              'iter_rate': int(rospy.get_param('~iter_rate', 1)),
              'max_iter': int(rospy.get_param('~max_iter', 360)),
              'max_stuck_time': float(rospy.get_param('~max_sutck_time', 60.0)),
              'data_path': rospy.get_param('~data_path', '~'),
              'visual': bool(rospy.get_param('~visual', 'true')),
              'rrt_max_iter': int(rospy.get_param('~rrt_max_iter')),
              'rrt_max_r': float(rospy.get_param('~rrt_max_r')),
              'rrt_min_r': float(rospy.get_param('~rrt_min_r')),
              'reach_waypoint_dis_th': float(rospy.get_param('~reach_waypoint_dis_th')),
              'sigma': float(rospy.get_param('~sigma')),
              'beta': float(rospy.get_param('~beta')),
              'obs_r': float(rospy.get_param('~obs_r')),
              }
    params['data_path'] = params['data_path'] + '/' + params['run_id']
    return params


def main():
    rospy.init_node('main', anonymous=True)
    params = get_param()
    
    mc = MapClient(params['map_topic'])
    rc = RobotClient(mc.map_frame, params['robot_frame'], params['real_pose_topic'], 
                     params['anemometer_topic'], params['gas_sensor_topic'], 
                     params['sensor_window'])
    fc = FinishCheck(rc, params)
    
    agent = Banana(mc, rc, params)
    
    iter_rate = rospy.Rate(params['iter_rate'])
    it = 0
    # input()
    # rospy.sleep(2)
    while not rospy.is_shutdown():
        rospy.loginfo(f'ITER {it} TIME {rospy.get_time():.2f}')
        agent.observe(it)
        agent.estimate()
        agent.evaluate()
        agent.navigate()
        
        if params['visual']:
            agent.visualize()
        agent.record_data()
        
        if fc.check(it):
            if not os.path.exists(params['data_path']):
                os.makedirs(params['data_path'])
            agent.save_data(params['data_path'])
            
            cp_ros_log_cmd = 'cp -r ~/.ros/log/' + params['run_id'] + '/main* ' + params['data_path'] + '/'
            rospy.loginfo('%s', cp_ros_log_cmd)
            os.system(cp_ros_log_cmd)
            
            zip_cmd = 'cd ' + params['data_path'] + '/.. && zip -rm ' + params['data_path'].split('/')[-1] + '.zip ' + params['data_path'].split('/')[-1] + ' > /dev/null 2>&1'
            rospy.loginfo('%s', zip_cmd)
            os.system(zip_cmd)
            
            rc.cancel_move()
            rospy.sleep(2)
            rospy.signal_shutdown("Shutdown")
        
        it = it + 1
        iter_rate.sleep()

main()