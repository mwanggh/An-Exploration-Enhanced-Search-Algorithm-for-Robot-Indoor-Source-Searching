#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlan, GetPlanRequest, GetPlanResponse
import tf
import copy
from olfaction_msgs.msg import anemometer as Anemoter
from olfaction_msgs.msg import gas_sensor as GasSensor
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import math


class RobotClient():
    def __init__(self, map_frame, robot_base_frame, robot_real_pose_topic, anemometer_topic, gas_sensor_topic, sensor_window=5):
        self.map_frame = map_frame
        self.robot_frame = robot_base_frame
        self.robot_real_pose_topic = robot_real_pose_topic
        self.anemometer_topic = anemometer_topic
        self.gas_sensor_topic = gas_sensor_topic
        self.sensor_window = sensor_window
        self.pose = None
        self.x = None
        self.y = None
        self.z = None
        self.yaw = None
        self.raw_real_pose = None
        self.real_pose = None
        self.real_x = None
        self.real_y = None
        self.real_z = None
        self.real_yaw = None
        self.raw_anemoter = []
        self.wind_direction = None
        self.wind_speed = None
        self.raw_gas = []
        self.gas = None
        self.move_base_act_client = None
        self.move_base_srv_client = None
        self.move_base_done = True
        self.tf_listener = tf.TransformListener()
        
        # init tf
        while not rospy.is_shutdown():
            try:
                rospy.loginfo(f'Waiting for transform from {str(self.map_frame)} to {str(self.robot_frame)}')
                self.tf_listener.waitForTransform(self.map_frame, self.robot_frame, rospy.get_rostime(), rospy.Duration(1.0))
            except:
                rospy.sleep(0.1)
                continue
            break
        
        # init robot_real_pose subscriber
        self.real_pose_sub = rospy.Subscriber(self.robot_real_pose_topic, Odometry, self.real_pose_cb, queue_size=50)
        while self.raw_real_pose is None and not rospy.is_shutdown():
            rospy.loginfo(f'Waiting for {str(self.real_pose_sub.resolved_name)}')
            rospy.sleep(0.1)
            
        # init anemoter sub
        self.anemoter_sub = rospy.Subscriber(self.anemometer_topic, Anemoter, self.anemoter_cb, queue_size=10)
        while self.raw_anemoter is None and not rospy.is_shutdown():
            rospy.loginfo(f'Waiting for {str(self.anemometer_topic.resolved_name)}')
            rospy.sleep(0.1)
        
        # init gas_sensor sub
        self.gas_sensor_sub = rospy.Subscriber(self.gas_sensor_topic, GasSensor, self.gas_sensor_cb, queue_size=2)
        while self.raw_gas is None and not rospy.is_shutdown():
            rospy.loginfo(f'Waiting for {str(self.gas_sensor_sub.resolved_name)}')
            rospy.sleep(0.1)
        
        # init move base action client
        self.move_base_act_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo(f'Waiting for action server')
        self.move_base_act_client.wait_for_server()
        
        # init move base service client
        self.move_base_srv_client = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
        rospy.loginfo(f'Waiting for plan check service')
        self.move_base_srv_client.wait_for_service()
        
        rospy.loginfo(f'Robot client initialized')
        
    
    def update_pose(self):
        try:
            p1 = PoseStamped()
            p1.header.frame_id = self.robot_frame
            p1.pose.orientation.w = 1.0
            self.pose = self.tf_listener.transformPose(self.map_frame, p1)
            self.x = self.pose.pose.position.x
            self.y = self.pose.pose.position.y
            self.z = self.pose.pose.position.z
            quaternion = (self.pose.pose.orientation.x, self.pose.pose.orientation.y, 
                          self.pose.pose.orientation.z, self.pose.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            self.yaw = euler[2]
            rospy.loginfo(f'Robot pose: x {self.x:.2f} y {self.y:.2f} yaw {self.yaw:.2f}')
        except:
            return self.pose
        return self.pose
    
    def update_real_pose(self):
        while self.raw_real_pose is None:
            rospy.loginfo(f'Waiting for robot real pose')
            rospy.sleep(0.1)
        self.real_pose = copy.deepcopy(self.raw_real_pose)
        self.real_x = self.real_pose.pose.pose.position.x
        self.real_y = self.real_pose.pose.pose.position.y
        self.real_z = self.real_pose.pose.pose.position.z
        quaternion = (self.real_pose.pose.pose.orientation.x, self.real_pose.pose.pose.orientation.y,
                      self.real_pose.pose.pose.orientation.z, self.real_pose.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.real_yaw = euler[2]
        rospy.loginfo(f'Robot real pose: x {self.real_x:.2f} y {self.real_y:2f} yaw {self.real_yaw:.2f}')
        return self.real_pose
        
    def real_pose_cb(self, pose:Odometry):
        self.raw_real_pose = pose
        
    def update_anemoter(self):
        while len(self.raw_anemoter) < 1:
            rospy.loginfo(f'Waiting for anemoter data')
            rospy.sleep(0.1)
        anemoter = copy.deepcopy(self.raw_anemoter)
        wd_x = [math.cos(item.wind_direction) for item in anemoter]
        wd_y = [math.sin(item.wind_direction) for item in anemoter]
        self.wind_direction = math.atan2(sum(wd_y), sum(wd_x))
        self.wind_speed = sum([item.wind_speed for item in anemoter]) / len(anemoter)
        rospy.loginfo(f'Anemoter data: direction {self.wind_direction:.2f} speed {self.wind_speed:.2f}')
    
    def anemoter_cb(self, anemoter_data:Anemoter):
        self.raw_anemoter.append(anemoter_data)
        while len(self.raw_anemoter) > self.sensor_window:
            self.raw_anemoter.pop(0)
    
    def update_gas(self):
        while len(self.raw_gas) < 1:
            rospy.loginfo(f'Waiting for gas sensor data')
            rospy.sleep(0.1)
        gas_data = copy.deepcopy(self.raw_gas)
        self.gas = sum(gas_data) / len(gas_data)
        rospy.loginfo(f'Gas sensor data: {self.gas:.2f}')
    
    def gas_sensor_cb(self, gas_data:GasSensor):
        self.raw_gas.append(gas_data.raw)
        # print('raw gas', self.raw_gas)
        while len(self.raw_gas) > self.sensor_window:
            self.raw_gas.pop(0)
    
    def check_plan(self, x, y, yaw=None):
        rospy.loginfo(f'Check plan: x {x:.2f} y {y:.2f}')
        start = PoseStamped()
        start.header.frame_id = self.map_frame
        start.header.stamp = rospy.Time.now()
        start.pose.position.x = self.x
        start.pose.position.y = self.y
        goal = PoseStamped()
        goal.header.frame_id = self.map_frame
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = x
        goal.pose.position.y = y
        req = GetPlanRequest()
        req.start = start
        req.goal = goal
        req.tolerance = 0.15
        ret:GetPlanResponse = self.move_base_srv_client.call(req)
        return len(ret.plan.poses) > 0
    
    def send_goal(self, x, y, yaw=None):
        rospy.loginfo(f'Send goal: x {x:.2f} y {y:.2f}')
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.map_frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        if yaw is None:
            goal.target_pose.pose.orientation.w = 1.0
        else:
            quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
            goal.target_pose.pose.orientation.x = quaternion[0]
            goal.target_pose.pose.orientation.y = quaternion[1]
            goal.target_pose.pose.orientation.z = quaternion[2]
            goal.target_pose.pose.orientation.w = quaternion[3]
        self.move_base_act_client.send_goal(goal, done_cb=self.move_base_done_cb )
        self.move_base_done = False
        
    def move_base_done_cb(self, status, result):
        self.move_base_done = True
        # print(status)
        rospy.loginfo(f'Move_base done {self.move_base_done}')
    
    def cancel_move(self):
        self.move_base_act_client.cancel_all_goals()
        rospy.loginfo(f'Cancel all goals')
        while not self.move_base_done:
            # rospy.loginfo(f'Waiting for cancel all goals')
            rospy.sleep(0.01)