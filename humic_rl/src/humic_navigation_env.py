#!/usr/bin/env python3
# -------------------------------
# Humic Navigation Task Environment
# Author:  Kim Young Gi(HCIR Lab.)
# -------------------------------

""" ROS """
import rospy
import rospkg
from std_msgs.msg import Float64
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist, Point, Pose
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ContactsState
from sensor_msgs.msg import Range
import message_filters

from src.respawnGoal import Respawn

""" python Library """
import numpy as np
import math
import time

# https://github.com/ROBOTIS-GIT/turtlebot3_machine_learning/blob/master/turtlebot3_dqn/src/turtlebot3_dqn/environment_stage_1.py
# https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_plugins/test/test_worlds/gazebo_ros_range.world#L82

class HumicNavigationEnv(object):
    def __init__(self, mapName):
        self.mapName = mapName
        """ Action """
        self.action_ = [.0, .0, .0]
        
        self.action_shape = np.shape(self.action_) # linear x, angular z
        self.action_space = {'low':np.ones(self.action_shape)*([-1, -1, -1]),
                            'high':np.ones(self.action_shape)*([1, 1, 1]),
                            'shape':self.action_shape}
        
        self.cmd_vel_pub = rospy.Publisher('/humic/cmd_vel', Twist, queue_size=1)

        """ Observation """
        self.observation_dim = 4 + 1 # 4 ultrasonic sensors + distance
        self.action_dim = len(self.action_)

        self.position = Pose()
        self.odom_sub = rospy.Subscriber('/humic/odom', Odometry, self.getOdometry)

        """ Goal Box"""
        self.goal_x = 0
        self.goal_y = 0
        
        self.respawn_goal = Respawn(mapName=self.mapName) # room, building
        
        self.goal_distance = 0.5
        self.pre_distance = 0.

        """ Reward """
        self.collision_min_distance = 0.25 #[m]
        self.goal_reward = 100
        self.collision_reward = -100

        self.goal_ctr = 0
        self.get_goal = False
        self.init_goal = True

        """ Gazebo """
        self.reset_world_proxy = rospy.ServiceProxy('/gazebo/reset_world', Empty)

    def getOdometry(self, odom):
        self.position = odom.pose.pose.position

    def action_sample(self):
        action = np.random.uniform(low=self.action_space['low'], high=self.action_space['high'], size=self.action_shape)
        return action
    
    def step(self, action):
        """
        Implement the environment step.
        Execute action and returns:
            - action
            - observation
            - reward
            - done
        """
        action = np.clip(action, self.action_space['low'], self.action_space['high'])
        
        msg = Twist()
        msg.linear.x = action[0] # linear x
        msg.linear.y = action[1] # linear y
        msg.angular.z = action[2] # angular z
        self.cmd_vel_pub.publish(msg)
        
        obs, collision = self.observation()

        distance = obs[-1]
        
        reward = self.pre_distance - distance
        self.pre_distance = distance
        
        done = False
        if self.get_goal:
            done = True
            reward = self.goal_reward
            self.cmd_vel_pub.publish(Twist())
            self.goal_x, self.goal_y = self.respawn_goal.getPosition(True, delete=True)
            self.pre_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y), 2)
            self.get_goal = False
        elif collision:
            done = True
            reward = self.collision_reward
            self.cmd_vel_pub.publish(Twist())

        info = {}
        return obs, reward, done, info
    
    def observation(self):
        """
        Return: state
        State:
        """
        self.ultrasonic_front_data = None
        self.ultrasonic_left_data = None
        self.ultrasonic_right_data = None
        self.ultrasonic_rear_data = None

        while self.ultrasonic_front_data is None:
            try:
                self.ultrasonic_front_data = rospy.wait_for_message('/humic/front_ultrasonic', Range, timeout=1)
                self.ultrasonic_front_data = self.ultrasonic_front_data.range
            except:
                self.ultrasonic_front_data = 0
        
        while self.ultrasonic_left_data is None:
            try:
                self.ultrasonic_left_data = rospy.wait_for_message('/humic/left_ultrasonic', Range, timeout=1)
                self.ultrasonic_left_data = self.ultrasonic_left_data.range
            except:
                self.ultrasonic_left_data = 0
          
        while self.ultrasonic_right_data is None:
            try:
                self.ultrasonic_right_data = rospy.wait_for_message('/humic/right_ultrasonic', Range, timeout=1)
                self.ultrasonic_right_data = self.ultrasonic_right_data.range
            except:
                self.ultrasonic_right_data = 0
        
        while self.ultrasonic_rear_data is None:
            try:
                self.ultrasonic_rear_data = rospy.wait_for_message('/humic/rear_ultrasonic', Range, timeout=1)
                self.ultrasonic_rear_data = self.ultrasonic_rear_data.range
            except:
                self.ultrasonic_rear_data = 0

        ultrasonic_datas = [self.ultrasonic_front_data, self.ultrasonic_left_data,
                            self.ultrasonic_right_data, self.ultrasonic_rear_data ]

        collision = False
        if 0 < min(ultrasonic_datas) < self.collision_min_distance:
            collision = True
        
        distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y), 2)
        
        if distance < self.goal_distance:
            self.get_goal = True

        return ultrasonic_datas + [distance], collision

    def reset(self):
        """
        Reset Gazebo Simulator
        """ 
        rospy.wait_for_service('/gazebo/reset_world')
        try:
            self.reset_world_proxy()
        except rospy.ServiceException as e:
            rospy.loginfo("/gazebo/reset_world service call failed")

        self.get_goal = False

        if self.init_goal:
            self.goal_x, self.goal_y = self.respawn_goal.getPosition()
            self.init_goal = False

        obs, _ = self.observation()

        self.pre_distance = obs[-1]

        return obs
    
    
