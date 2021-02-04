#!/usr/bin/env python3
# -------------------------------
# Humic Push Task Environment
# Author:  Kim Young Gi(HCIR Lab.)
# Date: 2021. 02. 02
# -------------------------------

""" ROS """
import rospy
import rospkg
from std_msgs.msg import Float64
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from gazebo_msgs.msg import ContactsState
import message_filters

""" python Library """
import time
import numpy as np
import gym
from gym import spaces

np.random.seed(14512281)

""" YCB Object Model """
from src.ycb_object import YCBObject
"""
YCB_Object_List:
[
    '003_cracker_box', 
    '004_sugar_box', 
    '005_tomato_soup_can', 
    '006_mustard_bottle', 
    '009_gelatin_box', 
    '010_potted_meat_can', 
    '021_bleach_cleanser'
]
"""
class MSG_INVALID_JOINT_NAMES_DIFFER(Exception):
    """Error object exclusively raised by _process_observations."""
    pass

class HumicPushEnv(object):
    def __init__(self):
        """ Arm Control """
        self.arm_joints = ['r_shoulder_joint','r_upperArm_joint','r_elbow_joint','r_foreArm_joint','r_lowerArm_joint','r_wrist_joint']

        self.joint_num = len(self.arm_joints)
        
        self.init_angles = np.array([.0, .0, .0, .0, .0, .0])
        
        self.action_clip = {'low':np.array([-1.57, -1.57, .0, -1.57, -1.57, -1.57]), #-1.57, -1.57, 0.0, -1.57, -1.57, -1.57
                            'high':np.array([1.57, .0, 0.52, 1.57, 1.57, 1.57])} # 1.57, 0, 0.52, 1.57, 1.57, 1.57
        
        self.arm_pub = rospy.Publisher('/humic/right_arm/command', JointTrajectory, queue_size=1)
        self.arm_sub = rospy.Subscriber('/humic/right_arm/state', JointTrajectoryControllerState, self.jointStateCallback)
        self.jointState_msg = None

        """ Finger Control"""
        self.right_finger1_pub = rospy.Publisher('/humic/right_finger1/command', Float64, queue_size=1)
        self.right_finger2_pub = rospy.Publisher('/humic/right_finger2/command', Float64, queue_size=1)
        self.right_finger3_pub = rospy.Publisher('/humic/right_finger3/command', Float64, queue_size=1)
        self.right_finger4_pub = rospy.Publisher('/humic/right_finger4/command', Float64, queue_size=1)
        self.right_finger5_pub = rospy.Publisher('/humic/right_finger5/command', Float64, queue_size=1)
        self.right_finger6_pub = rospy.Publisher('/humic/right_finger6/command', Float64, queue_size=1)
        self.right_finger7_pub = rospy.Publisher('/humic/right_finger7/command', Float64, queue_size=1)
        self.right_finger8_pub = rospy.Publisher('/humic/right_finger8/command', Float64, queue_size=1)
        self.right_thumb_pub = rospy.Publisher('/humic/right_thumb/command', Float64, queue_size=1)

        """ YCB Object """
        self.object_list = ['003_cracker_box', '004_sugar_box', '005_tomato_soup_can', '009_gelatin_box']
        self.cracker = YCBObject(model=self.object_list[0])
        self.sugar = YCBObject(model=self.object_list[1])
        # self.soup = YCBObject(model=self.object_list[2])
        # self.gelatin = YCBObject(model=self.object_list[3])
        
        self.init_obj = True
        self.obs_check = 0
        
        self.cracker_sub = message_filters.Subscriber('/dope/pose_cracker', PoseStamped)
        self.sugar_sub = message_filters.Subscriber('/dope/pose_sugar', PoseStamped)

        """ Observation """
        self.obs_msg = message_filters.TimeSynchronizer([self.cracker_sub, self.sugar_sub], queue_size=1)
        self.obs_msg.registerCallback(self.observationCallback)
        
        self.action_space = spaces.Box(self.action_clip['low'], self.action_clip['high'])
        self.observation_dim = self.joint_num + (6*2) + 1 # joint angles, object pose * num of objects, distance
        high = np.inf*np.ones(self.observation_dim)
        low = -high
        self.observation_space = spaces.Box(low, high)
        self.action_bound = 1.57

        """ Reward """
        self.min_dist = 0.17 #[m]
        # self.max_dist = 0.40 #[m]

        self.reward_goal = 100
        self.reward_collision = -10

        self.get_target = 0

        """ Gazebo """
        self.reset_simulation_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.reset_world_proxy = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)

        """ Collision """
        self.collision_sub = rospy.Subscriber('/humic/gazebo_contact', ContactsState, self.collisionCallback)
        self.collision_check = False
        self.collision_msg = None

    """ ROS Callback """
    def jointStateCallback(self, message):
        """
        Callback method for the subscriber of JointTrajectoryControllerState
        """
        self.jointState_msg = message

    def observationCallback(self, crackerPose, sugarPose):
        self.cracker_pose = np.array([crackerPose.pose.position.x, crackerPose.pose.position.y, crackerPose.pose.position.z,
                                crackerPose.pose.orientation.x, crackerPose.pose.orientation.y, crackerPose.pose.orientation.z])
        self.sugar_pose = np.array([sugarPose.pose.position.x, sugarPose.pose.position.y, sugarPose.pose.position.z,
                            sugarPose.pose.orientation.x, sugarPose.pose.orientation.y, sugarPose.pose.orientation.z])
        self.obs_check = 0

    def collisionCallback(self, message):
        """
        Callback method for the subscriber of ContactsState
        """
        self.collision_msg = message
        if self.collision_msg.states:
            if 'desk' in self.collision_msg.states[0].collision2_name\
                or 'torso_link' in self.collision_msg.states[0].collision2_name\
                or 'head_kinect_link' in self.collision_msg.states[0].collision2_name\
                or 'shoulder_link' in self.collision_msg.states[0].collision2_name:
                self.collision_check = True
            else:
                pass
                # rospy.loginfo('Collsion: {0} and {1}'.format(self.collision_msg.states[0].collision1_name, self.collision_msg.states[0].collision2_name))
        self.collision_msg = None
       
    """ ROS msg Check """
    def checkJointStateMsg(self, msg, joint_names):
        if not msg:
            rospy.loginfo("Message is empty")
        else:
            if msg.joint_names != joint_names:
                if len(msg.joint_names) != len(joint_names):
                    raise MSG_INVALID_JOINT_NAMES_DIFFER
        
        return np.array(msg.actual.positions)

    """ Utility """
    def setObjectPose(self, position_check=False, delete=False):
        self.cracker.setPose(position_check=position_check, delete=delete)
        self.sugar.setPose(position_check=position_check, delete=delete)
        # self.soup.setPose(position_check=position_check, delete=delete)
        # self.gelatin.setPose(position_check=position_check, delete=delete)

    def computeDistance(self, object1, object2):
        dist = np.round((np.linalg.norm((object1-object2),ord=2)),5)
        if dist < 0.1:
            dist = 0.1
        return dist

    def computeSparseReward(self):
        return 1

    def computeDenseReward(self, dist):
        return 2**((self.min_dist-dist)/self.min_dist)

    def getTrajectoryMsg(self, joint_names, angles):
        """
        Arm trajectory controller
        """
        arm_msg = JointTrajectory()
        arm_msg.joint_names = joint_names
        arm_msg.header.stamp = rospy.Time.now()

        arm_target = JointTrajectoryPoint()
        arm_target.positions = angles
        arm_target.time_from_start.secs = 1

        arm_msg.points = [arm_target]
        return arm_msg
    
    def initArmPose(self):
        time.sleep(0.3)
        arm_init = self.getTrajectoryMsg(self.arm_joints, self.init_angles)
        self.arm_pub.publish(arm_init)

        self.right_finger1_pub.publish(0)
        self.right_finger2_pub.publish(0)
        self.right_finger3_pub.publish(0)
        self.right_finger4_pub.publish(0)
        self.right_finger5_pub.publish(0)
        self.right_finger6_pub.publish(0)
        self.right_finger7_pub.publish(0)
        self.right_finger8_pub.publish(0)
        self.right_thumb_pub.publish(0)
    
    """ Gazebo Simulator """
    def unpauseSim(self):
        """
        Resume physics updates.
        """
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause_proxy()
            rospy.loginfo("unpause_simulation")
        except (rospy.ServiceException) as e:
            rospy.loginfo("/gazebo/unpause_physics service call failed")

    def pauseSim(self):
        """
        Pause physics updates.
        """
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause_proxy()
            rospy.loginfo("pause_simulation")
        except (rospy.ServiceException) as e:
            rospy.loginfo("/gazebo/pause_physics service call failed")

    def resetSimulation(self):
        """
        Resets the entire simulation including the time
        """
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            self.reset_simulation_proxy()
            rospy.loginfo("reset_simulation")
        except rospy.ServiceException as e:
            rospy.loginfo("/gazebo/reset_simulation service call failed")
    
    def resetWorld(self):
        """
        Resets the model's poses
        """
        rospy.wait_for_service('/gazebo/reset_world')
        try:
            self.reset_world_proxy()
            rospy.loginfo("reset_world")
        except rospy.ServiceException as e:
            rospy.loginfo("/gazebo/reset_world service call failed")

    # def close(self, seed=None):
    #     rospy.logdebug("Closing Humic Push environment")
    #     rospy.signal_shutdown("Closing Humic Push environment")
    
    """ Environment """
    def step(self, action):
        """
        Implement the environment step.
        Execute action and returns:
            - action
            - observation
            - reward
            - done
        """
        action = np.clip(action, self.action_clip['low'], self.action_clip['high'])

        self.arm_pub.publish(self.getTrajectoryMsg(self.arm_joints, action))
        time.sleep(.2)

        obs, collision = self.observation() # [joint_angels, cracker_pose, sugar_pose, distance]
        self.obs_check +=1

        done = False

        reward = self.computeDenseReward(dist=obs[-1])
        # reward = self.computeSparseReward()

        if collision:
            rospy.loginfo("Collision!!")
            reward = self.reward_collision
            done = True
        elif self.obs_check > 5:
            rospy.loginfo("Can't find Object")
            self.obs_check = 0
            reward = -1
            done = True
        elif obs[-1] <= self.min_dist:
            rospy.loginfo("Goal!!")
            reward = self.reward_goal
            self.setObjectPose(position_check=True, delete=True)
            self.get_target+=1
            done = True

        return obs, reward, done, {}
    
    def observation(self):
        """
        Return: state
        State: joint angles, objects pose
        """
        jointState_msg = self.jointState_msg

        if jointState_msg is None:
            return None

        last_jointStates = self.checkJointStateMsg(jointState_msg, self.arm_joints)

        dist = self.computeDistance(self.cracker_pose[:3], self.sugar_pose[:3])
        
        obs = np.r_[np.reshape(last_jointStates, -1), np.reshape(self.cracker_pose, -1), np.reshape(self.sugar_pose, -1), np.reshape(dist, -1)]

        collision = self.collision_check

        self.collision_check = False

        return obs, collision

    def reset(self):
        """
        Reset Gazebo Simulator
        """   
        self.initArmPose()
        self.pauseSim()
        self.resetWorld()
        self.unpauseSim()
        # self.resetSimulation()

        self.collision_check = False
        self.collision_msg = None

        if self.init_obj:
            self.setObjectPose()
            self.init_obj = False

        obs, _ = self.observation()
        self.init_dist = obs[-1]
        
        return obs