#!/usr/bin/env python3
# -------------------------------
# Humic Push Task Environment
# Author:  Kim Young Gi(HCIR Lab.)
# Date: 2021. 1. 4
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

class HumicPushEnv(gym.Env):
    def __init__(self, max_step):
        self.max_step = max_step
        self.step_ctr = 0

        """ Arm Control """
        self.arm_joints = ['r_shoulder_joint','r_upperArm_joint','r_elbow_joint','r_foreArm_joint','r_lowerArm_joint','r_wrist_joint']

        self.joint_num = len(self.arm_joints)
        
        self.init_angles = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        
        self.action_clip = {'low':np.array([-1.57, -1.57, -1.57, -1.57, -1.57, -1.57]), #-1.57, -1.57, 0.0, -1.57, -1.57, -1.57
                            'high':np.array([1.57, 1.57, 1.57, 1.57, 1.57, 1.57])} # 1.57, 0, 0.52, 1.57, 1.57, 1.57
        
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
        self.object_list = ["cracker", "sugar"]
        self.cracker = YCBObject(model='003_cracker_box')
        self.sugar = YCBObject(model='004_sugar_box')
        # self.soup = YCBObject(model='005_tomato_soup_can')
        # self.gelatin = YCBObject(model='009_gelatin_box')
        
        self.object_pose = 6
        self.initObjectPose()
        
        self.cracker_sub = rospy.Subscriber('/dope/pose_cracker', PoseStamped, self.crackerCallback)
        self.sugar_sub = rospy.Subscriber('/dope/pose_sugar', PoseStamped, self.sugarCallback)
        # self.soup_sub = rospy.Subscriber('/dope/pose_soup', PoseStamped, self.soupCallback)
        # self.gelatin_sub = rospy.Subscriber('/dope/pose_gelatin', PoseStamped, self.gelatinCallback)

        """ Observation """
        self.action_space = spaces.Box(self.action_clip['low'], self.action_clip['high'])
        self.observation_dim = self.joint_num + (self.object_pose*2)
        high = np.inf*np.ones(self.observation_dim)
        low = -high
        self.observation_space = spaces.Box(low, high)
        self.action_bound = 1.57

        """ Reward """
        self.min_dist = 0.15 #[m]
        self.max_dist = 0.30 #[m]
        self.pre_dist = 0

        self.reward_goal = 10
        self.reward_collision = -10

        self.get_target = 0

        """ Gazebo """
        self.reset_simulation_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.reset_world_proxy = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)

        """ Collision """
        self.collision_sub = rospy.Subscriber('/humic/gazebo_contact', ContactsState, self.collisionCallback)
        self.not_collision_list = ["finger", "thumb"]
        self.collision_msg = None
        self.collision_check = False

    """ ROS Callback """
    def crackerCallback(self, crackerPose):
        self.cracker_pose = np.array([crackerPose.pose.position.x, crackerPose.pose.position.y, crackerPose.pose.position.z,
                                crackerPose.pose.orientation.x, crackerPose.pose.orientation.y, crackerPose.pose.orientation.z])
        # rospy.loginfo("cracker Pose:{0}, Position:{1}".format(self.cracker_pose, self.cracker_pose[:3]))
    
    def sugarCallback(self, sugarPose):
        self.sugar_pose = np.array([sugarPose.pose.position.x, sugarPose.pose.position.y, sugarPose.pose.position.z,
                            sugarPose.pose.orientation.x, sugarPose.pose.orientation.y, sugarPose.pose.orientation.z])
        # rospy.loginfo("sugar Pose: {0}, Position:{1}".format(self.sugar_pose, self.sugar_pose[:3]))
    
    # def soupCallback(self, soupPose):
    #     self.soup_pose = np.array([soupPose.pose.position.x, soupPose.pose.position.y, soupPose.pose.position.z,
    #                         soupPose.pose.orientation.x, soupPose.pose.orientation.y, soupPose.pose.orientation.z])
    #     # rospy.loginfo("sugar Pose: {0}, Position:{1}".format(self.soup_pose, self.soup_pose[:3]))
    
    # def gelatinCallback(self, gelatinPose):
    #     self.gelatin_pose = np.array([gelatinPose.pose.position.x, gelatinPose.pose.position.y, gelatinPose.pose.position.z,
    #                         gelatinPose.pose.orientation.x, gelatinPose.pose.orientation.y, gelatinPose.pose.orientation.z])
    #     # rospy.loginfo("sugar Pose: {0}, Position:{1}".format(self.gelatin_pose, self.gelatin_pose[:3]))
    
    def jointStateCallback(self, message):
        """
        Callback method for the subscriber of JointTrajectoryControllerState
        """
        self.jointState_msg = message
    
    def collisionCallback(self, message):
        """
        Callback method for the subscriber of ContactsState
        """
        self.collision_msg = message

        if self.collision_msg.states:
            if self.object_list[0] not in self.collision_msg.states[0].collision2_name and\
                self.object_list[1] not in self.collision_msg.states[0].collision2_name and\
                'r_' not in self.collision_msg.states[0].collision2_name:
                # rospy.loginfo('Collsion: {0} and {1}'.format(self.collision_msg.states[0].collision1_name, self.collision_msg.states[0].collision2_name))
                self.collision_check = True
            else:
                pass
        
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
    def setObjectPosition(self, init_position=True, change_position=False):
        if init_position:
            self.cracker.setPosition()
            self.sugar.setPosition()
            # self.soup.setPosition()
            # self.gelatin.setPosition()

        if change_position:
            self.cracker.setPosition(position_check=True, delete=True)
            self.sugar.setPosition(position_check=True, delete=True)
            # self.soup.setPosition(position_check=True, delete=True)
            # self.gelatin.setPosition(position_check=True, delete=True)

    def computeDistance(self, object1, object2):
        return np.linalg.norm((object1-object2),ord=2)

    def computeReward(self, dist, pre_dist):
        d_ = pre_dist - dist
        if dist <= self.min_dist and dist > 0.03:
            return self.reward_goal
        elif dist > self.min_dist and dist <= self.max_dist: # -np.log -> 1.2 ~ 1.8
            return -np.log(dist)*d_
        else:
            rospy.loginfo("Can not find Object")
            return -1
        # else:
        #     return -1
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
    
    def initObjectPose(self):
        self.cracker_pose = np.zeros(self.object_pose)
        self.sugar_pose = np.zeros(self.object_pose)
        # self.soup_pose = np.zeros(self.object_pose)
        # self.gelatin_pose = np.zeros(self.object_pose)
    
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
        self.step_ctr+=1

        self.arm_pub.publish(self.getTrajectoryMsg(self.arm_joints, action))
        time.sleep(1.)

        obs = self.observation() # [joint_angels, cracker_pose, sugar_pose]
        collision = self.collision_check
        done = False
        
        if collision:
            reward = self.reward_collision
            done = True
            rospy.loginfo("Collision!!")
        else:
            """" Calculate distance between objectA and objectB positions[x,y,z] """
            dist = self.computeDistance(obs[self.joint_num:self.object_pose+3],
                                        obs[self.object_pose+self.object_pose:self.object_pose+self.object_pose+3])\
            
            #rospy.loginfo("Distance: {}".format(dist)
            reward = self.computeReward(dist=dist,pre_dist=self.pre_dist)
            self.pre_dist = dist
            if reward==self.reward_goal:
                done = True
                rospy.loginfo("Goal!! Two Objects Closed!!")
                self.get_target+=1
            elif self.step_ctr >= self.max_step:
                done = True
                rospy.loginfo("Max Step")

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

        obs = np.r_[np.reshape(last_jointStates, -1), np.reshape(self.cracker_pose, -1), np.reshape(self.sugar_pose, -1)]

        self.initObjectPose()

        # state = np.r_[np.reshape(last_jointStates, -1), np.reshape(self.cracker_pose, -1), np.reshape(self.sugar_pose, -1),\
        #                 np.reshape(self.soup_pose, -1), np.reshape(self.gelatin_pose, -1)]

        return obs

    def reset(self, init_position=True, change_position=False):
        """
        Reset Gazebo Simulator
        """   
        self.initArmPose()
        self.pauseSim()
        self.resetWorld()
        self.unpauseSim()
        # self.resetSimulation()
        
        self.setObjectPosition(init_position=init_position, change_position=change_position)

        obs = self.observation()
        self.pre_dist = self.computeDistance(obs[self.joint_num:self.object_pose+3],
                                        obs[self.object_pose+self.object_pose:self.object_pose+self.object_pose+3])

        self.collision_check = False
        self.collision_msg = None
        self.step_ctr = 0

        return obs

    def seed(self):
        pass