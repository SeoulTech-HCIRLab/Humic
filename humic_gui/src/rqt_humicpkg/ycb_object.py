# -------------------------------
# Humic YCB Object Controller
# Author: Kim Young Gi(HCIR Lab.)
# Refer to ROBOTIS Turtlebot3 Machine Learning package
# -------------------------------

""" ROS """
import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose

""" python Library """
import random
import time
import os
import numpy as np

"""
YCB Object List: 7 objects(number: 0~7)
['003_cracker_box', '004_sugar_box', '005_tomato_soup_can', '006_mustard_bottle', '009_gelatin_box', '010_potted_meat_can', '021_bleach_cleanser']
"""

class YCBObject(object):
    def __init__(self, model):
        
        self.modelName = model
        self.modelPath = os.path.dirname(os.path.realpath(__file__))
        self.modelPath = self.modelPath.replace('humic_gui/src/rqt_humicpkg', 'humic_gazebo/models/{0}/{1}.sdf'.format(self.modelName, self.modelName[4:]))
       
        self.f = open(self.modelPath, 'r')
        self.model = self.f.read()

        self.object_position = Pose()

        self.init_pose = {
                            '003_cracker_box':[0.713329, 0.215572, 0.733239, -0.000787, -0.004160, 0.693423], # translation[x,y,z], rotation[x,y,z]
                            '004_sugar_box':[0.788297, -0.055475, 0.729798, -0.012855, -0.006456, -0.260465],
                            # '005_tomato_soup_can':[],
                            '006_mustard_bottle':[0.710370, -0.151026, 0.733105, -0.001192, -0.000387, -0.002376], 
                            '009_gelatin_box':[0.592424, -0.061840, 0.730432, -0.002686, -0.003865, 2.873844], 
                            '010_potted_meat_can':[0.645572, 0.137873, 0.7331033, -0.003180, -0.002622, 0.046759], 
                            '021_bleach_cleanser':[0.817132, 0.043675, 0.730438, -0.001831, -0.001163, 0.035973]
                        }
        self.init_object_pose = self.init_pose[self.modelName]

        self.object_position.position.x = self.init_object_pose[0]
        self.object_position.position.y = self.init_object_pose[1]
        self.object_position.position.z = self.init_object_pose[2]
        self.object_position.orientation.x = self.init_object_pose[3]
        self.object_position.orientation.y = self.init_object_pose[4]
        self.object_position.orientation.z = self.init_object_pose[5]
        self.object_position.orientation.w = 1
                
        self.object_x_list = [self.init_object_pose[0], self.init_object_pose[0], self.init_object_pose[0]+0.03, self.init_object_pose[0]+0.03]
        self.object_y_list = [self.init_object_pose[1], self.init_object_pose[1]-0.01, self.init_object_pose[1], self.init_object_pose[1]-0.01]
        self.object_z_list = np.ones(4)*self.init_object_pose[2]

        self.last_object_x = self.init_object_pose[0]
        self.last_object_y = self.init_object_pose[1]
        self.last_object_z = self.init_object_pose[2]
        self.last_index = 0

        self.sub_model = rospy.Subscriber('gazebo/model_states', ModelStates, self.checkModel)
        self.check_model = False
        self.index = 0

    def checkModel(self, model):
        self.check_model = False
        for i in range(len(model.name)):
            if model.name[i] == self.modelName:
                self.check_model = True

    def respawnModel(self):
        while True:
            if not self.check_model:
                rospy.wait_for_service('gazebo/spawn_sdf_model')
                spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
                spawn_model_prox(self.modelName, self.model, 'robotos_name_space', self.object_position, "world")
                rospy.loginfo("RespawnModel: %s", self.modelName)
                break
            else:
                pass
    
    def deleteModel(self):
        while True:
            if self.check_model:
                rospy.wait_for_service('gazebo/delete_model')
                del_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
                del_model_prox(self.modelName)
                break
            else:
                pass

    def setPose(self, position_check=False, delete=False):
        if delete:
            self.deleteModel()

        while position_check:
            self.index = random.randrange(0,len(self.object_x_list))

            if self.last_index == self.index:
                position_check = True
            else:
                self.last_index = self.index
                position_check = False
            
            self.object_position.position.x = self.object_x_list[self.index]
            self.object_position.position.y = self.object_y_list[self.index]
            self.object_position.position.z = self.object_z_list[self.index]

        time.sleep(.5)
        self.respawnModel()

        self.last_object_x = self.object_position.position.x
        self.last_object_y = self.object_position.position.y
        self.last_object_z = self.object_position.position.z






