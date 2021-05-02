#!/usr/bin/env python3

# Refer to ROBOTIS Turtlebot3 Machine Learning package
#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################

# Authors: Gilbert #

import rospy
import random
import time
import os
from gazebo_msgs.srv import SpawnModel, DeleteModel
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose

class Respawn():
    def __init__(self, mapName=None):
        self.modelPath = os.path.dirname(os.path.realpath(__file__))
        self.modelPath = self.modelPath.replace('humic_rl/src', 'humic_gazebo/models/goal_box/model.sdf')
        self.f = open(self.modelPath, 'r')
        self.model = self.f.read()
        self.mapName = mapName
        
        self.goal_position = Pose()
        self.init_goal_x = 1.0
        self.init_goal_y = 1.0
        self.goal_position.position.x = self.init_goal_x
        self.goal_position.position.y = self.init_goal_y
        self.modelName = 'goal'

        self.last_goal_x = self.init_goal_x
        self.last_goal_y = self.init_goal_y
        self.last_index = 0
        self.sub_model = rospy.Subscriber('gazebo/model_states', ModelStates, self.checkModel)
        self.check_model = False
        self.index = 0

    def checkModel(self, model):
        self.check_model = False
        for i in range(len(model.name)):
            if model.name[i] == "goal":
                self.check_model = True

    def respawnModel(self):
        while True:
            if not self.check_model:
                rospy.wait_for_service('gazebo/spawn_sdf_model')
                spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
                spawn_model_prox(self.modelName, self.model, 'robotos_name_space', self.goal_position, "world")
                rospy.loginfo("Goal position : %.1f, %.1f", self.goal_position.position.x,
                              self.goal_position.position.y)
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

    def getPosition(self, position_check=False, delete=False):
        if delete:
            self.deleteModel()

        while position_check:
            if self.mapName == 'room': 
                goal_x_list = [1.0, 1.0, 1.5, 2.0,  2.0, 0.0,  0.0, 0.5,  0.5, -0.5, -1.0, -1.5, -1.5, -1.5, -2.0]
                goal_y_list = [1.0, 2.0, 1.5, 2.0, -0.5, 2.0, -2.0, 1.5, -1.5, -1.5, -1.0,  0.5, -1.5, -2.5, -2.0]
                self.index = random.randrange(0,len(goal_x_list))
            elif self.mapName == 'building':
                goal_x_list = [1.0, 1.0, 1.5, 2.0,  2.0, 0.0,  0.0, 0.5,  0.5, -0.5, -1.0, -1.5, -1.5, -1.5, -2.0, -2.5, -4.0, -5.0, -6.0, -7.0]
                goal_y_list = [1.0, 2.0, 1.5, 2.0, -0.5, 2.0, -2.0, 1.5, -1.5, -1.5, -1.0,  0.5, -1.5, -2.5, -2.0, -1.5, -5.0,  2.0, -3.0,  0.0]
                self.index = random.randrange(0,len(goal_x_list))         
            
            if self.last_index == self.index:
                position_check = True
            else:
                self.last_index = self.index
                position_check = False

            self.goal_position.position.x = goal_x_list[self.index]
            self.goal_position.position.y = goal_y_list[self.index]

        time.sleep(0.5)
        self.respawnModel()

        self.last_goal_x = self.goal_position.position.x
        self.last_goal_y = self.goal_position.position.y

        return self.goal_position.position.x, self.goal_position.position.y