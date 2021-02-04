# Humic Right Arm and Hands Controller
# Kim Young Gi
# 2020. 07. 29

import rospy
from std_msgs.msg import Float64
import math # radians(deg), degrees(rad)

class right_arm_controller():
    def __init__(self):

        #self.position
        #self.orientation

        # right arm controller
        self.r_shoulder  = rospy.Publisher('/humic_control/r_shoulder/command', Float64, queue_size=1) # -3.14 ~ 3.14(-180 ~ 180)
        self.r_uppperArm = rospy.Publisher('/humic_control/r_upperArm/command', Float64, queue_size=1) # -1.57 ~ 1.57(-90 ~ 90) 
        self.r_elbow     = rospy.Publisher('/humic_control/r_elbow/command'   , Float64, queue_size=1) #  0.00 ~ 1.57(0 ~ 90)
        self.r_foreArm   = rospy.Publisher('/humic_control/r_foreArm/command' , Float64, queue_size=1) # -1.57 ~ 1.57(-90 ~ 90)
        self.r_lowerArm  = rospy.Publisher('/humic_control/r_lowerArm/command', Float64, queue_size=1) # -1.57 ~ 1.57(-90 ~ 90)
        self.r_wrist     = rospy.Publisher('/humic_control/r_wrist/command'   , Float64, queue_size=1) # -1.57 ~ 1.57(-90 ~ 90)
    
        # right arm hands(finger, thumb) controller
        self.r_finger11 = rospy.Publisher('humic_control/r_finger1_1/command', Float64, queue_size=1) # 0.00 ~ 1.57(0 ~ 90)
        self.r_finger12 = rospy.Publisher('humic_control/r_finger1_2/command', Float64, queue_size=1) # 0.00 ~ 1.57(0 ~ 90)
        self.r_finger21 = rospy.Publisher('humic_control/r_finger2_1/command', Float64, queue_size=1) # 0.00 ~ 1.57(0 ~ 90)
        self.r_finger22 = rospy.Publisher('humic_control/r_finger2_2/command', Float64, queue_size=1) # 0.00 ~ 1.57(0 ~ 90)
        self.r_finger31 = rospy.Publisher('humic_control/r_finger3_1/command', Float64, queue_size=1) # 0.00 ~ 1.57(0 ~ 90)
        self.r_finger32 = rospy.Publisher('humic_control/r_finger3_2/command', Float64, queue_size=1) # 0.00 ~ 1.57(0 ~ 90)
        self.r_finger41 = rospy.Publisher('humic_control/r_finger4_1/command', Float64, queue_size=1) # 0.00 ~ 1.57(0 ~ 90)
        self.r_finger42 = rospy.Publisher('humic_control/r_finger4_2/command', Float64, queue_size=1) # 0.00 ~ 1.57(0 ~ 90)
        self.r_thumb    = rospy.Publisher('humic_control/r_thumb/command'    , Float64, queue_size=1) # 0.00 ~ 0.785(0 ~ 45)

        # right arm, hands position
        self.right_arm_init_pos = {'r_shoulder':0.0, 'r_upperArm':math.radians(-90), 'r_elbow':0.0, 'r_foreArm':0.0, 'r_lowerArm':0.0, 'r_wrist':0.0}
        
        self.right_hands_init_pos = {'r_finger1_1':0.0, 'r_finger1_2':0.0, 'r_finger2_1':0.0, 'r_finger2_2':0.0, 
                                        'r_finger3_1':0.0, 'r_finger3_2':0.0, 'r_finger4_1':0.0, 'r_finger4_2':0.0, 'r_thumb':math.radians(45)
                                    }
        self.right_hands_grip_pos = {'r_finger1_1':math.radians(86), 'r_finger1_2':math.radians(57.3), 
                                        'r_finger2_1':math.radians(86), 'r_finger2_2':math.radians(57.3), 
                                        'r_finger3_1':math.radians(86), 'r_finger3_2':math.radians(57.3), 
                                        'r_finger4_1':math.radians(86), 'r_finger4_2':math.radians(57.3), 
                                        'r_thumb':math.radians(45)
                                    }

        

    def init_state(self):
        self.r_shoulder.publish(self.right_arm_init_pos['r_shoulder'])
        self.r_uppperArm.publish(self.right_arm_init_pos['r_upperArm'])
        self.r_elbow.publish(self.right_arm_init_pos['r_elbow'])
        self.r_foreArm.publish(self.right_arm_init_pos['r_foreArm'])
        self.r_lowerArm.publish(self.right_arm_init_pos['r_lowerArm'])
        self.r_wrist.publish(self.right_arm_init_pos['r_wrist'])

        self.r_finger11.publish(self.right_hands_init_pos['r_finger1_1'])
        self.r_finger12.publish(self.right_hands_init_pos['r_finger1_2'])
        self.r_finger21.publish(self.right_hands_init_pos['r_finger2_1'])
        self.r_finger22.publish(self.right_hands_init_pos['r_finger2_2'])
        self.r_finger31.publish(self.right_hands_init_pos['r_finger3_1'])
        self.r_finger32.publish(self.right_hands_init_pos['r_finger3_2'])
        self.r_finger41.publish(self.right_hands_init_pos['r_finger4_1'])
        self.r_finger42.publish(self.right_hands_init_pos['r_finger4_2'])
        self.r_thumb.publish(self.right_hands_init_pos['r_thumb'])

    # def grip(self):
    #     self.r_finger11.publish(self.right_hands_grip_pos['r_finger1_1'])
    #     self.r_finger12.publish(self.right_hands_grip_pos['r_finger1_2'])
    #     self.r_finger21.publish(self.right_hands_grip_pos['r_finger2_1'])
    #     self.r_finger22.publish(self.right_hands_grip_pos['r_finger2_2'])
    #     self.r_finger31.publish(self.right_hands_grip_pos['r_finger3_1'])
    #     self.r_finger32.publish(self.right_hands_grip_pos['r_finger3_2'])
    #     self.r_finger41.publish(self.right_hands_grip_pos['r_finger4_1'])
    #     self.r_finger42.publish(self.right_hands_grip_pos['r_finger4_2'])
    #     self.r_thumb.publish(self.right_hands_grip_pos['r_thumb'])
