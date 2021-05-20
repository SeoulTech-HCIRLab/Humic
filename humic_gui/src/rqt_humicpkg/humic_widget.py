import os
import rospkg
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_srvs.srv import Empty
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtGui import QKeySequence
from python_qt_binding.QtWidgets import QShortcut
from python_qt_binding.QtWidgets import QWidget

import time

""" YCB Object Model """
from rqt_humicpkg.ycb_object import YCBObject

class HumicWidget(QWidget):
    def __init__(self):
        super(HumicWidget, self).__init__()
        self.setObjectName('HumicWidget')

        self.ui_filename = 'HumicController.ui'

        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('humic_gui'), 'resource', self.ui_filename)
        loadUi(ui_file, self)

        """ Mecanum Wheel(Mobile Platform) """
        self.pub_velocity = Twist()
        self.pub_velocity.linear.x = 0.0
        self.pub_velocity.linear.y = 0.0
        self.pub_velocity.angular.z = 0.0

        self.PUBLISH_INTERVAL = 100
        self.MODE = 0
        
        self.cmd_vel_pub = rospy.Publisher('/humic/cmd_vel', Twist, queue_size=1)

        self.cmd_pub_timer = QTimer(self)
        self.cmd_pub_timer.timeout.connect(self.sendVelocity)
        self.cmd_pub_timer.start(self.PUBLISH_INTERVAL)

        self.pushButton_w.pressed.connect(self.front)
        self.pushButton_x.pressed.connect(self.back)
        self.pushButton_a.pressed.connect(self.left)
        self.pushButton_d.pressed.connect(self.right)
        self.pushButton_q.pressed.connect(self.frontLeft)
        self.pushButton_e.pressed.connect(self.frontRight)
        self.pushButton_z.pressed.connect(self.backLeft)
        self.pushButton_c.pressed.connect(self.backRight)
        self.pushButton_r.pressed.connect(self.turnLeft)
        self.pushButton_f.pressed.connect(self.turnRight)
        self.pushButton_s.pressed.connect(self.stop)

        self.pushButton_w.setShortcut('w')
        self.pushButton_x.setShortcut('x')
        self.pushButton_a.setShortcut('a')
        self.pushButton_d.setShortcut('d')
        self.pushButton_q.setShortcut('q')
        self.pushButton_e.setShortcut('e')
        self.pushButton_z.setShortcut('z')
        self.pushButton_c.setShortcut('c')
        self.pushButton_r.setShortcut('r')
        self.pushButton_f.setShortcut('f')
        self.pushButton_s.setShortcut('s')

        """ Right and Left Arm """
        self.right_arm_timer = QTimer(self)
        self.right_arm_timer.timeout.connect(self.sendTrajectoryRightArm)
        self.right_arm_timer.start(self.PUBLISH_INTERVAL)

        self.r_arm_joint_angles = [0, 0, 0, 0, 0, 0]
        self.r_arm_joints = ['r_shoulder_joint','r_upperArm_joint','r_elbow_joint','r_foreArm_joint','r_lowerArm_joint','r_wrist_joint']
        self.r_arm_pub = rospy.Publisher('/humic/right_arm/command', JointTrajectory, queue_size=1)

        self.doubleSpinBox_r_shoulder.valueChanged.connect(self.rShoulder)
        self.doubleSpinBox_r_upperArm.valueChanged.connect(self.rUpperArm)
        self.doubleSpinBox_r_elbow.valueChanged.connect(self.rElbow)
        self.doubleSpinBox_r_foreArm.valueChanged.connect(self.rForeArm)
        self.doubleSpinBox_r_lowerArm.valueChanged.connect(self.rLowerArm)
        self.doubleSpinBox_r_wrist.valueChanged.connect(self.rWrist)

        self.left_arm_timer = QTimer(self)
        self.left_arm_timer.timeout.connect(self.sendTrajectoryLeftArm)
        self.left_arm_timer.start(self.PUBLISH_INTERVAL)

        self.l_arm_joint_angles = [0, 0, 0, 0, 0, 0]
        self.l_arm_joints = ['l_shoulder_joint','l_upperArm_joint','l_elbow_joint','l_foreArm_joint','l_lowerArm_joint','l_wrist_joint']
        self.l_arm_pub = rospy.Publisher('/humic/left_arm/command', JointTrajectory, queue_size=1)

        self.doubleSpinBox_l_shoulder.valueChanged.connect(self.lShoulder)
        self.doubleSpinBox_l_upperArm.valueChanged.connect(self.lUpperArm)
        self.doubleSpinBox_l_elbow.valueChanged.connect(self.lElbow)
        self.doubleSpinBox_l_foreArm.valueChanged.connect(self.lForeArm)
        self.doubleSpinBox_l_lowerArm.valueChanged.connect(self.lLowerArm)
        self.doubleSpinBox_l_wrist.valueChanged.connect(self.lWrist)

        """ Fingers Control"""
        self.right_finger1_pub = rospy.Publisher('/humic/right_finger1/command', Float64, queue_size=1)
        self.right_finger2_pub = rospy.Publisher('/humic/right_finger2/command', Float64, queue_size=1)
        self.right_finger3_pub = rospy.Publisher('/humic/right_finger3/command', Float64, queue_size=1)
        self.right_finger4_pub = rospy.Publisher('/humic/right_finger4/command', Float64, queue_size=1)
        self.right_finger5_pub = rospy.Publisher('/humic/right_finger5/command', Float64, queue_size=1)
        self.right_finger6_pub = rospy.Publisher('/humic/right_finger6/command', Float64, queue_size=1)
        self.right_finger7_pub = rospy.Publisher('/humic/right_finger7/command', Float64, queue_size=1)
        self.right_finger8_pub = rospy.Publisher('/humic/right_finger8/command', Float64, queue_size=1)
        self.right_thumb_pub = rospy.Publisher('/humic/right_thumb/command', Float64, queue_size=1)

        self.left_finger1_pub = rospy.Publisher('/humic/left_finger1/command', Float64, queue_size=1)
        self.left_finger2_pub = rospy.Publisher('/humic/left_finger2/command', Float64, queue_size=1)
        self.left_finger3_pub = rospy.Publisher('/humic/left_finger3/command', Float64, queue_size=1)
        self.left_finger4_pub = rospy.Publisher('/humic/left_finger4/command', Float64, queue_size=1)
        self.left_finger5_pub = rospy.Publisher('/humic/left_finger5/command', Float64, queue_size=1)
        self.left_finger6_pub = rospy.Publisher('/humic/left_finger6/command', Float64, queue_size=1)
        self.left_finger7_pub = rospy.Publisher('/humic/left_finger7/command', Float64, queue_size=1)
        self.left_finger8_pub = rospy.Publisher('/humic/left_finger8/command', Float64, queue_size=1)
        self.left_thumb_pub = rospy.Publisher('/humic/left_thumb/command', Float64, queue_size=1)

        self.pushButton_right_grip.pressed.connect(self.rightGrip)
        self.pushButton_right_place.pressed.connect(self.rightPlace)
        self.pushButton_left_grip.pressed.connect(self.leftGrip)
        self.pushButton_left_place.pressed.connect(self.leftPlace)

        """ Neck and Head """
        self.head_timer = QTimer(self)
        self.head_timer.timeout.connect(self.sendTrajectoryHead)
        self.head_timer.start(self.PUBLISH_INTERVAL)

        self.head_joint_angles = [0, 0]
        self.head_joints = ['neck_joint','head_joint']
        self.head_pub = rospy.Publisher('/humic/head/command', JointTrajectory, queue_size=1)

        self.doubleSpinBox_neck.valueChanged.connect(self.neck)
        self.doubleSpinBox_head.valueChanged.connect(self.head)

        """" Lift Column """
        self.lift_column_timer = QTimer(self)
        self.lift_column_timer.timeout.connect(self.sendLiftColumn)
        self.lift_column_timer.start(self.PUBLISH_INTERVAL)

        self.lift_column_pub = rospy.Publisher('/humic/lift_column/command', Float64, queue_size=1)
        self.lift_column_val = 0.1
        
        self.doubleSpinBox_lift_column.valueChanged.connect(self.liftColumn)

        """ Gazebo """
        self.reset_simulation_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.reset_world_proxy = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)

        self.pushButton_reset.pressed.connect(self.reset)
        self.pushButton_respawn.pressed.connect(self.respawn)
        self.pushButton_planning.pressed.connect(self.PlanningTask)

        """ YCB Object """
        self.object_list = ['003_cracker_box', '004_sugar_box', '006_mustard_bottle', '009_gelatin_box', '010_potted_meat_can', '021_bleach_cleanser']
        self.cracker = YCBObject(model=self.object_list[0])
        self.sugar = YCBObject(model=self.object_list[1])
        self.mustard = YCBObject(model=self.object_list[2])
        self.gelatin = YCBObject(model=self.object_list[3])
        self.meatcan = YCBObject(model=self.object_list[4])
        self.cleanser = YCBObject(model=self.object_list[5])

        self.init_obj = True
    
    def rightGrip(self):
        self.right_finger1_pub.publish(1.1)
        self.right_finger2_pub.publish(1.5)
        self.right_finger3_pub.publish(1.3)
        self.right_finger4_pub.publish(1.57)
        self.right_finger5_pub.publish(1.3)
        self.right_finger6_pub.publish(1.57)
        self.right_finger7_pub.publish(1.3)
        self.right_finger8_pub.publish(1.57)
        self.right_thumb_pub.publish(0)
    
    def rightPlace(self):
        self.right_finger1_pub.publish(0)
        self.right_finger2_pub.publish(0)
        self.right_finger3_pub.publish(0)
        self.right_finger4_pub.publish(0)
        self.right_finger5_pub.publish(0)
        self.right_finger6_pub.publish(0)
        self.right_finger7_pub.publish(0)
        self.right_finger8_pub.publish(0)
        self.right_thumb_pub.publish(0)

    def leftGrip(self):
        self.left_finger1_pub.publish(1.1)
        self.left_finger2_pub.publish(1.5)
        self.left_finger3_pub.publish(1.3)
        self.left_finger4_pub.publish(1.57)
        self.left_finger5_pub.publish(1.3)
        self.left_finger6_pub.publish(1.57)
        self.left_finger7_pub.publish(1.3)
        self.left_finger8_pub.publish(1.57)
        self.left_thumb_pub.publish(0)

    def leftPlace(self):
        self.left_finger1_pub.publish(0)
        self.left_finger2_pub.publish(0)
        self.left_finger3_pub.publish(0)
        self.left_finger4_pub.publish(0)
        self.left_finger5_pub.publish(0)
        self.left_finger6_pub.publish(0)
        self.left_finger7_pub.publish(0)
        self.left_finger8_pub.publish(0)
        self.left_thumb_pub.publish(0)

    def liftColumn(self):
        self.lift_column_val = self.doubleSpinBox_lift_column.value()

    def sendLiftColumn(self):
        self.lift_column_pub.publish(self.lift_column_val)

    def neck(self):
        self.head_joint_angles[0] = self.doubleSpinBox_neck.value()
    
    def head(self):
        self.head_joint_angles[1] = self.doubleSpinBox_head.value()
    
    def sendTrajectoryHead(self):
        angles = self.getTrajectoryMsg(self.head_joints, self.head_joint_angles)
        self.head_pub.publish(angles)

    def rShoulder(self):
        self.r_arm_joint_angles[0] = self.doubleSpinBox_r_shoulder.value()

    def rUpperArm(self):
        self.r_arm_joint_angles[1] = self.doubleSpinBox_r_upperArm.value()
    
    def rElbow(self):
        self.r_arm_joint_angles[2] = self.doubleSpinBox_r_elbow.value()

    def rForeArm(self):
        self.r_arm_joint_angles[3] = self.doubleSpinBox_r_foreArm.value()
    
    def rLowerArm(self):
        self.r_arm_joint_angles[4] = self.doubleSpinBox_r_lowerArm.value()
    
    def rWrist(self):
        self.r_arm_joint_angles[5] = self.doubleSpinBox_r_wrist.value()

    def lShoulder(self):
        self.l_arm_joint_angles[0] = self.doubleSpinBox_l_shoulder.value()

    def lUpperArm(self):
        self.l_arm_joint_angles[1] = self.doubleSpinBox_l_upperArm.value()
    
    def lElbow(self):
        self.l_arm_joint_angles[2] = self.doubleSpinBox_l_elbow.value()

    def lForeArm(self):
        self.l_arm_joint_angles[3] = self.doubleSpinBox_l_foreArm.value()
    
    def lLowerArm(self):
        self.l_arm_joint_angles[4] = self.doubleSpinBox_l_lowerArm.value()
    
    def lWrist(self):
        self.l_arm_joint_angles[5] = self.doubleSpinBox_l_wrist.value()

    def sendTrajectoryRightArm(self):
        angles = self.getTrajectoryMsg(self.r_arm_joints, self.r_arm_joint_angles)
        self.r_arm_pub.publish(angles)
    
    def sendTrajectoryLeftArm(self):
        angles = self.getTrajectoryMsg(self.l_arm_joints, self.l_arm_joint_angles)
        self.l_arm_pub.publish(angles)

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
    
    def front(self):
        if self.MODE != 1:
            self.pub_velocity.linear.x = 0.0
            self.pub_velocity.linear.y = 0.0
            self.pub_velocity.angular.z = 0.0
        self.MODE = 1
        self.pub_velocity.linear.x += 0.1

    def back(self):
        if self.MODE != 2:
            self.pub_velocity.linear.x = 0.0
            self.pub_velocity.linear.y = 0.0
            self.pub_velocity.angular.z = 0.0
        self.MODE = 2
        self.pub_velocity.linear.x -= 0.1

    def left(self):
        if self.MODE != 3:
            self.pub_velocity.linear.x = 0.0
            self.pub_velocity.linear.y = 0.0
            self.pub_velocity.angular.z = 0.0
        self.MODE = 3
        self.pub_velocity.linear.y += 0.1

    def right(self):
        if self.MODE != 4:
            self.pub_velocity.linear.x = 0.0
            self.pub_velocity.linear.y = 0.0
            self.pub_velocity.angular.z = 0.0
        self.MODE = 4
        self.pub_velocity.linear.y -= 0.1
    
    def frontLeft(self):
        if self.MODE != 5:
            self.pub_velocity.linear.x = 0.0
            self.pub_velocity.linear.y = 0.0
            self.pub_velocity.angular.z = 0.0
        self.MODE = 5
        self.pub_velocity.linear.x += 0.1
        self.pub_velocity.linear.y += 0.1
    
    def frontRight(self):
        if self.MODE != 6:
            self.pub_velocity.linear.x = 0.0
            self.pub_velocity.linear.y = 0.0
            self.pub_velocity.angular.z = 0.0
        self.MODE = 6
        self.pub_velocity.linear.x += 0.1
        self.pub_velocity.linear.y -= 0.1
    
    def backLeft(self):
        if self.MODE != 7:
            self.pub_velocity.linear.x = 0.0
            self.pub_velocity.linear.y = 0.0
            self.pub_velocity.angular.z = 0.0
        self.MODE = 7
        self.pub_velocity.linear.x -= 0.1
        self.pub_velocity.linear.y += 0.1
    
    def backRight(self):
        if self.MODE != 8:
            self.pub_velocity.linear.x = 0.0
            self.pub_velocity.linear.y = 0.0
            self.pub_velocity.angular.z = 0.0
        self.MODE = 8
        self.pub_velocity.linear.x -= 0.1
        self.pub_velocity.linear.y -= 0.1

    def turnLeft(self):
        self.pub_velocity.angular.z += 0.1

    def turnRight(self):
        self.pub_velocity.angular.z -= 0.1

    def stop(self):
        self.pub_velocity.linear.x = 0.0
        self.pub_velocity.linear.y = 0.0
        self.pub_velocity.angular.z = 0.0
    
    def sendVelocity(self):
        twist = Twist()
        twist.linear.x = self.pub_velocity.linear.x
        twist.linear.y = self.pub_velocity.linear.y
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.pub_velocity.angular.z
        self.cmd_vel_pub.publish(twist)

    def shutdown_widget(self):
        self.cmd_pub_timer.stop()

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

    def reset(self):
        """
        Reset Gazebo Simulator
        """
        self.resetRobotState()
        self.pauseSim()
        self.resetWorld()
        self.unpauseSim()
        self.resetSimulation()

    def resetRobotState(self):
        self.pub_velocity.linear.x = 0.0
        self.pub_velocity.linear.y = 0.0
        self.pub_velocity.angular.z = 0.0
        
        self.r_arm_joint_angles = [0, 0, 0, 0, 0, 0]
        self.doubleSpinBox_r_shoulder.setValue(0.0)
        self.doubleSpinBox_r_upperArm.setValue(0.0)
        self.doubleSpinBox_r_elbow.setValue(0.0)
        self.doubleSpinBox_r_foreArm.setValue(0.0)
        self.doubleSpinBox_r_lowerArm.setValue(0.0)
        self.doubleSpinBox_r_wrist.setValue(0.0)
        self.sendTrajectoryRightArm()

        self.l_arm_joint_angles = [0, 0, 0, 0, 0, 0]
        self.doubleSpinBox_l_shoulder.setValue(0.0)
        self.doubleSpinBox_l_upperArm.setValue(0.0)
        self.doubleSpinBox_l_elbow.setValue(0.0)
        self.doubleSpinBox_l_foreArm.setValue(0.0)
        self.doubleSpinBox_l_lowerArm.setValue(0.0)
        self.doubleSpinBox_l_wrist.setValue(0.0)
        self.sendTrajectoryLeftArm()

        self.head_joint_angles = [0, 0]
        self.doubleSpinBox_neck.setValue(0.0)
        self.doubleSpinBox_head.setValue(0.0)
        self.sendTrajectoryHead()

        self.lift_column_val = 0.1
        self.doubleSpinBox_lift_column.setValue(0.1)
        self.sendLiftColumn()
        
        self.rightPlace()
        self.leftPlace()

    def respawn(self):
        if self.init_obj:
            self.setObjectPose()
            self.init_obj = False
        else:
            self.setObjectPose(position_check=True, delete=True)

    def setObjectPose(self, position_check=False, delete=False):
        self.cracker.setPose(position_check=position_check, delete=delete)
        self.sugar.setPose(position_check=position_check, delete=delete)
        self.mustard.setPose(position_check=position_check, delete=delete)
        self.gelatin.setPose(position_check=position_check, delete=delete)
        self.meatcan.setPose(position_check=position_check, delete=delete)
        self.cleanser.setPose(position_check=position_check, delete=delete)

    def PlanningTask(self):
        self.r_arm_joint_angles = [0, -1.57, 0, 0, 0, 0]
        self.l_arm_joint_angles = [0, -1.57, 0, 0, 0, 0]
        angles = self.getTrajectoryMsg(self.r_arm_joints, self.r_arm_joint_angles)
        self.r_arm_pub.publish(angles)
        angles = self.getTrajectoryMsg(self.l_arm_joints, self.l_arm_joint_angles)
        self.l_arm_pub.publish(angles)

        time.sleep(1.5)

        self.r_arm_joint_angles = [1.4, -1.57, 0.52, 0, 0, 0]
        self.l_arm_joint_angles = [1.4, -1.57, 0.52, 0, 0, 0]
        angles = self.getTrajectoryMsg(self.r_arm_joints, self.r_arm_joint_angles)
        self.r_arm_pub.publish(angles)
        angles = self.getTrajectoryMsg(self.l_arm_joints, self.l_arm_joint_angles)
        self.l_arm_pub.publish(angles)

        time.sleep(1.5)

        self.r_arm_joint_angles = [1.4, -0.06, 0.52, 0, 0.4, 0]
        self.l_arm_joint_angles = [1.4, -0.06, 0.52, 0, 0.4, 0]
        angles = self.getTrajectoryMsg(self.r_arm_joints, self.r_arm_joint_angles)
        self.r_arm_pub.publish(angles)
        angles = self.getTrajectoryMsg(self.l_arm_joints, self.l_arm_joint_angles)
        self.l_arm_pub.publish(angles)


