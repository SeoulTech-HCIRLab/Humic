# HCIR Lab(Human-Centered Intelligent Robotics Lab)
   * https://sites.google.com/view/hcir
## Humic Robot ROS Gazebo package

### Dependency
    $sudo apt-get install -y ros-noetic-joint-*
    $sudo apt-get install -y ros-noetic-gazebo-*
    $sudo apt-get install -y ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro
    $sudo apt-get install -y ros-noetic-compressed-image-transport ros-noetic-rqt-image-view ros-noetic-interactive-markers
    
## How to run gazebo simulator
1. run launch file
      
        $ roslaunch humic_gazebo humic.launch

2. You can see Humic Robot in gazebo
  ![Humic_in_gazebo_empty](https://user-images.githubusercontent.com/37207332/111419168-4e7c1e80-872c-11eb-9de5-afb159a405f5.png)

3. You can control Joint
  * Joints
         
         1. Right Arm: r_shoulder, r_upperArm, r_elbow, r_foreArm, r_lowerArm, r_wrist
         2. Left Arm: l_shoulder, l_upperArm, l_elbow, l_foreArm, l_lowerArm, l_wrist
         3. Lift Column
         4. Neck & Head
         5. Right Finger: r_finger1_1, r_finger1_2, r_finger2_1, r_finger2_2, r_finger3_1, r_finger3_2, r_finger4_1, r_finger4_2, r_thumb
         6. Left Finger: l_finger1_1, l_finger1_2, l_finger2_1, l_finger2_2, l_finger3_1, l_finger3_2, l_finger4_1, l_finger4_2, l_thumb
         7. Mecanum Wheel(Mobile Platform)
