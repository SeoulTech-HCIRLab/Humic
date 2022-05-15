# Humic ROS package
### Paper
https://www.koreascience.or.kr/article/JAKO202124553294650.page
## 1. Installation ROS Noetic

### ROS Version: Noetic(on Ubuntu 20.04)

### Installation instructions

* http://wiki.ros.org/noetic/Installation/Ubuntu

### Or script
     
* https://robocademy.com/2020/05/23/getting-started-with-new-ros-noetic-ninjemys/
         
      wget -c https://raw.githubusercontent.com/qboticslabs/ros_install_noetic/master/ros_install_noetic.sh && chmod +x ./ros_install_noetic.sh && ./ros_install_noetic.sh 

## 2. Configuring Your ROS Environment

* http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

## 3. Download HUMIC Package in Your Workspace

* Move to Your ROS workspace
    
      $ cd ~/catkin_ws/src
    
* Clone HUMIC package
   
      $ git clone https://github.com/SeoulTech-HCIRLab/Humic.git

## 4. Build

      $ cd ~/catkin_ws/ && catkin_make

## 5. Run HUMIC URDF Model

* HUMIC in gazebo world

       $ roslaunch humic_gazebo humic.launch
     
     * Source code: https://github.com/SeoulTech-HCIRLab/Humic/tree/master/humic_description

* HUMIC rqt controller

       $ rqt
     
     * Source code: https://github.com/SeoulTech-HCIRLab/Humic/tree/master/humic_gui
     * http://wiki.ros.org/rqt
     * Plugins option: HUMIC Controller selection
     * Perspectives option: HUMIC GUI.perspective selection
       
## 6. Run HUMIC Navigation using reinforcement learning
   * This package was tested with Noetic and python3
   * Pytorch

   * Terminate the gazebo, Terminate the HUMIC model in gazebo
   
   * Run Navigation world
   
            $ roslaunch humic_rl humic_navigation.launch
