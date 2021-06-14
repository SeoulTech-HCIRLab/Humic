# Humic ROS package(ROS Version: Noetic or Melodic)

## 1. Installation ROS Noetic or Melodic

### Installation instructions

* http://wiki.ros.org/noetic/Installation/Ubuntu
* http://wiki.ros.org/melodic/Installation/Ubuntu

### Installation noetic Script
     
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

* HUMIC rqt controller

       $ rqt
      
     * http://wiki.ros.org/rqt
     * Plugins option: HUMIC Controller selection
     * Perspectives option: HUMIC GUI.perspective selection
       
## 6. Run HUMIC Navigation using reinforcement learning

   * Terminate the gazebo, Terminate the HUMIC model in gazebo
   
   * Run Navigation world
   
            $ roslaunch humic_rl humic_navigation.launch
