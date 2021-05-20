# Humic ROS package(ROS Version: Noetic)

## HOW TO USE

### 1. Installation ROS Noetic

* http://wiki.ros.org/noetic/Installation/Ubuntu

### 2. Configuring Your ROS Environment

* http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

### 3. Download HUMIC Package in Your Workspace

* Move to Your ROS workspace
    
      $cd ~/catkin_ws/src
    
* Clone HUMIC package
   
      $git clone https://github.com/SeoulTech-HCIRLab/Humic.git

### 4. Build

      $cd ~/catkin_ws/ && catkin_make

### 5. Run

* HUMIC in gazebo world

       $ roslaunch humic_gazebo humic.launch

* HUMIC rqt controller

       $ rqt
       
     * Plugins option: HUMIC Controller selection
     * Perspectives option: HUMIC GUI.perspective selection
       
