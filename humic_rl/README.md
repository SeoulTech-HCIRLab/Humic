# HCIR Lab(Human-Centered Intelligent Robotics Lab)
   * https://sites.google.com/view/hcir
## Humic Robot ROS Reinforcement Learning(using gazebo) package

## Dependency
  
   * Install anaconda 
      
       * https://docs.anaconda.com/anaconda/install/
   
   * ROS1 melodic python3 환경 구축하기(ROS pkg는 python2.7로 로컬에 설치하고, python 스크립트만 python3 패키지들을 사용할 수 있도록 함.) 
   
          https://medium.com/@zuxinl/ubuntu-18-04-ros-python3-anaconda-cuda-environment-configuration-cb8c8e42c68d
          $sudo apt-get install python3-pip python3-yaml
          $pip install rospkg catkin_pkg
          
          # Make Conda Environment
          $ conda create -n Humic python=3.6
   
          
   * To Use Anaconda with ROS
    
          $pip install -U rosinstall msgpack empy defusedxml netifaces
   
   * Install tensorflow-cpu or gpu version
        
         $conda install tensorflow(cpu)
         $conda install tensorflow-gpu
   
   
          
   * Dependency python Library
      
          $conda install sip
          $conda install matplotlib
          $conda install numpy scipy numba
          $conda install -c conda-forge quaternion # https://github.com/moble/quaternion
          $pip install gym
    
   * how use in conda python3.6 -> from pykdl_utils.kdl_kinematics import KDLKinematics
      
          hrl-kdl pkg download in /home/young
          $cd hrl-kdl {python_utils folder}
          $conda create -n humic python=3.6
          $pip install rospkg
          $conda activate humic
          $pip install .
          anaconda3/envs/{conda_env}/lib/python3.6/site-packages 안에 만들어진다
          내용은 python2 version print 사용하기 때문에 ()를 추가
      
   * how to make python3 PyKDL(It must be compile in local)
         
        * https://github.com/orocos/orocos_kinematics_dynamics/issues/115
