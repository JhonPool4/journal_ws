Ros Workspace: 
--------------
    Info:
    -----
    - Control the ur5 robot with the proportional-derivative and sliding mode control method. 
    The two control methods modify their control gains with a cost function focus on minimizing 
    the position and jerk error. Finally, the robot with the control methods could be simulated 
    in gazebo or rviz.

    Requirements:
    -------------
    - Ros Noetic (http://wiki.ros.org/noetic/Installation/Ubuntu)
    - RBDL (review how_to_install_rbdl_orb)
    - Pandas (>=2.24)

    Config .bashrc:
    ---------------
    - gedit ~/.bashrc
    - Add these lines:
        # ROS
        source /opt/ros/noetic/setup.bash

        # ROS WORKSPACE (don't modify)
        export work_space="${HOME}/catkin_ws/journal_ws"
        source $work_space/devel/setup.bash

        # EIGEN3: environment variable (could be modified)
        export eigien3_include_dir="/usr/include/eigen3/Eigen"

        # RBDL: environment variables (modify to your setup!)
        export rbdl_include_dir="${HOME}/catkin_ws/rbdl_ws/install/include"
        export rbdl_lib="${HOME}/catkin_ws/rbdl_ws/install/lib"
        export rbdl_urdfreader_include_dir="${HOME}/catkin_ws/rbdl_ws/install/include"
        export rbdl_urdfreader_lib="${HOME}/catkin_ws/rbdl_ws/install/lib"
        
        # RBDL: python library (could be modified)
        export LD_LIBRARY_PATH=$rbdl_lib:$LD_LIBRARY_PATH
        export PYTHONPATH=$rbdl_lib/python/:$PYTHONPATH    

    Config workspace:
    ----------------
    - mkdir -p ~/catkin_ws/journal_ws/src
    - cd ~/catkin_ws/
    - git clone https://github.com/JhonPool4/journal_ws.git
    - cd ~/catkin_ws/journal_ws/ 
    - catkin_make    

    Aditional Ros packages:
    ---------------------
    - sudo apt-get install ros-melodic-ros-control
    - sudo apt-get install ros-melodic-ros-controllers

    
Packages info:
--------------
    ur5_description: 
    ---------------
    - This package has urdf file of ur5 robot. 
    - Display ur5 robot in rviz: roslaunch ur5_description ur5_rviz.launch
    - Display ur5 robot in gazebo: roslaunch ur5_description ur5_gazebo.launch
     
    motion_ur5_rviz:
    ----------------
    - This package use rviz and rbdl to simulate ur5 robot with the control methods.
    - ur5 robot + proportional-derivative control method: roslaunch motion_ur5_rviz articular_PDi_UR5.launch
    - ur5 robot + sliding mode control method: roslaunch motion_ur5_rviz articular_SMCi_UR5.launch

    motion_ur5_gazebo:
    ------------------
    - This package use gazebo and rbdl to simulate ur5 robot with the control methods.
    - ur5 robot + PD control method: roslaunch motion_ur5_gazebo articular_PDi_UR5.launch (in process)
    - ur5 robot + sliding mode control method: roslaunch motion_ur5_gazebo articular_SMCi_UR5.launch (in process)

    my_control_gazego:
    ------------------
  




Doing:
------    
- file to save data from circ trajectory

To Do:
------
- add optimization equations in PD control template
- solve position problems in /joint_states

Done
------
- implement a PD control template gazebo
- improve documentation of each ros package and commands