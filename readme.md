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

    Config workspace: (in process)
    ----------------
    - mkdir -p ~/catkin_ws/journal_ws/src
    - cd ~/catkin_ws/journal_ws/ 
    - catkin_make
    - gedit ~/.bashrc
    - Add these lines:
        # ROS
        source /opt/ros/noetic/setup.bash

        # ROS WORKSPACE
        work_space="catkin_ws/journal_ws"
        source ~/$work_space/devel/setup.bash

    - source ~/.bashrc
    - cd ~/catkin_ws/
    - git clone https://github.com/JhonPool4/journal_ws.git
    - cd ~/catkin_ws/journal_ws/ 
    - catkin_make    
    
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
    - ur5 robot + PD control method: roslaunch motion_ur5_gazebo articular_PDi_UR5.launch(process)
    - ur5 robot + sliding mode control method: roslaunch motion_ur5_gazebo articular_SMCi_UR5.launch (process)

    my_control_gazego:
    ------------------
  




Doing:
------    
-- improve documentation of each ros package and commands

To Do:
------
- add optimization equations in pd control template
- solve position problems in /joint_states

Done
------
-- implement a pd control template gazebo
