Ros Workspace: 
--------------
    Info:
    -----
    - Control the ur5 robot with the proportional-derivative and sliding mode control method. The two control methods modify their control gains with a cost function focus on minimizing the position and jerk error. Finally, the robot with the control methods could be simulated in gazebo or rviz.

    Requirements:
    -------------
    - Ros Noetic
    - RBDL (review how_to_install_rbdl_python3.txt)
    - Pandas (>=2.24)

    Install: (in process)
    --------
    - Set ros workspace as "~/catkin_ws/journal_ws/"

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
