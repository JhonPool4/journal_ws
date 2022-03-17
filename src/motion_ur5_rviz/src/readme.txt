
Useful commands:
----------------
    - xacro file.xacro > file.urdf
    - roslaunch motion_ur5_rviz new_cartesian_PDi_UR5.launch 
        trajectory:=circular
        exp=exp1
        uncertainty:=100 
        alpha:=0 
        sim_time:=100 
        record_rosbag:=true 
        

Updates:
--------
	- (working) new formulation of dq/du and du/dK 
    - (developing) new uncertainty test: 
        - nomi_robot: urdf model with nominal weights of ur5 links (factory values)
        - real_robot: urdf model with new weights of ur5 sixth link (25%, 50%, 75%, 100%)
        - control and optimization equations should use "nomi_robot" just state update should use
          "real_robot"  
