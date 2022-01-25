# Ros workspace:
## Description:
Control the ur5 robot with the proportional-derivative and sliding mode control method. The two control methods modify their control gains with a cost function focus on minimizing the position and jerk error. Finally, the robot with the control methods could be simulated in gazebo or rviz.

## Requirements:
- Ros Noetic (http://wiki.ros.org/noetic/Installation/Ubuntu)
- Pinocchio (https://stack-of-tasks.github.io/pinocchio/download.html)
- Pandas (>=0.24)

## Config .bashrc file
Open .basrhc file
<pre><code>$ gedit ~/.bashrc </code></pre>
Add these lines:
<pre><code># ROS
source /opt/ros/noetic/setup.bash 
# ROS WORKSPACE (don't modify)
export work_space="${HOME}/catkin_ws/journal_ws"
source $work_space/devel/setup.bash
# EIGEN3: environment variable (could be modified)
export eigien3_include_dir="/usr/include/eigen3/Eigen"
</code></pre>    

## Config workspace:
Create the workspace
<pre><code>$ mkdir -p ~/catkin_ws/journal_ws/src 
$ cd ~/catkin_ws/
</code></pre>

Clone repository
<pre><code>$ git clone https://github.com/JhonPool4/journal_ws.git 
</code></pre>

Create necessary files
<pre><code>$ cd ~/catkin_ws/journal_ws/
$ catkin_make
</code></pre>

Additional ROS packages
<pre><code>$ sudo apt-get install ros-melodic-ros-control </code></pre>
<pre><code>$ sudo apt-get install ros-melodic-ros-controllers</code></pre>

    
## Packages info
### ur5_description (working)
- This package has urdf file of ur5 robot. 
- Display ur5 robot in rviz: roslaunch ur5_description ur5_rviz.launch
- Display ur5 robot in gazebo: roslaunch ur5_description ur5_gazebo.launch

### motion_ur5_rviz (working)
- This package use rviz and rbdl to simulate ur5 robot with the control methods.
- ur5 robot + proportional-derivative control method: roslaunch motion_ur5_rviz articular_PDi_UR5.launch
- ur5 robot + sliding mode control method: roslaunch motion_ur5_rviz articular_SMCi_UR5.launch

### motion_ur5_gazebo (developing)
- This package use gazebo and rbdl to simulate ur5 robot with the control methods.
- ur5 robot + PD control method: roslaunch motion_ur5_gazebo articular_PDi_UR5.launch (in process)
- ur5 robot + sliding mode control method: roslaunch motion_ur5_gazebo articular_SMCi_UR5.launch (in process)

### my_control_gazego (developing)
    



