// ROS
#include <ros/ros.h>

// ROS message
#include <my_control_gazebo/Pose.h>
#include <std_msgs/Int64.h>

// Math
#include <cmath>

// Eigen
#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;

// Circular trajectory generator
typedef Matrix<double, 7, 1> Vector_d_7x1;

Vector_d_7x1 circular_trayectory_generator(double t);

int f_start = 0;

void startCallback(const std_msgs::Int64::ConstPtr& msg)
{
	f_start = msg->data;
}


int main(int argc, char **argv)
{
	// Start Node
	ros::init(argc, argv, "send_trajectory_node");

	// Handle Node
	ros::NodeHandle nh;

	// Publisher
	ros::Publisher  pub = nh.advertise<my_control_gazebo::Pose>("PD_controller/command", 100);
	ros::Subscriber sub = nh.subscribe("PD_controller/f_start", 100, startCallback);

	// Rate
	ros::Rate loop_rate(1000);	 // 1000 Hz

	// Pose vector eigen
	Vector_d_7x1 pose_eigen;

	// Simulation time
	double t = 0.0;

	while (ros::ok())
	{
		if (f_start==1)
		{
		// Pose message
		my_control_gazebo::Pose pose;
		
		// Generate trajectory
		pose_eigen = circular_trayectory_generator(t);

		// Send Pose
		pose.x = pose_eigen(0);
		pose.y = pose_eigen(1);
		pose.z = pose_eigen(2);
		pose.w = pose_eigen(3);
		pose.ex= pose_eigen(4);
		pose.ey= pose_eigen(5);
		pose.ez= pose_eigen(6);

		//Publish array
		pub.publish(pose);

		//Added a delay so not to spam
		t = t + 0.001;
		//ROS_INFO("Time: %.3f", t);			
		loop_rate.sleep();
		}

		//Do this.
		ros::spinOnce();		
	}
	return 0;
}


Vector_d_7x1 circular_trayectory_generator(double t){


	Vector_d_7x1 pose_eigen; 	// pose   = [x, y, z, w, ex, ey, ez]
	//Vector_d_4x1 d_pose_eigen;  // d_pose = [dx, dy, dz, 0.0, 0.0, 0.0, 0.0]

	// Ubicación del paciente
	double x_paciente =  0.500; // [m]
	double y_paciente = -0.275; // [m]
	// Longitud del brazo del paciente
	double l 		  =  0.550; // [m]

	// Limits
	double r_max = 0.70; // [m]
	double r_min = 0.30; // [m]
	double y_max = y_paciente + 0.80*l; // [m]
	double y_min = y_paciente + 0.20*l; // [m]
	// Radious
	double r_cir = 0.1; 	// [m]
	// Frecuency
	double f 	 = 0.1;		 // [Hz]
	double w 	 = 2*M_PI*f; // [rad/s]

	// Center of trajectory
	double x0 = r_min + 0.5*(r_max - r_min);
	double y0 = y_min + 0.5*(y_max - y_min);

	// Position point of circular trajectory
	double x = -x0 + r_cir*std::cos(w*(t+2.5));
	double y = y0 + r_cir*std::sin(w*(t+2.5));

	// Velocity point of circular trajectory
	//double dx = -w*r_cir*std::sin(w*(t+2.5));
	//double dy =  w*r_cir*std::cos(w*(t+2.5));

	// Pose wiht fixed orientation
	pose_eigen(0) = x;
	pose_eigen(1) = y;
	pose_eigen(2) = 0.0;
	pose_eigen(3) = 0.000695258;
	pose_eigen(4) = 0.0102506;
	pose_eigen(5) = 0.999807;
	pose_eigen(6) = 0.0167624;

	return pose_eigen;
}










