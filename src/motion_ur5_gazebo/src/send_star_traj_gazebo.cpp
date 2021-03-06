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


int f_start = 0;

Vector_d_7x1 star_trayectory_generator(double t);

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
	ros::Publisher  pub = nh.advertise<my_control_gazebo::Pose>("impedance_controller/command", 100);
	ros::Subscriber sub = nh.subscribe("impedance_controller/f_start", 100, startCallback);

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
		pose_eigen = star_trayectory_generator(t);

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

		//time in seconds
		t = t + 0.001;
		//ROS_INFO("Time: %.3f", t);			
		loop_rate.sleep();
		}

		//Do this.
		ros::spinOnce();		
	}
	return 0;
}


Vector_d_7x1 star_trayectory_generator(double t){


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
	//double f 	 = 0.1;		 // [Hz]
	//double w 	 = 2*M_PI*f; // [rad/s]

	// Center of trajectory
	double x0_tray = r_min + 0.5*(r_max - r_min); x0_tray *=-1;
	double y0_tray = y_min + 0.5*(y_max - y_min);

	// time to perform line trajectory
	double T = 2.5;
	// star trajectory has eight position
	/*
	pos1: x=-0.4; y= 0.0
	pos3: x=-0.5; y= 0.1
	pos5: x=-0.6; y= 0.0
	pos7: x=-0.5; y=-0.1
	
			5
		6		4
	7				3
		8		2
			1
	*/
    int pos     = int(t/(2*T)) + 1;		// 0
    int vueltas = int(pos/8);			// 0
    int aspas   = pos - 1 - vueltas*8;	// 0
	double tiempo  = t - (pos-1)*(2*T); // time from 0 to 2T
    double angulo =  aspas*2*M_PI/8; 	// 0

    double xf_tray =  x0_tray + r_cir*std::cos(angulo+M_PI/2); // -0.5  
    double yf_tray =  y0_tray + r_cir*std::sin(angulo+M_PI/2); //  +0.1

    double x = 0.0;
    double y = 0.0;

    if ((tiempo >=0) & (tiempo<T))
    {
    	//alcanzar vaso
        x =  x0_tray + (xf_tray - x0_tray)/(T)*(tiempo);
        y =  y0_tray + (yf_tray - y0_tray)/(T)*(tiempo);
    }

    else if((tiempo >=T) & (tiempo<=(2*T)))
    {
    	// llevar vaso
        x = xf_tray + (x0_tray - xf_tray)/(T)*(tiempo-T);
        y = yf_tray + (y0_tray - yf_tray)/(T)*(tiempo-T);
    }

	// Pose 
	pose_eigen(0) = x;
	pose_eigen(1) = y;
	pose_eigen(2) = 0.0;
	pose_eigen(3) = 0.000695258;
	pose_eigen(4) = 0.0102506;
	pose_eigen(5) = 0.999807;
	pose_eigen(6) = 0.0167624;

	return pose_eigen;
}










