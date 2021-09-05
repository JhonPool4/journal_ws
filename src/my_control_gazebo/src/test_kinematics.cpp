// urKinematics
#include <ur_kinematics/forward.h>
#include <std_msgs/Float64.h>
#include "control_gazebo/Float64Array.h"
#include "control_gazebo/Jacobian.h"

// Ros
#include "ros/ros.h"


void print(int rows, int columns, std::vector<double> A);

int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "dummy_node");
	ros::NodeHandle nh;
	ros::Publisher chatter_pub = nh.advertise<control_gazebo::Jacobian>("chatter", 100);	
	ros::Rate loop_rate(1);
	
	std::vector<double> M(8,0.0);
	std::vector<double> A(4,0.0);
	std::vector<double> C(8,0.0);

	M[0] = 2;
	M[1] = 9;
	M[2] = 5;
	M[3] = 3;
	M[4] = 6;
	M[5] = 1;

	M[6] = 1;
	M[7] = 1;

	A[0] = 1;
	A[1] = 2;
	A[2] = 3;
	A[3] = 4;

	//ur_kinematics::mulMatrix(4, 2, 2, 2, M, A, C);
	//ur_kinematics::print(4, 2, C);
	
	//ur_kinematics::print(2,2, M[0:5]);
	//int rows=3, columns=2;
	//ROS_INFO("matrix M \t");
	//ur_kinematics::print(rows, columns, M);
	//ur_kinematics::peusoInverseMatrix(rows, columns, M, M_inv);
	//ur_kinematics::print(columns, rows, M_inv);
	//ROS_INFO("inv matrix M \t");
	//ur_kinematics::print(columns, rows, M_inv);
	/*
	ur_kinematics::transposeMatrix(rows, columns, M, M_tran);
	ur_kinematics::mulMatrix(columns, rows, rows, columns, M_tran, M, M_rspt);
	ROS_INFO("first matrix");
	print(rows, columns, M);

	ROS_INFO("second matrix");
	print(columns, rows, M_tran);

	ROS_INFO("multiply matrix");
	print(columns, columns, M_rspt);*/


}

