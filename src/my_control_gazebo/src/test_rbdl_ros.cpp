
// ROS Libraries
#include "ros/ros.h"
#include "std_msgs/Int64.h"


// RBDL Libraries
#include <iostream>
#include <rbdl/rbdl.h>
#ifndef RBDL_BUILD_ADDON_URDFREADER
	#error "Error: RBDL addon URDFReader not enabled."
#endif

#include <rbdl/addons/urdfreader/urdfreader.h>
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

#include <vector>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "dummy_node");
	ros::NodeHandle nh;
	ros::Publisher chatter_pub = nh.advertise<std_msgs::Int64>("chatter", 1000);
	ros::Rate loop_rate(10);

	rbdl_check_api_version (RBDL_API_VERSION);
	Model* model = new Model();

	if (!Addons::URDFReadFromFile ("/home/utec/catkin_ws/src/ur5_description/urdf/ur5_joint_limited_robot.urdf", model, false)) {
		std::cerr << "Error loading model ./samplemodel.urdf" << std::endl;
		abort();
	}

	VectorNd Q = VectorNd::Zero(6);
	MatrixNd M = MatrixNd::Zero(6,6);
	CompositeRigidBodyAlgorithm(*model, Q, M);
	std::cout<<"Matriz de masa"<<std::endl;
	std::cout << M << std::endl;
	
	std::cout<<"Primer elemento"<<std::endl;
	std::cout << M(1) << std::endl;

	std::cout<<"Segundo elemento"<<std::endl;
	std::cout << M(2) << std::endl;

	std::cout<<"Septimo elemento"<<std::endl;
	std::cout << M(7) << std::endl;


	while (ros::ok())
	{
		std_msgs::Int64 msg;
		msg.data = 22;
		chatter_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();

	}
	delete model;
	return 0;

}