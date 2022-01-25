/*
	Control library: articular PD controller
*/

// Control
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <trajectory_msgs/JointTrajectory.h>

// Ros msg
#include <my_control_gazebo/AngularPosition.h>
#include <my_control_gazebo/AngularVelocity.h>
#include <std_msgs/Int64.h>

// Console
#include <ros/console.h>

// Real time
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

// Eigen
#include <iostream>
#include <Eigen/Dense>


// RBDL 
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
using namespace Eigen;

// Vector of fixed size
typedef Matrix<double, 6, 1> Vector_d_6x1;
typedef Matrix<double, 7, 1> Vector_d_7x1;
typedef Matrix<double, 4, 1> Vector_d_4x1;
typedef Matrix<double, 4, 4> Matrix_d_4x4;
typedef Matrix<double, 7, 7> Matrix_d_7x7;
typedef Matrix<double, 7, 6> Matrix_d_7x6;
typedef Matrix<double, 6, 6> Matrix_d_6x6;

// Effort control class: PD control method 
namespace effort_controllers_ns{

	class ArticularPDController : 	public controller_interface::Controller<hardware_interface::EffortJointInterface>
	{
		private:
			// To save joint names
			std::vector< std::string > joint_names_;
			// To handle joint vector
			std::vector< hardware_interface::JointHandle > joints_;
			// Number of joints
			unsigned int n_joints_;
			// Real time buffer: angular position 
			realtime_tools::RealtimeBuffer<std::vector<double> > position_command_;
			// Real time buffer: angular velocity
			realtime_tools::RealtimeBuffer<std::vector<double> > velocity_command_;
			// Subscriber
			ros::Subscriber sub_position_command_;
			ros::Subscriber sub_velocity_command_;
			// Publisher
			ros::Publisher pub_commad_;
			ros::Publisher pub_current_position_;
			ros::Publisher pub_current_velocity_;
			// Control gains
			Vector_d_6x1 kp_;
			Vector_d_6x1 kd_;
			// RBDL
			Model* model_ = new Model();
			// Just to print
			int counter_ = 0.0;
			int p_rate_ = 100;
			// start flag
			std_msgs::Int64 f_start_;
			// current joint states
			my_control_gazebo::AngularPosition joint_position_;
			my_control_gazebo::AngularVelocity joint_velocity_;
		public:
			bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
			{
				// Read joints of robot
				std::string param_name = "joints";
				if(!n.getParam(param_name, joint_names_))
				{
					ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << n.getNamespace() << ").");
					return false;
				}

				ROS_INFO_STREAM("Success to getParam '" << param_name <<"'(namespace: "<< n.getNamespace() << "). ");
				
				// Number of DoF
				n_joints_ = joint_names_.size();

				// Add joints to hardware interface
				for(unsigned int i=0; i<n_joints_; i++)
				{
					try
					{
						joints_.push_back(hw->getHandle(joint_names_[i]));
					}
					catch (const hardware_interface::HardwareInterfaceException& e)
					{
						ROS_ERROR_STREAM("Exception thrown: " << e.what());
						return false;
					}
				}
				// resize command buffer with non real time communication
				position_command_.writeFromNonRT(std::vector<double>(6, 0.0));
				velocity_command_.writeFromNonRT(std::vector<double>(6, 0.0));

				// Load urdf model
				if (!Addons::URDFReadFromFile ("/home/jhon/catkin_ws/journal_ws/src/ur5_description/urdf/ur5_joint_limited_robot.urdf", model_, false))
                {
					/* Do nothing*/
				}

				// Subscriber
				sub_position_command_ = n.subscribe<my_control_gazebo::AngularPosition>("position_command", 1, &ArticularPDController::positionCommandCB, this);
				sub_velocity_command_ = n.subscribe<my_control_gazebo::AngularVelocity>("velocity_command", 1, &ArticularPDController::velocityCommandCB, this);

				// Publisher
				pub_commad_ = n.advertise<std_msgs::Int64>("f_start", 1);
				pub_current_position_ = n.advertise<my_control_gazebo::AngularPosition>("current_position", 1);
				pub_current_velocity_ = n.advertise<my_control_gazebo::AngularVelocity>("current_velocity", 1);
				return true;					
			}

			void positionCommandCB(const my_control_gazebo::AngularPosition::ConstPtr& msg_p){
				
				// To recieve angular position message
				std::vector<double> desired_position(6,0.0);
				// Reciving desired position message
				desired_position[0] = msg_p->q1;
				desired_position[1] = msg_p->q2;
				desired_position[2] = msg_p->q3;
				desired_position[3] = msg_p->q4;
				desired_position[4] = msg_p->q5;
				desired_position[5] = msg_p->q6;

				// Send desired angular position
				position_command_.initRT(desired_position); // ee_pose_command_ require std::vector<double>	
			}

			void velocityCommandCB(const my_control_gazebo::AngularVelocity::ConstPtr& msg_v){
				
				// To recieve angular velocity message
				std::vector<double> desired_velocity(6,0.0);
				// Reciving desired velocity message
				desired_velocity[0] = msg_v->dq1;
				desired_velocity[1] = msg_v->dq2;
				desired_velocity[2] = msg_v->dq3;
				desired_velocity[3] = msg_v->dq4;
				desired_velocity[4] = msg_v->dq5;
				desired_velocity[5] = msg_v->dq6;

				// Send desired angular velocity
				velocity_command_.initRT(desired_velocity); // ee_pose_command_ require std::vector<double>										
			}

			void starting(const ros::Time& time)
			{
				// Initial values of control gains
				for(int i=0; i<6; i++)
				{
					kp_[i] = 25;
					kd_[i] = 10;
				}


				// To recieve angular position and velocity message
				std::vector<double> desired_position(6,0.0);
				std::vector<double> desired_velocity(6,0.0);
				Vector_d_6x1 q;
				Vector_d_6x1 dq;

				// Home position: Angular Position
				q[0] = 0.22555947;
				q[1] = -2.16092376;
				q[2] = -2.13975583;
				q[3] = -0.41997402;
				q[4] =  1.53827725;
				q[5] = -1.35006513;  
				// Home position: Angular Velocity
				dq[0] = 0.0;
				dq[1] = 0.0;
				dq[2] = 0.0;
				dq[3] = 0.0;
				dq[4] = 0.0;
				dq[5] = 0.0;				

				// Reciving desired position message
				desired_position[0] = q[0];
				desired_position[1] = q[1];
				desired_position[2] = q[2];
				desired_position[3] = q[3];
				desired_position[4] = q[4];
				desired_position[5] = q[5];
				// Reciving desired velocity message
				desired_velocity[0] = dq[0];
				desired_velocity[1] = dq[1];
				desired_velocity[2] = dq[2];
				desired_velocity[3] = dq[3];
				desired_velocity[4] = dq[4];
				desired_velocity[5] = dq[5];	

				// Send desired angular position
				position_command_.initRT(desired_position); // ee_pose_command_ require std::vector<double>	
				// Send desired angular velocity
				velocity_command_.initRT(desired_velocity); // ee_pose_command_ require std::vector<double>					  						
			}

			void update(const ros::Time& time, const ros::Duration& period)
			{
				// useful vectors	
				Vector_d_6x1 q; 	// current angular position
				Vector_d_6x1 dq;	// current angular velocity

				// control terms
				Vector_d_6x1 p_error;
				Vector_d_6x1 p_term;
				Vector_d_6x1 d_error;				
				Vector_d_6x1 d_term;
				Vector_d_6x1 u;
				Vector_d_6x1 tau;
				
				// robot dynamics
				MatrixNd M = MatrixNd::Zero(6,6);	// inertia matrix
				VectorNd b = VectorNd::Zero(6);	// nonlinear effects vector


				// Get desired angular position and velocity
				std::vector<double> & des_angular_position = *position_command_.readFromRT();
				std::vector<double> & des_angular_velocity = *velocity_command_.readFromRT();

				// Get current angular position and velocity
				for(unsigned int i=0; i<n_joints_; i++)
				{										
					q[i] = joints_[i].getPosition();
					dq[i] = joints_[i].getVelocity();
				}
				
				// Control terms
				for(int i=0; i<6; i++)
				{
					// Proportional term
					p_error[i] = des_angular_position[i] - q[i];
					p_term[i]  = kp_[i]*p_error[i];
					// Derivative term
					d_error[i] = des_angular_velocity[i] - dq[i];
					d_term[i]  = kd_[i]*d_error[i];									
				}

				// Robot dynamics
				CompositeRigidBodyAlgorithm(*model_, q, M); // inertia matrix
				NonlinearEffects(*model_, q, dq, b);		// nonlinear effects vector

				// control law
				u = M*p_term + M*d_term + b;

				// safety limits
				//tau = eval_limits_control_singal(u);

				// send control signal
				for (int i = 0; i < 6; ++i)
				{	
					double effort_command = u[i];
					joints_[i].setCommand(effort_command);
				}				
				// send current position
				joint_position_.q1= q[0];
				joint_position_.q2= q[1];
				joint_position_.q3= q[2];
				joint_position_.q4= q[3];
				joint_position_.q5= q[4];
				joint_position_.q6= q[5];
				pub_current_position_.publish(joint_position_);
				// send current velocity
				joint_velocity_.dq1 = dq[0];
				joint_velocity_.dq2 = dq[1];				
				joint_velocity_.dq3 = dq[2];
				joint_velocity_.dq4 = dq[3];
				joint_velocity_.dq5 = dq[4];
				joint_velocity_.dq6 = dq[5];
				pub_current_velocity_.publish(joint_velocity_);

				// f_start = 1 indicates that control file load without problems
				// Thus, node starts publishing desired trajectory
				if (p_error.norm()<=0.002)
				{ 
					f_start_.data = 1;
				}
				pub_commad_.publish(f_start_);

				//print
				counter_ += 1;
				if (false)
				{
					if (counter_>= 50000){
						counter_ = 0;
					}
					if (counter_%p_rate_ == 0){
						std::cout<<"\n==============="<<std::endl;
						//std::cout<<"\nq_d: "<<p_error+q<<std::endl;
						std::cout<<"\nq_e: "<<p_error<<std::endl;
						std::cout<<"\ndq_e: "<<d_error<<std::endl;
						std::cout<<"\nnorm_e: "<<p_error.norm()<<std::endl;
						std::cout<<"\nnorm_de: "<<d_error.norm()<<std::endl;
						//std::cout<<"\np_term: "<<M*p_term <<std::endl;
						//std::cout<<"\nd_term: "<<M*d_term<<std::endl;
						//std::cout<<"\nb: "<<b<<std::endl;
						//std::cout<<"\nu: "<<u<<std::endl;
						
					}
				}


			}

			void stopping(const ros::Time& time){}

			// @info limits control signal to safe values
			// @param u control singal
			Vector_d_6x1 eval_limits_control_singal(Vector_d_6x1& u)
			{
				double size0 = 12*0.9;
				double size1 = 28*0.9;
				double size2 = 56*0.9;
				double size3 = 150*0.9;
				double size4 = 330*0.9;

				Vector_d_6x1 tau;


				if (std::abs(u[0]) >= size3)
				{	tau[0] = sgn(u[0])*size3;	}
				else{ tau[0] = u[0]; }

				if (std::abs(u[1]) >= size3)
				{	tau[1] = sgn(u[1])*size3;	}
				else{ tau[1] = u[1]; }

				if (std::abs(u[2]) >= size3)
				{	tau[2] = sgn(u[2])*size3;	}
				else{ tau[2] = u[2]; }

				if (std::abs(u[3]) >= size1)
				{	tau[3] = sgn(u[3])*size1;	}
				else{ tau[3] = u[3]; }

				if (std::abs(u[4]) >= size1)
				{	tau[4] = sgn(u[4])*size1;	}
				else{ tau[4] = u[4]; }

				if (std::abs(u[5]) >= size1)
				{	tau[5] = sgn(u[5])*size1;	}
				else{ tau[5] = u[5]; }	
				
				return tau;			
			}

			// @info Computes forward kinematics of ur5 robot
			// @param q         The joint values 6x1 
			// @param T         Transform matrix in row-major ordering 4x4
			Matrix_d_4x4 fkine_ur5(Vector_d_6x1& q0)
			{
			  Matrix_d_4x4 T;

			  const double d1 =  0.08916;
			  const double a2 = -0.42500;
			  const double a3 = -0.39225;
			  const double d4 =  0.10915;
			  const double d5 =  0.09465;
			  const double d6 =  0.08230;

			  double q1=q0[0], q2=q0[1], q3=q0[2], q4=q0[3], q5=q0[4], q6=q0[5];
			  double q23 = q2+q3, q234 = q2+q3+q4;

			  double s1 = sin(q1), c1 = cos(q1);
			  double s2 = sin(q2), c2 = cos(q2);
			  double s5 = sin(q5), c5 = cos(q5);
			  double s6 = sin(q6), c6 = cos(q6); 
			  double s23 = sin(q23), c23 = cos(q23);
			  double s234 = sin(q234), c234 = cos(q234);

			  T(0,0) =  c6*(c1*c5*c234 + s1*s5) - c1*s6*s234;
			  T(0,1) = -s6*(c1*c5*c234 + s1*s5) - c6*c1*s234;
			  T(0,2) = -c1*s5*c234 + s1*c5;
			  T(0,3) =  d6*(s1*c5 - c1*s5*c234) + d5*c1*s234 + d4*s1 + c1*(a3*c23 + a2*c2);

			  T(1,0) =  c6*(s1*c5*c234 - c1*s5) - s1*s6*s234;
			  T(1,1) = -s6*(s1*c5*c234 - c1*s5) - c6*s1*s234;
			  T(1,2) = -s1*s5*c234 - c1*c5;
			  T(1,3) =  d6*(-c1*c5 - s1*s5*c234) + d5*s1*s234 - d4*c1 + s1*(a3*c23 + a2*c2);

			  T(2,0) =  c5*c6*s234 + s6*c234;
			  T(2,1) = -s6*c5*s234 + c6*c234;
			  T(2,2) = -s5*s234;
			  T(2,3) = -d6*s5*s234 - d5*c234 + d1 + a3*s23 + a2*s2;  
			  
			  T(3,0) = 0.0;
			  T(3,1) = 0.0;
			  T(3,2) = 0.0;
			  T(3,3) = 1.0;

			  return T;
			}			

			// @info signed function
			int sgn(double val)  
			{
			  if (val >= 0){ return 1;}

			  else{return -1;}
			} 



			// @info Transform from homegeneous matrix to quaternions
			// @param T     Transform matrix in row-major ordering 4x4
			// @param Q     Quaternion vector 4x1
			Vector_d_4x1 rot2quat( Matrix_d_4x4& T)
			{
			  double dEpsilon = 1e-6;
			  Vector_d_4x1 Q;

			  // Q[0]
			  Q(0) = 0.5*std::sqrt(T(0,0) + T(1,1) + T(2,2) + 1);


			  // Q[1]    
			  if (std::abs(T(0,0) - T(1,1) - T(2,2) + 1) < dEpsilon)
			  { Q(1) = 0.0; }
			  else { Q(1) = 0.5*sgn(T(2,1) - T(1,2))*std::sqrt(T(0,0) - T(1,1) - T(2,2) + 1); }

			  // Q[2]    
			  if (std::abs(T(1,1) - T(2,2) - T(0,0) + 1) < dEpsilon)
			  { Q(2) = 0.0; }
			  else
			  { Q(2) = 0.5*sgn(T(0,2) - T(2,0))*std::sqrt(T(1,1) - T(2,2) - T(0,0) + 1); }  

			  // Q[3]    
			  if (std::abs(T(2,2) - T(0,0) - T(1,1) + 1) < dEpsilon)
			  { Q(3) = 0.0; }
			  else
			  { Q(3) = 0.5*sgn(T(1,0) - T(0,1))*std::sqrt(T(2,2) - T(0,0) - T(1,1) + 1); }  

			  return Q;
			}



			//@info compute difference between quaternions
			//@param Qdes   Desired quaterion   4x1
			//@param Q      Measured quaternion 4x1
			Vector_d_4x1 quatError( Vector_d_4x1& Qdes, Vector_d_4x1& Q)
			{
			  Vector_d_4x1 Qerror;
			  Qerror(0) =  Qdes(0)*Q(0) + Qdes(1)*Q(1) + Qdes(2)*Q(2) + Qdes(3)*Q(3) - 1;   
			  Qerror(1) = -Qdes(0)*Q(1) + Q(0)*Qdes(1) - (Qdes(2)*Q(3) - Qdes(3)*Q(2)); 
			  Qerror(2) = -Qdes(0)*Q(2) + Q(0)*Qdes(2) + (Qdes(1)*Q(3) - Qdes(3)*Q(1));
			  Qerror(3) = -Qdes(0)*Q(3) + Q(0)*Qdes(3) - (Qdes(1)*Q(2) - Qdes(2)*Q(1));

			  return Qerror;
			}



			//@info compute jacobian pose for [x y z w ex ey ez]
			//@param q      Joint values  6x1
			//@param J      Jacobian matrix 7x6
			Matrix_d_7x6 jacobianPose( Vector_d_6x1& q)
			{
			  Matrix_d_7x6 J;
			  Matrix_d_7x6 dJ;
			  Matrix_d_4x4 T;
			  Matrix_d_4x4 dT;
			  Vector_d_4x1 Q;
			  Vector_d_4x1 dQ;
			  Vector_d_4x1 Qe;

			  Vector_d_6x1 dq;
			  double delta = 0.00001; 
			  
			  // Compute T that belongs to q
			  T = fkine_ur5(q);
			  // Represent orietantion with quaternions
			  Q = rot2quat(T); 
			  
			  for (int i = 0; i < 6; ++i)
			  {
			    dq     = q;
			    dq(i)  = dq(i) + delta;
			    dT     = fkine_ur5(dq);
			    dQ     = rot2quat(dT);
			    Qe     = quatError(dQ, Q); // Quat error
			    J(0,i) = dT(0,3) - T(0, 3); // x
			    J(1,i) = dT(1,3) - T(1, 3); // y 
			    J(2,i) = dT(2,3) - T(2, 3); // z  
			    J(3,i) = Qe(0);             // w
			    J(4,i) = Qe(1);             // ex
			    J(5,i) = Qe(2);             // ey
			    J(6,i) = Qe(3);             // ez      
			  }
			  J = J/delta;
			  return J;
			}			

	};
	// export control library (very important!)
	PLUGINLIB_EXPORT_CLASS(effort_controllers_ns::ArticularPDController, controller_interface::ControllerBase);
}