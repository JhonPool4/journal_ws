// Control
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <trajectory_msgs/JointTrajectory.h>

// Ros msg
#include <my_control_gazebo/Pose.h>
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
#ifndef RBDL_BUILD_ADDON_URDFREADER
	#error "Error: RBDL addon URDFReader not enabled."
#endif

#include <rbdl/addons/urdfreader/urdfreader.h>
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
using namespace Eigen;

// vector of fixed size
typedef Matrix<double, 6, 1> Vector_d_6x1;
typedef Matrix<double, 7, 1> Vector_d_7x1;
typedef Matrix<double, 4, 1> Vector_d_4x1;
typedef Matrix<double, 4, 4> Matrix_d_4x4;
typedef Matrix<double, 7, 7> Matrix_d_7x7;
typedef Matrix<double, 7, 6> Matrix_d_7x6;
typedef Matrix<double, 6, 6> Matrix_d_6x6;

// effort control class: Impendance control method 
namespace effort_controllers_ns{

	class CartesianImpedanceController : 	public controller_interface::Controller<hardware_interface::EffortJointInterface>
	{
		private:
			// To save joint names
			std::vector< std::string > joint_names_;
			// To handle joint vector
			std::vector< hardware_interface::JointHandle > joints_;
			// Number of joints
			unsigned int n_joints_;
			// Real time buffer: end-effector pose
			realtime_tools::RealtimeBuffer<std::vector<double> > ee_pose_command_;
			// Subscriber
			ros::Subscriber sub_command_;
			// Publisher
			ros::Publisher pub_commad_;
			// RBDL
			Model* model_ = new Model();
			// Just to print
			int counter_ = 0.0;
			int p_rate_ = 100;

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
				ee_pose_command_.writeFromNonRT(std::vector<double>(7, 0.0));

				// Load urdf model
				if (!Addons::URDFReadFromFile ("/home/utec/catkin_ws/src/ur5_description/urdf/ur5_joint_limited_robot.urdf", model_, false)) 
				{
					/* Do nothing*/
				}

				// Subscriber
				sub_command_ = n.subscribe<my_control_gazebo::Pose>("command", 1, &CartesianImpedanceController::commandCB, this);

				// Publisher
				pub_commad_ = n.advertise<std_msgs::Int64>("f_start", 1);

				return true;					
			}

			void commandCB(const my_control_gazebo::Pose::ConstPtr& msg){
				
				// To recieve end-effector pose
				std::vector<double> ee_pose(7,0.0);

				ee_pose[0] = msg->x;
				ee_pose[1] = msg->y;
				ee_pose[2] = msg->z;
				ee_pose[3] = msg->w;
				ee_pose[4] = msg->ex;
				ee_pose[5] = msg->ey;
				ee_pose[6] = msg->ez;

				ee_pose_command_.initRT(ee_pose); // ee_pose_command_ require std::vector<double>						

			}
			void starting(const ros::Time& time)
			{
				std::vector<double> ee_pose(7,0.0);
				Vector_d_6x1 q;
				Matrix_d_4x4 T;
				Vector_d_4x1 Q;

				// Read current position of the joints
				/*
				for (std::size_t i = 0; i < n_joints_; ++i)
				{	q[i] = joints_[i].getPosition();	}
				*/
				// Home position
				q[0] =  3.14;
				q[1] = -2.17330664;
				q[2] = -2.11274865;
				q[3] = -0.42801517;
				q[4] =  1.53728324;
				q[5] = -1.55185632;  				

				// Transform from articular to Cartesian space
				T = fkine_ur5(q); // forward kinematics
				Q = rot2quat(T);  // from rotational matrix to quaternions
				
				ee_pose[0] = T(0,3);
				ee_pose[1] = T(1,3);
				ee_pose[2] = T(2,3);
				ee_pose[3] = Q[0];
				ee_pose[4] = Q[1];
				ee_pose[5] = Q[2];
				ee_pose[6] = Q[3];	

				ee_pose_command_.initRT(ee_pose); // ee_pose_command_ require std::vector<double>					  						
			}

			void update(const ros::Time& time, const ros::Duration& period)
			{
				// f_start = 1 indicates that control file is ok
				std_msgs::Int64 f_start;
				f_start.data = 1;
				pub_commad_.publish(f_start);

				// useful vectors	
				Vector_d_7x1 des_ee_pose_eigen;
				Vector_d_7x1 med_ee_d_pose_eigen;
				Vector_d_6x1 q;
				Vector_d_6x1 dq;
				Matrix_d_4x4 T;
				Vector_d_4x1 Q;		
				Vector_d_4x1 Qd;
				Vector_d_4x1 Qe;
				Matrix_d_7x6 J;
				Matrix_d_7x6 M_temp;	

				Vector_d_7x1 p_term;
				Vector_d_7x1 d_term;
				Vector_d_7x1 error_pose;

				MatrixNd M = MatrixNd::Zero(6,6);
				Matrix_d_7x7 Mx;

				Vector_d_7x1 F;
				Vector_d_6x1 u;
				Vector_d_6x1 tau;

				typedef Matrix<double, Dynamic, Dynamic> MatrixX_ld;
				MatrixX_ld  Jld(7,6);
				MatrixX_ld  Jinv(6,7);

				// Desired end-effector pose
				std::vector<double> & des_ee_pose = *ee_pose_command_.readFromRT();

				for (int i = 0; i < 7; ++i)
				{
					des_ee_pose_eigen[i] = des_ee_pose[i];
				}
				// Read current position of the joints
				for(unsigned int i=0; i<n_joints_; i++)
				{										
					q(i) = joints_[i].getPosition();
					dq(i) = joints_[i].getVelocity();
				}
				
				// Desired pose
				T = fkine_ur5(q);
				Q = rot2quat(T);
				Qd(0) = des_ee_pose_eigen(3);
				Qd(1) = des_ee_pose_eigen(4);
				Qd(2) = des_ee_pose_eigen(5);
				Qd(3) = des_ee_pose_eigen(6); 
				Qe = quatError(Qd, Q);

				// Proportional term
				p_term(0) = des_ee_pose_eigen(0) - T(0,3);
				p_term(1) = des_ee_pose_eigen(1) - T(1,3);
				p_term(2) = des_ee_pose_eigen(2) - T(2,3);
				p_term(3) = Qe(0);
				p_term(4) = Qe(1);
				p_term(5) = Qe(2);
				p_term(6) = Qe(3);

				error_pose = p_term;


				// Funciona bien: No hay vibraciones, ligero error en orientación y posición
				// xe[0:3] = -3.75941    10.895    10.342  [mm]
				// xe[3:7] = -0.00151685   0.0235369   0.0497442  0.00170838  // quaternion
				// KD = 200 200 800 40 40 40
				// BD = 30  30  60  15  15  15
				p_term(0) = 200*p_term(0); 
				p_term(1) = 200*p_term(1);
				p_term(2) = 800*p_term(2);
				p_term(3) =  40*p_term(3);
				p_term(4) =  40*p_term(4);
				p_term(5) =  40*p_term(5);
				p_term(6) =  40*p_term(6);


				// Derivative Pose
				J    = jacobianPose(q);  // Jacobian
				Jld  = J;
				Jinv = Jld.completeOrthogonalDecomposition().pseudoInverse(); // Jinv

				med_ee_d_pose_eigen = J*dq;

				// Derivative term
				d_term(0) =  -30*med_ee_d_pose_eigen(0);
				d_term(1) =  -30*med_ee_d_pose_eigen(1);
				d_term(2) =  -60*med_ee_d_pose_eigen(2);
				d_term(3) =  -15*med_ee_d_pose_eigen(3);
				d_term(4) =  -15*med_ee_d_pose_eigen(4);
				d_term(5) =  -15*med_ee_d_pose_eigen(5);
				d_term(6) =  -15*med_ee_d_pose_eigen(6);

				// y = p_term + d_term
				p_term = p_term + d_term;				

				// M_x = Jinv^T * M * Jinv
				CompositeRigidBodyAlgorithm(*model_, q, M);
				M_temp = Jinv.transpose()*M; 
				Mx = M_temp*Jinv;

				// effort signal
				F = Mx*p_term;
				u = J.transpose()*F; 
				tau = eval_limits_control_singal(u);


				// Norm of x y z
				//double norm_e   = sqrt(error_pose.array().square().sum());
				double norm_ori = sqrt(error_pose(3)*error_pose(3)+error_pose(4)*error_pose(4)+error_pose(5)*error_pose(5)+
									   error_pose(6)*error_pose(6));
				double norm_xyz = sqrt(error_pose(0)*error_pose(0)+error_pose(1)*error_pose(1)+error_pose(2)*error_pose(2));

				/*
				if (counter_ <= 10)
				{
					std::cout<<"\nx pose:\n"<<des_ee_pose_eigen.transpose()<<std::endl;
					std::cout<<"\nxe term:\n"<<error_pose.transpose()<<std::endl;
					//std::cout<<"\nJ mat: \n"<<J<<std::endl;
					//std::cout<<"\nJinv mat: \n"<<Jinv<<std::endl;
					//std::cout<<"\nM mat: \n"<<M<<std::endl;
					std::cout<<"\nMx mat: \n"<<Mx<<std::endl;
					std::cout<<"\nq term: \n"<<q.transpose()<<std::endl;
					//std::cout<<"\ntau term:\n"<<tau.transpose()<<std::endl;
					std::cout<<"----------------------------------------";
					std::cout<<"\n\n";
				}
				*/
				// To print position and orientation error
				counter_ += 1;
				if (counter_>= 50000){
					counter_ = 0;
				}
				if (counter_%p_rate_ == 0){
					std::cout<<"\nnorm_xyz: "<<norm_xyz<<"\tnorm_ori: "<<norm_ori<<std::endl;
				}
								
				// send control signal
				for (int i = 0; i < 6; ++i)
				{	
					double effort_command = tau[i];
					joints_[i].setCommand(effort_command);
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
	PLUGINLIB_EXPORT_CLASS(effort_controllers_ns::CartesianImpedanceController, controller_interface::ControllerBase);
}