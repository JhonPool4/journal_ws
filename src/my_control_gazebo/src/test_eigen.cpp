// ROS
#include <ros/ros.h>
// EIGEN
#include <iostream>
#include <Eigen/Dense>
//RBDL
#include <rbdl/rbdl.h>
#ifndef RBDL_BUILD_ADDON_URDFREADER
  #error "Error: RBDL addon URDFReader not enabled."
#endif

#include <rbdl/addons/urdfreader/urdfreader.h>
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

using namespace Eigen;

typedef Matrix<double, 6, 1> Vector_d_6x1;
typedef Matrix<double, 4, 1> Vector_d_4x1;
typedef Matrix<double, 4, 4> Matrix_d_4x4;
typedef Matrix<double, 7, 6> Matrix_d_7x6;
typedef Matrix<double, 6, 6> Matrix_d_6x6;
typedef Matrix<double, Dynamic, Dynamic> MatrixX_ld;


// Functions
Matrix_d_4x4 fkine_ur5( Vector_d_6x1& q0);
int sgn(double val);
Vector_d_4x1 rot2quat( Matrix_d_4x4& T);
Vector_d_4x1 quatError( Vector_d_4x1& Qdes, Vector_d_4x1& Q);
Matrix_d_7x6 jacobianPose( Vector_d_6x1& q);


int main()
{
  Vector_d_6x1 q;
  Matrix_d_4x4 T;
  Vector_d_4x1 Q;
  Matrix_d_7x6 J;
  MatrixX_ld  Jld(7,6);
  MatrixX_ld  Jinv(6,7);

  q[0] =  0.02387449;
  q[1] = -2.17330664;
  q[2] = -2.11274865;
  q[3] = -0.42801517;
  q[4] =  1.53728324;
  q[5] = -1.55185632;  

  T = fkine_ur5(q);
  Q = rot2quat(T);
  J = jacobianPose(q);

  Jld = J;


  Model* model = new Model();
  if (!Addons::URDFReadFromFile ("/home/utec/catkin_ws/src/ur5_description/urdf/ur5_joint_limited_robot.urdf", model, false)) {
    std::cerr << "Error loading model ./samplemodel.urdf" << std::endl;
    abort();
  }

  MatrixNd M = MatrixNd::Zero(6,6);
  CompositeRigidBodyAlgorithm(*model, q, M);    

  Jinv = Jld.completeOrthogonalDecomposition().pseudoInverse();

  std::cout<< "Matrix M:\n"<<M<<std::endl;
  std::cout<< "Matrix Jld:\n"<< Jld << std::endl;
  std::cout<< "\nInverse Jacobian:\n"<<Jinv<<std::endl;
  std::cout<< "\nJinv^T*M*Jinv:\n"<<Jinv.transpose()*M*Jinv<<std::endl;
}



// @brief Computes forward kinematics of ur5 robot
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



// @brief signed function
int sgn(double val)  
{
  if (val >= 0){ return 1;}

  else{return -1;}
} 



// @brief Transform from homegeneous matrix to quaternions
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

//@brief compute difference between quaternions
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



//@brief compute jacobian pose for [x y z w ex ey ez]
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
  double delta = 0.0001; 
  
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


