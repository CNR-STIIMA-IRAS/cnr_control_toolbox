#include <cstdlib>
#include <ctime>
#include <iostream>

#include <gtest/gtest.h>
#include <gtest/gtest-death-test.h>

#include <ros/ros.h>
#include <eigen_matrix_utils/eigen_matrix_utils.h>
#include <state_space_systems/discrete_state_space_systems.h>
#include <state_space_systems/continuous_state_space_systems.h>
#include <state_space_systems/integral_state_space_systems.h>
#include <state_space_systems/integral_discrete_state_space_systems.h>
//#include <state_space_ros/ros_params.h>

ros::NodeHandle* nh;
constexpr unsigned int order=10; // system order
constexpr unsigned int nin=1;    // number of inputs
constexpr unsigned int nout=1;   // number of outputs

using DiscreteStateSpaceNXX = eigen_control_toolbox::DiscreteStateSpace<order,  -1,   -1>;
using DiscreteStateSpaceNNN = eigen_control_toolbox::DiscreteStateSpace<order, nin, nout>;
using DiscreteStateSpaceXXX = eigen_control_toolbox::DiscreteStateSpace<   -1,  -1,   -1>;

TEST(TestSuite, TestDiscreteStateSpaceNXX)
{

  eigen_control_toolbox::DiscreteStateSpaceArgs<order,-1,-1> args;
  args.A.resize(order,order);
  args.B.resize(order,nin);
  args.C.resize(nout,order);
  args.D.resize(nout,nin);
  
  args.A.setRandom();
  args.B.setRandom();
  args.C.setRandom();
  args.D.setRandom();
  
  // FIRST WAY TO DEFINE: consider the state with fixed-order, 
  // but the input and the output are defined online, 
  // according to the dimension of A,B,C and D.
  // NOTE: since it is in runtime, if the Input or the Output are of
  //       dimension 1, you have to use the "Eigen::VectorXd" as input
  //       to the function, or Eigen::VectorXd with the proper dimension
  EXPECT_NO_FATAL_FAILURE(DiscreteStateSpaceNXX ss(args));

  DiscreteStateSpaceNXX ss(args);
  Eigen::VectorXd u(nin);   //input vector
  Eigen::VectorXd y(nout);  //output vector
  
  u.setRandom();
  y.setRandom();
    
  EXPECT_TRUE(ss.setStateFromLastIO(u,y)); // initialize initial state value for dumpless startup
  
  EXPECT_NO_FATAL_FAILURE( y=ss.update(u) ); // computing one step, updating state and output
}

TEST(TestSuite, TestDiscreteStateSpaceNNN)
{
  eigen_control_toolbox::DiscreteStateSpaceArgs<order, nin, nout> args;
  eigen_utils::resize(args.A,order,order);
  eigen_utils::resize(args.B,order,nin);
  eigen_utils::resize(args.C,nout,order);
  eigen_utils::resize(args.D,nout,nin);

  eigen_utils::setRandom(args.A);
  eigen_utils::setRandom(args.B);
  eigen_utils::setRandom(args.C);
  eigen_utils::setRandom(args.D);

  // NOTE: since it is templated at compile time, if the Input or the Output are of
  //       dimension 1, you have to use the "double" as input
  EXPECT_NO_FATAL_FAILURE(DiscreteStateSpaceNNN ss(args));

  DiscreteStateSpaceNNN ss(args);
  double u;   //input vector
  double y;  //output vector

  EXPECT_TRUE(ss.setStateFromLastIO(u,y)); // initialize initial state value for dumpless startup
  EXPECT_NO_FATAL_FAILURE(y=ss.update(u)); // computing one step, updating state and output
}

TEST(TestSuite, TestDiscreteStateSpaceDynamicButOne)
{

  eigen_control_toolbox::DiscreteStateSpaceArgs<-1, -1, -1> args;
  eigen_utils::resize(args.A,1,1);
  eigen_utils::resize(args.B,1,1);
  eigen_utils::resize(args.C,1,1);
  eigen_utils::resize(args.D,1,1);

  eigen_utils::setRandom(args.A);
  eigen_utils::setRandom(args.B);
  eigen_utils::setRandom(args.C);
  eigen_utils::setRandom(args.D);

  // FIRST WAY TO DEFINE: consider the state with fixed-order,
  // but the input and the output are defined online,
  // according to the dimension of A,B,C and D.
  // NOTE: since it is in runtime, if the Input or the Output are of
  //       dimension 1, you have to use the "Eigen::VectorXd" as input
  //       to the function, or Eigen::VectorXd with the proper dimension
  EXPECT_NO_FATAL_FAILURE(DiscreteStateSpaceXXX ss(args));

  DiscreteStateSpaceXXX ss(args);
  Eigen::VectorXd u(1);   //input vector
  Eigen::VectorXd y(1);  //output vector

  u.setRandom();
  y.setRandom();

  EXPECT_TRUE(ss.setStateFromLastIO(u,y)); // initialize initial state value for dumpless startup

  EXPECT_NO_FATAL_FAILURE( y=ss.update(u) ); // computing one step, updating state and output
}



int main(int argc,char* argv[])
{
  // ------ Init ROS ------
  ros::init(argc,argv,"test_filters");
  
  nh = new ros::NodeHandle("~");

  srand((unsigned int) time(0));

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS(); 
}
