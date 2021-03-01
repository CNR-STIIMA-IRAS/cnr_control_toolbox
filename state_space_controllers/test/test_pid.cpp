#include <iostream>
#include <cstdlib>
#include <ctime>

#include <gtest/gtest.h>

#include <ros/ros.h>
#include <eigen_matrix_utils/eigen_matrix_utils.h>
#include <state_space_controllers/controllers.h>
//#include <state_space_systems/ros_params.h>

using ControllerX = eigen_control_toolbox::Controller<-1>;

ros::NodeHandle* nh;

// Declare a test
TEST(TestSuite, ProportionalController)
{
  int ret=-1;
  std::string what;
  EXPECT_NO_FATAL_FAILURE(ControllerX proportional);
  ControllerX proportional;
  
  //EXPECT_NO_FATAL_FAILURE(ret = eigen_control_toolbox::setMatricesFromParam<-1>(proportional,*nh,"/ctrl1", what) );
  EXPECT_TRUE(ret>=0);
  if (ret==0)
  {
    ROS_WARN("Failing initializing controller ctrl1: %s", what.c_str());
  }
  else if(ret==-1)
  {
    ROS_ERROR("Failing initializing controller ctrl1: %s", what.c_str());
  }

  ROS_INFO("ctrl1:");
  EXPECT_NO_FATAL_FAILURE(std::cout << proportional << std::endl; );

  for (unsigned int idx=0;idx<200;idx++)
  {
    ControllerX::Input controller_input;
    ControllerX::Output p_output;
    ControllerX::State  state;
    EXPECT_NO_FATAL_FAILURE(eigen_utils::resize(controller_input,proportional.uDim()) );
    EXPECT_NO_FATAL_FAILURE(eigen_utils::setConstant(controller_input, 1));

    EXPECT_NO_FATAL_FAILURE(p_output = proportional.update(controller_input));
    EXPECT_NO_FATAL_FAILURE(state = proportional.x());
    std::cout << "controller_input=" << eigen_utils::to_string(controller_input) 
              << ", output=" << eigen_utils::to_string(p_output) 
              << ", state=" << eigen_utils::to_string(state) 
              << std::endl;
  }

}


// Declare a test
TEST(TestSuite, ProportionalIntegralController)
{
  int ret=-1;
  std::string what;
  ControllerX pi;
  //EXPECT_NO_FATAL_FAILURE(ret = eigen_control_toolbox::setMatricesFromParam<-1>(pi,*nh,"/ctrl2", what));
  EXPECT_TRUE(ret>=0);
  if (ret==0)
  {
    ROS_WARN("Failing initializing controller ctrl2: %s", what.c_str());
  }
  else if(ret==-1)
  {
    ROS_ERROR("Failing initializing controller ctrl2: %s", what.c_str());
  }

  ROS_INFO("ctrl2:");
  EXPECT_NO_FATAL_FAILURE(std::cout << pi << std::endl; );

  for (unsigned int idx=0;idx<200;idx++)
  {
    ControllerX::Input  controller_input;
    ControllerX::Output pi_output;
    ControllerX::Output pi_sat;
    EXPECT_NO_FATAL_FAILURE(eigen_utils::resize(controller_input,pi.uDim()));
    EXPECT_NO_FATAL_FAILURE(eigen_utils::setConstant(controller_input, 1));

    EXPECT_NO_FATAL_FAILURE(pi_output= pi.update(controller_input));
    EXPECT_NO_FATAL_FAILURE(pi_sat   = pi_output);
    EXPECT_NO_FATAL_FAILURE(eigen_utils::saturate(pi_sat,-2.0,2.0));
    EXPECT_NO_FATAL_FAILURE(pi.antiwindup(pi_sat,pi_output));
    std::cout << "controller_input=" << eigen_utils::to_string(controller_input) 
              << ", output=" << eigen_utils::to_string(pi_output)
              << ", saturated output=" << eigen_utils::to_string(pi_sat) << std::endl;
   }
}


int main(int argc,char** argv)
{
  // ------ Init ROS ------
  ros::init(argc,&*argv,"test_pid");
  nh = new ros::NodeHandle();

  srand((unsigned int) time(0));

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS(); 

  return 0; 
}
