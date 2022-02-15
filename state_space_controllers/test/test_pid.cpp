#include <iostream>
#include <cstdlib>
#include <ctime>

#include <gtest/gtest.h>

#include <eigen_matrix_utils/eigen_matrix_utils.h>
#include <state_space_controllers/controllers.h>

using ControllerX = eigen_control_toolbox::Controller<-1>;
using ControllerXStateSpaceArgs = eigen_control_toolbox::ControllerStateSpaceArgs<-1>;


// Declare a test
TEST(TestSuite, ProportionalController)
{
  bool ret = false;
  std::string what;
  EXPECT_NO_FATAL_FAILURE(ControllerX proportional);
  ControllerX proportional;
  ControllerXStateSpaceArgs a_ok, a_notok; 
  /*
  <rosparam>
  ss:
    A:
    - [0, 1]
    - [0, 0]
    B:
    - [0]
    - [1]
    C:
    - [1, 0]
    D:
    - [0]

  filter:
    frequency: 5 # [Hz]
    sampling_period: 0.01 # [s]


  ctrl1: {type: "proportional", proportional_gain: 5.0}
  ctrl2: {type: "PI", proportional_gain: 1, integral_gain: 1, sample_period: 8.0e-3}
  </rosparam>
  */

  a_ok.A.resize(2,2);
  a_ok.B.resize(2,1);
  a_ok.C.resize(1,2);
  a_ok.D.resize(1,1);
  std::cout << __LINE__ << std::endl; a_ok.A << 0, 1, 0, 0;
  std::cout << __LINE__ << std::endl; a_ok.B << 0, 1;
  std::cout << __LINE__ << std::endl; a_ok.C << 1, 0;
  std::cout << __LINE__ << std::endl; a_ok.D << 0;
  EXPECT_TRUE( proportional.setMatrices(a_ok,what) );

  a_notok.A.resize(3,3);
  a_notok.B.resize(2,1);
  a_notok.C.resize(1,2);
  a_notok.D.resize(1,1);
  std::cout << __LINE__ << std::endl; a_notok.A << 0, 1, 0, 
               0, 0, 0, 0, 0, 0;
  std::cout << __LINE__ << std::endl; a_notok.B << 0, 
               1;
  std::cout << __LINE__ << std::endl; a_notok.C << 1, 0;
  std::cout << __LINE__ << std::endl; a_notok.D << 0;

  EXPECT_FALSE( proportional.setMatrices(a_notok,what) );
  
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
  Eigen::Matrix<double,2,2> Kp_ok; Kp_ok << 0,1,1,0;
  Eigen::Matrix<double,2,2> Ki_ok; Ki_ok << 0,1,1,0;
  Eigen::Matrix<double,3,3> Kp_notok; Kp_notok << 0,1,1,0,0,0,0,0,0;
  Eigen::Matrix<double,2,2> Ki_notok; Ki_notok << 0,1,1,0;

  double sampling_period = 0.001; 

  EXPECT_TRUE( pi.setPI(Kp_ok, Ki_ok, sampling_period, what) );
  EXPECT_FALSE( pi.setPI(Kp_notok, Ki_notok, sampling_period, what) );

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


int main(int argc,char* argv[])
{ 
  srand((unsigned int) time(0));

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS(); 

  return 0; 
}
