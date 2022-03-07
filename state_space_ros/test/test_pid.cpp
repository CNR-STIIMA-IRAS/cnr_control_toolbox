#include <iostream>
#include <cstdlib>
#include <ctime>

#include <gtest/gtest.h>

#include <ros/ros.h>
#include <eigen_matrix_utils/eigen_matrix_utils.h>
#include <state_space_controllers/controllers.h>
#include <state_space_ros/ros_params.h>

namespace ect = eigen_control_toolbox;
using ControllerX = ect::Controller<-1>;

ros::NodeHandle* nh;

namespace detail
{
  struct unwrapper
  {
    unwrapper(std::exception_ptr pe) : pe_(pe) {}

    operator bool() const { return bool(pe_);}

    friend auto operator<<(std::ostream& os, unwrapper const& u) -> std::ostream&
    {
      try
      {
          std::rethrow_exception(u.pe_);
          return os << "no exception";
      }
      catch(std::runtime_error const& e) { return os << "runtime_error: " << e.what();}
      catch(std::logic_error const& e)   { return os << "logic_error: " << e.what();}
      catch(std::exception const& e)     { return os << "exception: " << e.what();}
      catch(...)      {return os << "non-standard exception";}
    }
    std::exception_ptr pe_;
  };
}

auto unwrap(std::exception_ptr pe)
{
  return detail::unwrapper(pe);
}

template<class F>
::testing::AssertionResult does_not_throw(F&& f)
{
  try
  {
     f();
     return ::testing::AssertionSuccess();
  }
  catch(...)
  {
     return ::testing::AssertionFailure() << unwrap(std::current_exception());
  }
}

// Declare a test
TEST(TestSuite, ProportionalController)
{
  int ret=0;
  std::string what;
  EXPECT_NO_FATAL_FAILURE(ControllerX proportional);
  ControllerX proportional;
  
  EXPECT_TRUE(does_not_throw([&]{ret = ect::setMatricesFromParam<-1>(proportional,*nh,"/ctrl1", what);}));
  ROS_WARN_COND(ret==0, "Failing initializing controller ctrl1: %s", what.c_str());
  ROS_ERROR_COND(ret==-1, "Failing initializing controller ctrl1: %s", what.c_str());

  ROS_INFO("ctrl1:");
  EXPECT_TRUE(does_not_throw([&]{std::cout << proportional << std::endl;}));

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
  EXPECT_TRUE(does_not_throw([&]{ret = ect::setMatricesFromParam<-1>(pi,*nh,"/ctrl2", what);}));
  ROS_WARN_COND(ret==0, "Failing initializing controller ctrl12: %s", what.c_str());
  ROS_ERROR_COND(ret==-1, "Failing initializing controller ctrl2: %s", what.c_str());

  ROS_INFO("ctrl2:");
  EXPECT_TRUE(does_not_throw([&]{std::cout << pi << std::endl; }));

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


// Declare a test
TEST(TestSuite, ControllerSS)
{
  int ret=0;
  std::string what;
  EXPECT_NO_FATAL_FAILURE(ControllerX ctrl_ss);
  ControllerX ctrl_ss;

  EXPECT_TRUE(does_not_throw([&]{ret = ect::setMatricesFromParam<-1>(ctrl_ss,*nh,"/ss", what);}));
  ROS_WARN_COND(ret, "Warning(s) raised while initializing the controller ss: %s", what.c_str());

  ROS_INFO("ctrl_ss:");
  EXPECT_TRUE(does_not_throw([&]{std::cout << ctrl_ss << std::endl;}));
  for (unsigned int idx=0;idx<200;idx++)
  {
    ControllerX::Input controller_input;
    ControllerX::Output p_output;
    ControllerX::State  state;
    EXPECT_NO_FATAL_FAILURE(eigen_utils::resize(controller_input,ctrl_ss.uDim()) );
    EXPECT_NO_FATAL_FAILURE(eigen_utils::setConstant(controller_input, 1));

    EXPECT_NO_FATAL_FAILURE(p_output = ctrl_ss.update(controller_input));
    EXPECT_NO_FATAL_FAILURE(state = ctrl_ss.x());
    std::cout << "controller_input=" << eigen_utils::to_string(controller_input)
              << ", output=" << eigen_utils::to_string(p_output)
              << ", state=" << eigen_utils::to_string(state)
              << std::endl;
  }

}

int main(int argc,char* argv[])
{
  // ------ Init ROS ------
  ros::init(argc,argv,"test_pid");
  nh = new ros::NodeHandle();

  srand((unsigned int) time(0));

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS(); 

  return 0; 
}
