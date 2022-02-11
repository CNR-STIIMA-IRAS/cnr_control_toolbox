#include <chrono>  // for high_resolution_clock

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <ctime>

#include <eigen_matrix_utils/eigen_matrix_utils.h>
#include <kinematics_filters/kinematics_filters.h>
#include <gtest/gtest.h>


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
::testing::AssertionResult catch_throw(F&& f)
{
  try
  {
     f();
  }
  catch(...)
  {
    std::cerr << unwrap(std::current_exception()) << std::endl;
  }
  return ::testing::AssertionSuccess();
}


using namespace cnr_control_toolbox;

Eigen::IOFormat fmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "", "");

constexpr int stress_cycles = 1e5;
constexpr int cycles = 1000;
double natural_frequency =   50; // [rad/s] 2 pi * f
double sampling_period   = 0.001; // s

double dt = 1e-3;
double max_velocity_multiplier = 1.0;
bool   preserve_direction = true;

TEST(TestSuite, OneDoFSaturatePos)
{   
  double q_max = 10.0;
  double q_min = -10.0;
  double qd_max = 1.0;
  double qdd_max = 2.0;

  std::vector<double> qq_max = {10.0, 10.0, -2.0, -2.0, 12.0};
  std::vector<double> qq_min = {-10.0, 2.0, -10.0, -8.0, 14.0};
  for(size_t i=0;i<qq_max.size();i++)
  {
    double q_max = qq_max[i];
    double q_min = qq_min[i];

    std::vector<double> qq_target = { q_max - 2.0, q_min + 2.0, q_max + 2.0, q_min - 2.0};
    bool saturated = true;
    for(auto & q_target : qq_target)
    {
      std::cout << "-----------------------------" << std::endl;
      std::stringstream report;
      EXPECT_TRUE(catch_throw([&]{ saturated = saturatePosition(q_target,q_max, q_min, &report);}));
      std::cout << report.str();
    }
  }

}


Eigen::Matrix<double,6,1> I6 = Eigen::Matrix<double,6,1>::Ones();
Eigen::Matrix<double,6,1> R6 = Eigen::Matrix<double,6,1>::Random();

TEST(TestSuite, SixDoFSaturatePos)
{   
  
  Eigen::VectorXd q_max = I6 * 10.0;
  Eigen::Matrix<double,6,1> q_min = I6 * -10.0;
  
  std::vector<Eigen::VectorXd> qq_max = {
    10.0 * I6, 10.0 * I6, -2.0 * I6, -2. * I6, 12.0 * I6, 
    10.0 * R6, 10.0 * R6, -2.0 * R6, -2. * R6, 12.0 * R6
  };
  std::vector<Eigen::VectorXd> qq_min = {
    -10.0* I6, 2.0* I6, -10.0* I6, -8.0*I6, 14.0*I6,
    -10.0* R6, 2.0* R6, -10.0* R6, -8.0*R6, 14.0*R6
  };

  for(size_t i=0;i<qq_max.size();i++)
  {
    auto q_max = qq_max[i];
    auto q_min = qq_min[i];

    std::vector<Eigen::Matrix<double,6,1>> qq_target = { q_max - 2.0*I6, q_min + 2.0*I6, q_max + 2.0*I6, q_min - 2.0*I6};
    bool saturated = true;
    for(auto & q_target : qq_target)
    {
      std::cout << "-----------------------------" << std::endl;
      std::stringstream report;
      EXPECT_TRUE(catch_throw([&]{ saturated = saturatePosition(q_target,q_max, q_min, &report);}));
      std::cout << report.str();
    }
  }

}

/*

TEST(TestSuite, OneDoFSaturateSpeed)
{
  double qd_target = 8.0;
  bool EXPECT_TRUE(catch_throw([&]{ saturated = saturateSpeed(qd_target,qd_max,max_velocity_multiplier,preserve_direction,&report);
  EXPECT_FALSE( saturated );
  std::cout << report.str() << std::endl;

  qd_target = 12.0;
  EXPECT_TRUE(catch_throw([&]{ saturated = saturateSpeed(qd_target,qd_max,max_velocity_multiplier,preserve_direction,&report);
  EXPECT_FALSE( saturated );
  std::cout << report.str() << std::endl;
}


TEST(TestSuite, OneDoFSaturateSpeedFirstorderState)
{
  double qd_target = 0.8;
  double qd_actual = 0.0;
  double q_actual  = 0.0;
  bool EXPECT_TRUE(catch_throw([&]{ saturated = saturateSpeedFirstOrderState(qd_target,qd_actual, qd_max,qdd_max,dt,max_velocity_multiplier,preserve_direction,&report);
  EXPECT_FALSE( saturated );
  std::cout << report.str() << std::endl;

  qd_target = 8.0;
  qd_actual = 5.0;
  q_actual  = 9.5;
  EXPECT_TRUE(catch_throw([&]{ saturated = saturateSpeedFirstOrderState(qd_target,qd_actual, qd_max,qdd_max,dt,max_velocity_multiplier,preserve_direction,&report);
  EXPECT_TRUE( saturated );
  std::cout << report.str() << std::endl;
}



TEST(TestSuite, OneDoFSaturateSpeedFullOrderState)
{
  double qd_target = 0.8;
  double qd_actual = 0.0;
  double q_actual  = 0.0;
  bool EXPECT_TRUE(catch_throw([&]{ saturated = saturateSpeedFullState(qd_target,q_actual,qd_actual,q_max,q_min,qd_max,qdd_max,dt,max_velocity_multiplier,preserve_direction,&report);
  EXPECT_FALSE( saturated );
  std::cout << report.str() << std::endl;

  qd_target = 8.0;
  qd_actual = 5.0;
  q_actual  = 9.5;
  EXPECT_TRUE(catch_throw([&]{ saturated = saturateSpeedFullState(qd_target,q_actual,q_actual,q_max,q_min,qd_max,qdd_max,dt,max_velocity_multiplier,preserve_direction,&report);
  EXPECT_TRUE( saturated );
  std::cout << report.str() << std::endl;
}


*/

int main(int argc,char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS(); 
}
