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
    explicit unwrapper(std::exception_ptr pe) : pe_(pe) {}

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
std::vector<double> max_velocity_multipliers = {1.0, 2.0};
std::vector<bool> preserve_directions = {true, false} ;
Eigen::Matrix<double,6,1> I6 = Eigen::Matrix<double,6,1>::Ones();
Eigen::Matrix<double,6,1> R6 = Eigen::Matrix<double,6,1>::Random();

TEST(TestSuite, OneDoFSaturatePos)
{   
  double qd_max = 1.0;
  double qdd_max = 2.0;

  std::vector<double> qq_max = {10.0, 10.0, -2.0, -2.0, 12.0};
  std::vector<double> qq_min = {-10.0, 2.0, -10.0, -8.0, 14.0};
  for(size_t i=0;i<qq_max.size();i++)
  {
    double q_max = qq_max[i];
    double q_min = qq_min[i];

    std::vector<double> qq = { q_max - 2.0, q_min + 2.0, q_max + 2.0, q_min - 2.0};
    for(auto & q : qq)
    {
      bool saturated = true;
      EXPECT_TRUE(catch_throw([&]{ saturated = saturatePosition(q,q_max, q_min, nullptr);}));
    }
  }

}

TEST(TestSuite, SixDoFSaturatePos)
{   
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

    std::vector<Eigen::Matrix<double,6,1>> qq = { q_max - 2.0*I6, q_min + 2.0*I6, q_max + 2.0*I6, q_min - 2.0*I6};
    for(auto & q : qq)
    {
      bool saturated = true;
      EXPECT_TRUE(catch_throw([&]{ saturated = saturatePosition(q,q_max, q_min, nullptr);}));
    }
  }
}



TEST(TestSuite, OneDoFSaturateVel)
{   
  std::vector<double> qqd_max{10.0, 10.0, -2.0, -2.0, 12.0};
  for(const auto & qd_max : qqd_max)
  {
    std::vector<double> qqd{ qd_max - 2.0, qd_max + 2.0};
    bool saturated = true;
    for(const double & max_velocity_multiplier : max_velocity_multipliers)
    {
      for(const bool & preserve_direction : preserve_directions )
      {
        for(auto & qd : qqd)
        {
          const double _qd = qd;
          std::stringstream report;
          EXPECT_TRUE(catch_throw([&]{ saturated = saturateSpeed(qd,qd_max, max_velocity_multiplier, preserve_direction, nullptr);}));
          if(saturated) 
          {
            std::cout << report.str();
          }
          else
          {
            std::cout << "CHECK: " << std::fabs(qd - qd)<< " - " << (std::fabs(qd - qd)==0) << std::endl;;
          }
        }
      }
    }
  }
}


TEST(TestSuite, SixDoFSaturateVel)
{   
   
  std::vector<Eigen::VectorXd> qqd_max = {
    10.0 * I6, 10.0 * I6, -2.0 * I6, -2. * I6, 12.0 * I6, 
    10.0 * R6, 10.0 * R6, -2.0 * R6, -2. * R6, 12.0 * R6
  };

  for(size_t i=0;i<qqd_max.size();i++)
  {
    auto qd_max = qqd_max[i];

    std::vector<Eigen::Matrix<double,6,1>> qqd = { qd_max - 2.0*I6, qd_max + 2.0*I6};
    bool saturated = true;
    for(const double & max_velocity_multiplier : max_velocity_multipliers)
    {
      for(const bool & preserve_direction : preserve_directions )
      {
        for(auto & qd : qqd)
        {
          const auto _qd = qd;
          EXPECT_TRUE(catch_throw([&]{ saturated = saturateSpeed(qd,qd_max, max_velocity_multiplier, preserve_direction, nullptr);}));
          if(!saturated) 
          {
            std::cout << "CHECK: " << (qd - qd).maxCoeff() << " - " << ((qd - qd).maxCoeff() ==0) << std::endl;;
          }
        }
      }
    }
  }
}


TEST(TestSuite, OneDoFSaturateVelFirstOrder)
{   
  std::vector<double> qqd_max{10.0, 10.0, -2.0, -2.0, 12.0};
  std::vector<double> qqdd_max{100.0, 100.0, -20.0, -20.0, 120.0};
  for(const auto & qd_max : qqd_max)
  {
    for(const auto & qdd_max : qqdd_max)
    {
      std::vector<double> qqd{ qd_max - 2.0, qd_max + 2.0};
      std::vector<double> qqd_prev{ 0, qd_max - 1.0, qd_max - 2.0, qd_max - 10.0, -qd_max +1.0, -qd_max + 2.0, qd_max + 10.0};
      bool saturated = true;
      for(const double & max_velocity_multiplier : max_velocity_multipliers)
      {
        for(const bool & preserve_direction : preserve_directions )
        {
          for(auto & qd_prev : qqd_prev)
          {
            for(auto & qd : qqd)
            {
              const double _qd = qd;
                EXPECT_TRUE(catch_throw([&]{ saturated = saturateSpeed(qd,
                  qd_prev, qd_max, qdd_max, dt, max_velocity_multiplier, preserve_direction, nullptr);}));
              if(!saturated) 
              {
                std::cout << "CHECK: " << std::fabs(qd - qd)<< " - " << (std::fabs(qd - qd)==0) << std::endl;;
              }
            }
          }
        }
      }
    }
  }
}




TEST(TestSuite, SixDoFSaturateVelFirstOrder)
{   
  std::vector<Eigen::VectorXd> qqd_max = {
    10.0 * I6, 10.0 * I6, 12.0 * I6, 
    10.0 * R6.cwiseAbs(), 10.0 * R6.cwiseAbs(), 12.0 * R6.cwiseAbs()
  };
  std::vector<Eigen::VectorXd> qqdd_max = {
    100.0 * I6, 100.0 * I6, 120.0 * I6, 
    100.0 * R6.cwiseAbs(), 100.0 * R6.cwiseAbs(), 120.0 * R6.cwiseAbs()
  };
  for(const auto & qd_max : qqd_max)
  {
    for(const auto & qdd_max : qqdd_max)
    {
      std::vector<Eigen::Matrix<double,6,1>> qqd = { qd_max - 2.0*I6, qd_max + 2.0*I6};
      std::vector<Eigen::Matrix<double,6,1>> qqd_prev = { 0 * I6, qd_max - 1.0*I6, 
          qd_max - 2.0*I6, qd_max - 10.0*I6, -qd_max +1.0*I6, -qd_max + 2.0*I6, qd_max + 10.0*I6};
      bool saturated = true;
      for(const double & max_velocity_multiplier : max_velocity_multipliers)
      {
        for(const bool & preserve_direction : preserve_directions )
        {
          for(auto & qd_prev : qqd_prev)
          {
            for(auto & qd : qqd)
            {
              const auto _qd = qd;
              EXPECT_TRUE(catch_throw([&]{ saturated = saturateSpeed(qd,
                  qd_prev, qd_max, qdd_max, dt, max_velocity_multiplier, preserve_direction, nullptr);}));
              if(!saturated) 
              {
                std::cout << "CHECK: " << eigen_utils::to_string(qd - qd)<< std::endl;;
              }
            }
          }
        }
      }
    }
  }
}

TEST(TestSuite, OneDoFSaturateVelFullState)
{   
  std::vector<double> qq_max = {10.0, 10.0, -2.};
  std::vector<double> qq_min = {-10.0, 2.0, -10.0};
  std::vector<double> qq_prev = {-9.0, 3.0, -8.0};

  std::vector<double> qqd_max{10.0, 10.0, 12.0};
  std::vector<double> qqdd_max{100.0, 100.0, 120.0};
  for(const auto & q_max : qq_max)
    for(const auto & q_min : qq_min)
      for(const auto & q_prev : qq_prev)
        for(const auto & qd_max : qqd_max)      
          for(const auto & qdd_max : qqdd_max)
          {
            std::vector<double> qqd{ qd_max - 2.0, qd_max + 2.0};
            std::vector<double> qqd_prev{ 0, qd_max - 1.0, qd_max - 2.0, qd_max - 10.0, -qd_max +1.0, -qd_max + 2.0, qd_max + 10.0};
            bool saturated = true;
            for(const double & max_velocity_multiplier : max_velocity_multipliers)
            {
              for(const bool & preserve_direction : preserve_directions )
              {
                for(auto & qd_prev : qqd_prev)
                {
                  for(auto & qd : qqd)
                  {
                    const double _qd = qd;
                    EXPECT_TRUE(catch_throw([&]{ saturated = saturateSpeed(qd, 
                        q_prev, qd_prev, q_max, q_min, qd_max, qdd_max, dt, max_velocity_multiplier, preserve_direction, nullptr);}));
                    if(!saturated) 
                    {
                      std::cout << "CHECK: " << std::fabs(qd - qd)<< " - " << (std::fabs(qd - qd)==0) << std::endl;;
                    }
                  }
                }
              }
            }
          }
}

TEST(TestSuite, SixDoFSaturateVelFullState)
{   
  std::vector<Eigen::VectorXd> qq_max = {
    10.0 * I6, 10.0 * I6, -2.0 * I6
  };
  std::vector<Eigen::VectorXd> qq_min = {
    -10.0* I6, 2.0* I6, -10.0* I6
  };

  std::vector<Eigen::VectorXd> qq_prev = {
    0.9 * qq_max.at(0), 
      0.0001 * qq_max.at(1), 
        qq_min.at(0) + 0.01 * (qq_max.at(2)-qq_min.at(2)) 
  };

  std::vector<Eigen::VectorXd> qqd_max = {
    10.0 * I6, 10.0 * I6, 12.0 * I6, 
    10.0 * R6.cwiseAbs(), 10.0 * R6.cwiseAbs(), 12.0 * R6.cwiseAbs()
  };
  std::vector<Eigen::VectorXd> qqdd_max = {
    100.0 * I6, 100.0 * I6, 120.0 * I6, 
    100.0 * R6.cwiseAbs(), 100.0 * R6.cwiseAbs(), 120.0 * R6.cwiseAbs()
  };

  for(const auto & q_max : qq_max)
  {
    for(const auto & q_min : qq_min)
    {
      for(const auto & q_prev : qq_prev)
      {
        for(const auto & qd_max : qqd_max)
        {
          for(const auto & qdd_max : qqdd_max)
          {
            std::vector<Eigen::Matrix<double,6,1>> qqd = { qd_max - 2.0*I6, qd_max + 2.0*I6};
            std::vector<Eigen::Matrix<double,6,1>> qqd_prev = { 0 * I6, qd_max - 1.0*I6, 
                qd_max - 2.0*I6, qd_max - 10.0*I6, -qd_max +1.0*I6, -qd_max + 2.0*I6, qd_max + 10.0*I6};
            bool saturated = true;
            for(const double & max_velocity_multiplier : max_velocity_multipliers)
            {
              for(const bool & preserve_direction : preserve_directions )
              {
                for(auto & qd_prev : qqd_prev)
                {
                  for(auto & qd : qqd)
                  {
                    const auto _qd = qd;
                    EXPECT_TRUE(catch_throw([&]{ saturated = saturateSpeed(qd, 
                              q_prev, qd_prev, q_max, q_min, qd_max, qdd_max, dt, max_velocity_multiplier, preserve_direction, nullptr);}));
                    if(!saturated) 
                    {
                      std::cout << "CHECK: " << eigen_utils::to_string(qd - qd)<< std::endl;;
                    }
                  }
                }
              }
            }
          }
        }
      }

    }
  }
}


int main(int argc,char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS(); 
}
