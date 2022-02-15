#include <chrono>  // for high_resolution_clock

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <ctime>

#include <ros/ros.h>

#include <eigen_matrix_utils/eigen_matrix_utils.h>
#include <state_space_filters/common_filters.h>
#include <gtest/gtest.h>

using namespace eigen_control_toolbox;

Eigen::IOFormat fmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "", "");

constexpr int stress_cycles = 1e5;
constexpr int cycles = 1000;
double natural_frequency =   50; // [rad/s] 2 pi * f
double sampling_period   = 0.001; // s


TEST(TestSuite, FirstOrderLowPassX)
{
  EXPECT_NO_FATAL_FAILURE( FirstOrderLowPassX lpf );
  EXPECT_NO_FATAL_FAILURE( FirstOrderLowPassX lpf(natural_frequency,sampling_period, 3) );

  EXPECT_NO_FATAL_FAILURE( FirstOrderLowPassXPtr lpf );
  EXPECT_NO_FATAL_FAILURE( FirstOrderLowPassXPtr lpf(
    new FirstOrderLowPassX(natural_frequency,sampling_period, 3)) );

  FirstOrderLowPassX lpf;
  FirstOrderLowPassXPtr lpf_ptr(new FirstOrderLowPassX());
  EXPECT_TRUE( lpf.init(natural_frequency, sampling_period, 3) );
  EXPECT_TRUE( lpf_ptr->init(natural_frequency, sampling_period, 3) );

  EXPECT_NO_FATAL_FAILURE( int order = lpf.xDim()           );
  EXPECT_NO_FATAL_FAILURE( int nin   = lpf.uDim()  );
  EXPECT_NO_FATAL_FAILURE( int nout  = lpf.yDim() );

  EXPECT_NO_FATAL_FAILURE( int order = lpf_ptr->xDim()           );
  EXPECT_NO_FATAL_FAILURE( int nin   = lpf_ptr->uDim()  );
  EXPECT_NO_FATAL_FAILURE( int nout  = lpf_ptr->yDim() );

  EXPECT_TRUE( lpf.xDim() == lpf.xDim()           );
  EXPECT_TRUE( lpf.uDim() == lpf.uDim()  );
  EXPECT_TRUE( lpf.yDim() == lpf.yDim() );

  int ch;
  EXPECT_NO_FATAL_FAILURE( ch = lpf.getChannels()        );
  Eigen::VectorXd u(ch); u.setRandom();
  Eigen::VectorXd y(ch); y.setRandom();
  EXPECT_TRUE( lpf.setStateFromLastIO(u,  y) );
  EXPECT_TRUE( eigen_utils::norm(lpf.u()  - u) < 1e-12 ) << "get: " + std::to_string(eigen_utils::norm(lpf.u()));  
  EXPECT_TRUE( eigen_utils::norm(lpf.y() - y) < 1e-12 ) << "get: " + std::to_string(eigen_utils::norm(lpf.y()));  
  EXPECT_NO_FATAL_FAILURE( y=lpf.update(u) );
}



TEST(TestSuite, FirstOrderLowPassXPerformance)
{
  for(int i=2; i<10;i++)
  {
    FirstOrderLowPassX lpf;
    lpf.init(natural_frequency, sampling_period, i);

    int ch = lpf.getChannels();
    Eigen::VectorXd u(ch); u.setRandom();
    Eigen::VectorXd y(ch); y.setRandom();
    lpf.setStateFromLastIO(u,  y);
    
    auto start = std::chrono::high_resolution_clock::now();
    for (unsigned int i=0;i<stress_cycles ;i++)
    {
      y=lpf.update(u);
    }
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::microseconds elapsed = std::chrono::duration_cast<std::chrono::microseconds>(finish - start);
    std::cout << "Num. Channels: " << ch << ", Performance over " << stress_cycles << " cycles: " << elapsed.count() << "us\t\t";
    std::cout << "Time/ch/cycles: " << (double(elapsed.count())/double(ch))/double(stress_cycles) << "us" << std::endl;
    EXPECT_TRUE( eigen_utils::norm(lpf.y() - u) < 1e-12 ) << "get: " + std::to_string(eigen_utils::norm(lpf.y() - u));
  }
}

TEST(TestSuite, FirstOrderLowPassXPtrPerformance)
{
  for(int i=2; i<10;i++)
  {
    FirstOrderLowPassXPtr lpf(new FirstOrderLowPassX());
    lpf->init(natural_frequency, sampling_period, i);

    int ch = lpf->getChannels();
    Eigen::VectorXd u(ch); u.setRandom();
    Eigen::VectorXd y(ch); y.setRandom();
    lpf->setStateFromLastIO(u,  y);
    
    auto start = std::chrono::high_resolution_clock::now();
    for (unsigned int i=0;i<stress_cycles ;i++)
    {
      y=lpf->update(u);
    }
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::microseconds elapsed = std::chrono::duration_cast<std::chrono::microseconds>(finish - start);
    std::cout << "Num. Channels: " << ch << ", Performance over " << stress_cycles << " cycles: " << elapsed.count() << "us\t\t";
    std::cout << "Time/ch/cycles: " << (double(elapsed.count())/double(ch))/double(stress_cycles) << "us" << std::endl;
    EXPECT_TRUE( eigen_utils::norm(lpf->y() - u) < 1e-12 ) << "get: " + std::to_string(eigen_utils::norm(lpf->y() - u));
  }
}

// Declare a test
TEST(TestSuite, FirstOrderLowPassXPlot)
{
  FirstOrderLowPassX lpf;
  lpf.init(natural_frequency, sampling_period, 3);

  int ch = lpf.getChannels();
  Eigen::VectorXd u(ch); u.setRandom();
  Eigen::VectorXd y(ch); y.setRandom();
  lpf.setStateFromLastIO(u,  y);

  std::ofstream ofile("testX.plt", std::ofstream::out);
  ofile << "u1,u2,u3," 
        << "x1,x2,x3," 
        << "y1,y2,y3," 
        << "(u-y).norm," 
        << "(u-i).norm" << std::endl;
  u(0) = 1.0;
  for (unsigned int i=0;i<cycles;i++)
  {
    ofile << u.transpose().format(fmt) << ", " 
          << lpf.x().transpose().format(fmt) << ", " 
          << lpf.y().transpose().format(fmt) << ", "
          << eigen_utils::norm(u-lpf.y()) << ","
          << eigen_utils::norm(u-lpf.u()) << std::endl; 
    y=lpf.update(u);
  }
  EXPECT_TRUE( eigen_utils::norm(lpf.y() - u) < 1e-12 ) << "get: " + std::to_string(eigen_utils::norm(lpf.y() - u));  
}

TEST(TestSuite, FirstOrderLowPass6)
{
  EXPECT_NO_FATAL_FAILURE( FirstOrderLowPass<6> lpf(natural_frequency,sampling_period) );
  EXPECT_NO_FATAL_FAILURE( FirstOrderLowPassPtr<6> lpf(
      new FirstOrderLowPass<6>(natural_frequency,sampling_period)) );
  
  FirstOrderLowPass<6> lpf(natural_frequency,sampling_period);
  FirstOrderLowPassPtr<6> lpf_ptr(new FirstOrderLowPass<6>(natural_frequency,sampling_period));

  EXPECT_NO_FATAL_FAILURE( int order = lpf.xDim() );
  EXPECT_NO_FATAL_FAILURE( int nin   = lpf.uDim() );
  EXPECT_NO_FATAL_FAILURE( int nout  = lpf.yDim() );

  EXPECT_NO_FATAL_FAILURE( int order = lpf_ptr->xDim() );
  EXPECT_NO_FATAL_FAILURE( int nin   = lpf_ptr->uDim() );
  EXPECT_NO_FATAL_FAILURE( int nout  = lpf_ptr->yDim() );

  int ch;
  EXPECT_NO_FATAL_FAILURE( ch = lpf.getChannels() );
  Eigen::VectorXd u(ch); u.setRandom();
  Eigen::VectorXd y(ch); y.setRandom();

  EXPECT_TRUE( lpf.setStateFromLastIO(u,  y) );
  EXPECT_TRUE( eigen_utils::norm(lpf.u() - u) < 1e-12 ) << "get: " + std::to_string(eigen_utils::norm(lpf.u()));  
  EXPECT_TRUE( eigen_utils::norm(lpf.y() - y) < 1e-12 ) << "get: " + std::to_string(eigen_utils::norm(lpf.y()));  
  EXPECT_NO_FATAL_FAILURE( y=lpf.update(u) );
}

TEST(TestSuite, FirstOrderLowPass6Performance)
{
  FirstOrderLowPass<6> lpf(natural_frequency,sampling_period);
 
  int ch = lpf.getChannels();
  Eigen::VectorXd u(ch); u.setZero();
  Eigen::VectorXd y(ch); y.setZero();

  lpf.setStateFromLastIO(u,  y);
  u(0) = 1.0;
  auto start = std::chrono::high_resolution_clock::now();
  for (unsigned int i=0;i<stress_cycles ;i++)
  {
    y=lpf.update(u);
  }
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::microseconds elapsed = std::chrono::duration_cast<std::chrono::microseconds>(finish - start);
  std::cout << "Num. Channels: " << ch << ", Performance over " << stress_cycles << " cycles: " << elapsed.count() << "us\t\t";
  std::cout << "Time/ch/cycles: " << (double(elapsed.count())/double(ch))/double(stress_cycles) << "us" << std::endl;
  EXPECT_TRUE( eigen_utils::norm(lpf.y() - u) < 1e-12 ) << "get: " + std::to_string(eigen_utils::norm(lpf.y() - u));
}

TEST(TestSuite, FirstOrderLowPass6PtrPerformance)
{
  FirstOrderLowPassPtr<6> lpf(new FirstOrderLowPass<6>(natural_frequency,sampling_period));
 
  int ch = lpf->getChannels();
  Eigen::VectorXd u(ch); u.setZero();
  Eigen::VectorXd y(ch); y.setZero();

  lpf->setStateFromLastIO(u,  y);
  u(0) = 1.0;
  auto start = std::chrono::high_resolution_clock::now();
  for (unsigned int i=0;i<stress_cycles ;i++)
  {
    y=lpf->update(u);
  }
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::microseconds elapsed = std::chrono::duration_cast<std::chrono::microseconds>(finish - start);
  std::cout << "Num. Channels: " << ch << ", Performance over " << stress_cycles << " cycles: " << elapsed.count() << "us\t\t";
  std::cout << "Time/ch/cycles: " << (double(elapsed.count())/double(ch))/double(stress_cycles) << "us" << std::endl;
  EXPECT_TRUE( eigen_utils::norm(lpf->y() - u) < 1e-12 ) << "get: " + std::to_string(eigen_utils::norm(lpf->y() - u));
}

TEST(TestSuite, FirstOrderLowPass6Plot)
{
  FirstOrderLowPass<6> lpf(natural_frequency,sampling_period);
  int ch = lpf.getChannels();
  Eigen::VectorXd u(ch); u.setRandom();
  Eigen::VectorXd y(ch); y.setRandom();
  lpf.setStateFromLastIO(u,  y);

  std::ofstream ofile("test3.plt", std::ofstream::out);
  ofile << "u1,u2,u3,u4,u5,u6," 
        << "x1,x2,x3,x4,x5,x6," 
        << "y1,y2,y3,y4,y5,y6," 
        << "(u-y).norm," 
        << "(u-i).norm" << std::endl;
  u(0) = 1.0;
  for (unsigned int i=0;i<cycles;i++)
  {
    ofile << u.transpose().format(fmt) << ", " 
          << lpf.x().transpose().format(fmt) << ", " 
          << lpf.y().transpose().format(fmt) << ", "
          << eigen_utils::norm(u-lpf.y()) << ","
          << eigen_utils::norm(u-lpf.u()) << std::endl; 
    y=lpf.update(u);
  }
  EXPECT_TRUE( eigen_utils::norm(lpf.y() - u) < 1e-12 ) << "get: " + std::to_string(eigen_utils::norm(lpf.y() - u));  
}

TEST(TestSuite, FirstOrderLowPass1)
{
  EXPECT_NO_FATAL_FAILURE( FirstOrderLowPass<1> lpf(natural_frequency,sampling_period) );

  FirstOrderLowPass<1> lpf(natural_frequency,sampling_period);
  EXPECT_NO_FATAL_FAILURE( int order = lpf.xDim()           );
  EXPECT_NO_FATAL_FAILURE( int nin   = lpf.uDim()  );
  EXPECT_NO_FATAL_FAILURE( int nout  = lpf.yDim() );

  int ch;
  EXPECT_NO_FATAL_FAILURE( ch = lpf.getChannels()        );
  double u = 0.67;
  double y = 0.34;

  EXPECT_TRUE( lpf.setStateFromLastIO(u,  y) );
  EXPECT_TRUE( eigen_utils::norm(lpf.u()  - u) < 1e-12 ) << "get: " + std::to_string(eigen_utils::norm(lpf.u()));  
  EXPECT_TRUE( eigen_utils::norm(lpf.y() - y) < 1e-12 ) << "get: " + std::to_string(eigen_utils::norm(lpf.y()));  
  EXPECT_NO_FATAL_FAILURE( y=lpf.update(u) );
}

TEST(TestSuite, FirstOrderLowPass1Performance)
{
  FirstOrderLowPass<1> lpfa(natural_frequency,sampling_period);
  FirstOrderLowPass<1> lpfb(natural_frequency,sampling_period);
  FirstOrderLowPass<1> lpfc(natural_frequency,sampling_period);
  FirstOrderLowPass<1> lpfd(natural_frequency,sampling_period);
  FirstOrderLowPass<1> lpfe(natural_frequency,sampling_period);
  FirstOrderLowPass<1> lpff(natural_frequency,sampling_period);

  double u = 0.67;
  double y = 0.34;

  lpfa.setStateFromLastIO(u,  y);
  lpfb.setStateFromLastIO(u,  y);
  lpfc.setStateFromLastIO(u,  y);
  lpfd.setStateFromLastIO(u,  y);
  lpfe.setStateFromLastIO(u,  y);
  lpff.setStateFromLastIO(u,  y);
  u = 1.0;
  auto start = std::chrono::high_resolution_clock::now();
  for (unsigned int i=0;i<stress_cycles ;i++)
  {
    double ya = lpfa.update(u);
    double yb = lpfb.update(u);
    double yc = lpfc.update(u);
    double yd = lpfd.update(u);
    double ye = lpfe.update(u);
    double yf = lpff.update(u);
  }
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::microseconds elapsed = std::chrono::duration_cast<std::chrono::microseconds>(finish - start);
  std::cout << "Num. Channels: " << 6 << ", Performance over " << stress_cycles << " cycles: " << elapsed.count() << "us\t\t";
  std::cout << "Time/ch/cycles: " << (double(elapsed.count())/double(6))/double(stress_cycles) << "us" << std::endl;
  EXPECT_TRUE( eigen_utils::norm(lpfa.y() - u) < 1e-12 ) << "get: " + std::to_string(eigen_utils::norm(lpfa.y() - u));
  EXPECT_TRUE( eigen_utils::norm(lpfb.y() - u) < 1e-12 ) << "get: " + std::to_string(eigen_utils::norm(lpfb.y() - u));
  EXPECT_TRUE( eigen_utils::norm(lpfc.y() - u) < 1e-12 ) << "get: " + std::to_string(eigen_utils::norm(lpfc.y() - u));
  EXPECT_TRUE( eigen_utils::norm(lpfd.y() - u) < 1e-12 ) << "get: " + std::to_string(eigen_utils::norm(lpfa.y() - u));
  EXPECT_TRUE( eigen_utils::norm(lpfe.y() - u) < 1e-12 ) << "get: " + std::to_string(eigen_utils::norm(lpfb.y() - u));
  EXPECT_TRUE( eigen_utils::norm(lpff.y() - u) < 1e-12 ) << "get: " + std::to_string(eigen_utils::norm(lpfc.y() - u));
}

TEST(TestSuite, FirstOrderLowPass1Plot)
{
  FirstOrderLowPass<1> lpf(natural_frequency,sampling_period);
 
  double u = 0.67;
  double y = 0.34;

  lpf.setStateFromLastIO(u,  y);
  std::ofstream ofile("test1.plt", std::ofstream::out);
  ofile << u << ", " 
        << lpf.x() << ", " 
        << lpf.y() << ", "
        << eigen_utils::norm(u-lpf.y()) << ","
        << eigen_utils::norm(u-lpf.u()) << std::endl;
          
  
  ofile << "u," 
        << "x," 
        << "y," 
        << "(u-y).norm," 
        << "(u-i).norm" << std::endl;
  u = 1.0;
  for (unsigned int i=0;i<cycles;i++)
  { 
    y=lpf.update(u);
    ofile << u << ", " 
          << lpf.x() << ", " 
          << lpf.y() << ", "
          << eigen_utils::norm(u-lpf.y()) << ","
          << eigen_utils::norm(u-lpf.u()) << std::endl;
  }
  EXPECT_TRUE( eigen_utils::norm(lpf.y() - u) < 1e-12 ) << "get: " + std::to_string(eigen_utils::norm(lpf.y() - u));  
}

TEST(TestSuite, FirstOrderHighPassX)
{
  EXPECT_NO_FATAL_FAILURE( FirstOrderHighPassX hpf );
  EXPECT_NO_FATAL_FAILURE( FirstOrderHighPassX hpf(natural_frequency,sampling_period, 3) );

  EXPECT_NO_FATAL_FAILURE( FirstOrderHighPassXPtr hpf );
  EXPECT_NO_FATAL_FAILURE( FirstOrderHighPassXPtr hpf(
    new FirstOrderHighPassX(natural_frequency,sampling_period, 3)) );

  FirstOrderHighPassX hpf;
  FirstOrderHighPassXPtr hpf_ptr(new FirstOrderHighPassX());
  EXPECT_TRUE( hpf.init(natural_frequency, sampling_period, 3) );
  EXPECT_TRUE( hpf_ptr->init(natural_frequency, sampling_period, 3) );

  EXPECT_NO_FATAL_FAILURE( int order = hpf.xDim()           );
  EXPECT_NO_FATAL_FAILURE( int nin   = hpf.uDim()  );
  EXPECT_NO_FATAL_FAILURE( int nout  = hpf.yDim() );

  EXPECT_NO_FATAL_FAILURE( int order = hpf_ptr->xDim()           );
  EXPECT_NO_FATAL_FAILURE( int nin   = hpf_ptr->uDim()  );
  EXPECT_NO_FATAL_FAILURE( int nout  = hpf_ptr->yDim() );

  EXPECT_TRUE( hpf.xDim() == hpf.xDim()           );
  EXPECT_TRUE( hpf.uDim() == hpf.uDim()  );
  EXPECT_TRUE( hpf.yDim() == hpf.yDim() );

  int ch;
  EXPECT_NO_FATAL_FAILURE( ch = hpf.getChannels()        );
  Eigen::VectorXd u(ch); u.setRandom();
  Eigen::VectorXd y(ch); y.setRandom();
  EXPECT_TRUE( hpf.setStateFromLastIO(u,  y) );
  EXPECT_TRUE( eigen_utils::norm(hpf.u()  - u) < 1e-12 ) << "get: " + std::to_string(eigen_utils::norm(hpf.u()));
  EXPECT_TRUE( eigen_utils::norm(hpf.y() - y) < 1e-12 ) << "get: " + std::to_string(eigen_utils::norm(hpf.y()));
  EXPECT_NO_FATAL_FAILURE( y=hpf.update(u) );
}









int main(int argc,char* argv[])
{ 
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS(); 
}
