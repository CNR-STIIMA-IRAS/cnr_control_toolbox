/*
 *  Software License Agreement (New BSD License)
 *
 *  Copyright 2020 National Council of Research of Italy (CNR)
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <cstdlib>
#include <ctime>
#include <iostream>

#include <gtest/gtest.h>
#include <gtest/gtest-death-test.h>

#include <eigen_matrix_utils/eigen_matrix_utils.h>
#include <state_space_filters/filtered_values.h>

constexpr size_t cycles = 1e4;


namespace detail
{
  struct unwrapper
  {
    unwrapper(std::exception_ptr pe) : pe_(pe) {}

    operator bool() const
    {
      return bool(pe_);
    }

    friend auto operator<<(std::ostream& os, unwrapper const& u) -> std::ostream&
    {
      try
      {
          std::rethrow_exception(u.pe_);
          return os << "no exception";
      }
      catch(std::runtime_error const& e)
      {
          return os << "runtime_error: " << e.what();
      }
      catch(std::logic_error const& e)
      {
          return os << "logic_error: " << e.what();
      }
      catch(std::exception const& e)
      {
          return os << "exception: " << e.what();
      }
      catch(...)
      {
          return os << "non-standard exception";
      }
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
};


// Declare a test
TEST(TestSuite, FilteredScalar)
{

  EXPECT_NO_FATAL_FAILURE(eigen_control_toolbox::FilteredScalar fv1) << "Ctor";
  
  constexpr size_t N = 1;
  eigen_control_toolbox::FilteredScalar fv1;
  double  d = 9;
  Eigen::Matrix<double,-1,1> x(1); x << -1;
  Eigen::Matrix<double,-1,1> y(2); y << 1,2; 

  EXPECT_NO_FATAL_FAILURE( fv1.value() = d );
  EXPECT_NO_FATAL_FAILURE( d = fv1.value() );
  EXPECT_TRUE( eigen_utils::norm(fv1.value()) == std::fabs(d) );

  // These will break at compile time 
  // EXPECT_FATAL_FAILURE( fv1.value() = x );
  // EXPECT_ANY_THROW( fv1.value() = y )<< "The allocation is static, and the dimensions mismatches";
  
  // === ACTIVATED FILTER
  EXPECT_NO_FATAL_FAILURE( fv1.activateFilter ( d, d, 0.1, 1e-3, d) );
  EXPECT_ANY_THROW( fv1.value() = d ) << "The filter is activated, you must call, update";
  EXPECT_NO_FATAL_FAILURE( fv1.update(d) );
  
  // === DEACTIVATED FILTER
  EXPECT_NO_FATAL_FAILURE( fv1.deactivateFilter(  ) );
  EXPECT_NO_FATAL_FAILURE( fv1.value() = d );
  EXPECT_NO_FATAL_FAILURE( fv1.update(d) );
  
  // ACCESSOR
  double* ptr = nullptr;
  EXPECT_NO_FATAL_FAILURE( ptr = fv1.data() );
  
  constexpr double vv = 1234.0;
  *ptr = vv;

  EXPECT_TRUE( fv1.value() == vv ); 
}

// Declare a test
TEST(TestSuite, FilteredVectorXd)
{
  EXPECT_NO_FATAL_FAILURE(eigen_control_toolbox::FilteredVectorXd fv) << "Ctor";
  
  eigen_control_toolbox::FilteredVectorXd fv;
  Eigen::Matrix<double, 1,1> d;    d <<  9;
  Eigen::Matrix<double,-1,1> x(1); x << -1;
  Eigen::Matrix<double,-1,1> y(2); y << 1,2; 

  // DIMENSION ARE NOT CHECKED, BECAUSE OF DYNAMIC ALLOCATION
  EXPECT_NO_FATAL_FAILURE( fv.value() = d ) << "Dynamic resize";
  EXPECT_TRUE( fv.value().norm() == d.norm() );
  EXPECT_TRUE( fv.value().rows() == d.rows() );

  EXPECT_NO_FATAL_FAILURE( fv.value() = x ) << "Dynamic resize";
  EXPECT_TRUE( (fv.value()- x).norm() == 0 );
  EXPECT_TRUE( fv.value().rows() == x.rows() );
  
  EXPECT_NO_FATAL_FAILURE( fv.value() = y );
  EXPECT_TRUE( fv.value().rows() == y.rows() );
  
  // ======================
  EXPECT_NO_FATAL_FAILURE( fv.activateFilter ( d, d, 0.1, 1e-3, d) );
  
  // if filter activated, you cannot direct assign the value, 
  // you must use "update"
  EXPECT_ANY_THROW( fv.value() = d ); 
  // Specifically, the value() method throw an exception when the filter is activated 
  EXPECT_ANY_THROW( int rows = fv.value().rows()) << "Getting the row() is not a const operator, sigh";

  EXPECT_NO_FATAL_FAILURE( fv.update(d) );
  EXPECT_NO_FATAL_FAILURE( d = fv.getUpdatedValue( ) );
  // ======================

  // ======================
  EXPECT_NO_FATAL_FAILURE( fv.deactivateFilter(  ) );
  EXPECT_NO_FATAL_FAILURE( fv.value() = d );
  EXPECT_NO_FATAL_FAILURE( fv.update(d) );
  EXPECT_NO_FATAL_FAILURE( d = fv.getUpdatedValue( ) );
  EXPECT_TRUE( fv.value().norm() == d.norm() );
  // ======================
  
  // ACCESSORS
  // ======================
  EXPECT_NO_FATAL_FAILURE( d = fv.value() );
  EXPECT_TRUE( fv.value().norm() == d.norm() );
  
  double a = 0.0;
  EXPECT_NO_FATAL_FAILURE( a = fv.value(0) );
  EXPECT_TRUE( fv.value(0) == a );
  
  a = 99.0;
  EXPECT_NO_FATAL_FAILURE( fv.value(0) = a );
  EXPECT_TRUE( fv.value(0) == a );

  double* ptr = nullptr;
  EXPECT_NO_FATAL_FAILURE( ptr = fv.data() );
  EXPECT_ANY_THROW( ptr = fv.data(99) );

  constexpr double vv = 1234.0;
  *ptr = vv;

  EXPECT_TRUE( fv.value(0) == vv ); 
  // ======================
}


// Declare a test
TEST(TestSuite, FilteredVector2d)
{

  EXPECT_NO_FATAL_FAILURE(eigen_control_toolbox::FilteredVector2d fv2) << "Ctor";
  constexpr size_t N = 2;
  eigen_control_toolbox::FilteredValue<N> fv2; // equivalent to eigen_control_toolbox::FilteredVector1d
  Eigen::Matrix<double, N,1> d; d <<  9,10;
  Eigen::Matrix<double,-1,1> x(1); x << -1;
  Eigen::Matrix<double,-1,1> y(3); y << 1,2,3; 

  Eigen::Matrix<double, N,1> dd; dd <<  9,10;
  Eigen::Matrix<double,-1,1> yy(3); yy << 1,2,3; 

  // DIMENSION ARE CHECKED, BECAUSE OF STATIC ALLOCATION
  EXPECT_NO_FATAL_FAILURE( fv2.value() = d ) << "The Eigen underlay equality fail if dimensions are different";
  EXPECT_TRUE( fv2.value().norm() == d.norm() );

#if NDEBUG == 0
  EXPECT_DEATH( fv2.value() = x, "") 
    << "If compiled in debug, there is an assert that check the dimension. x is of dimension 1, while fv2 is of dimension2";
  
  EXPECT_DEATH( fv2.value() = y, "" ) 
    << "If compiled in debug, there is an assert that check the dimension. x is of dimension 1, while fv2 is of dimension2";
#endif
  EXPECT_TRUE( eigen_utils::rows(fv2.value()) == N );
  EXPECT_TRUE( eigen_utils::rows(fv2.value()) != y.rows() );
  EXPECT_TRUE( fv2.value().rows() == N );
  EXPECT_TRUE( fv2.value().rows() != y.rows() );
  
  // ======================
  EXPECT_NO_FATAL_FAILURE( fv2.activateFilter ( d, d, 0.1, 1e-3, d) );
  EXPECT_ANY_THROW( fv2.value() = d );

  // if filter activated, you cannot direct assign the value, 
  // you must use "update"
  // Specifically, the value() method throw an exception when the filter is activated 
  EXPECT_ANY_THROW( int rows = fv2.value().rows()) << "Getting the row() is not a const operator, sigh";

  EXPECT_NO_FATAL_FAILURE( fv2.update(d) );
  EXPECT_NO_FATAL_FAILURE( d = fv2.getUpdatedValue( ) );
  // ======================

  // ======================
  EXPECT_NO_FATAL_FAILURE( fv2.deactivateFilter(  ) );
  EXPECT_NO_FATAL_FAILURE( fv2.value() = d );
  EXPECT_NO_FATAL_FAILURE( fv2.update(d) );
  EXPECT_NO_FATAL_FAILURE( d = fv2.getUpdatedValue( ) );
  EXPECT_TRUE( fv2.value().norm() == d.norm() );
  // ======================
  
  // ACCESSORS
  // ======================
  EXPECT_NO_FATAL_FAILURE( d = fv2.value() );
  EXPECT_TRUE( fv2.value().norm() == d.norm() );
  
  double a = 0.0;
  EXPECT_NO_FATAL_FAILURE( a = fv2.value(0) );
  EXPECT_TRUE( fv2.value(0) == a );
  
  a = 99.0;
  EXPECT_NO_FATAL_FAILURE( fv2.value(0) = a );
  EXPECT_TRUE( fv2.value(0) == a );

  double* ptr = nullptr;
  EXPECT_NO_FATAL_FAILURE( ptr = fv2.data() );
  EXPECT_ANY_THROW( ptr = fv2.data(6) );

  constexpr double vv = 1234.0;
  *ptr = vv;

  EXPECT_TRUE( fv2.value(0) == vv ); 
// ======================
}

// Declare a test
TEST(TestSuite, FilteredScalarPerformance)
{
  eigen_control_toolbox::FilteredScalar fv;
  double d = 1;

  std::srand(std::time(NULL));

  // ======================
  fv.activateFilter ( d, d, 0.1, 1e-3, d);
  
  for(size_t i=0;i<cycles; i++)
  {
    //d = std::rand() / RAND_MAX;
    fv.update(d);
  }
  // ======================
}

TEST(TestSuite, FilteredVectorXdPerformance)
{
  eigen_control_toolbox::FilteredVectorXd fv;
  Eigen::Matrix<double, 2,1> d;
  d << 1,2;

  // ======================
  fv.activateFilter ( d, d, 0.1, 1e-3, d);
  
  for(size_t i=0;i<cycles; i++)
  {
    //d.setRandom();
    fv.update(d);
  }
  // ======================
}

// Declare a test
TEST(TestSuite, FilteredVector2dPerformance)
{
  eigen_control_toolbox::FilteredVector2d fv;
  Eigen::Matrix<double, 2,1> d;
  d << 1,2;

  // ======================
  fv.activateFilter ( d, d, 0.1, 1e-3, d);
  
  for(size_t i=0;i<cycles; i++)
  {
    //d.setRandom();
    fv.update(d);
  } 
  // ======================
}

// Declare a test
TEST(TestSuite, FilteredVector3dPerformance)
{
  eigen_control_toolbox::FilteredVector3d fv;
  Eigen::Matrix<double, 3,1> d;
  d << 1,2,3;

  // ======================
  fv.activateFilter ( d, d, 0.1, 1e-3, d);
  
  for(size_t i=0;i<cycles; i++)
  {
    //d.setRandom();
    fv.update(d);
  } 
  // ======================
}


// Declare a test
TEST(TestSuite, FilteredVector4dPerformance)
{
  eigen_control_toolbox::FilteredVector4d fv;
  Eigen::Matrix<double, 4,1> d;
  d << 1,2,3,4;

  // ======================
  fv.activateFilter ( d, d, 0.1, 1e-3, d);
  
  for(size_t i=0;i<cycles; i++)
  {
    //d.setRandom();
    fv.update(d);
  } 
  // ======================
}


// Declare a test
TEST(TestSuite, FilteredVector5dPerformance)
{
  eigen_control_toolbox::FilteredValue<5> fv;
  Eigen::Matrix<double, 5,1> d;
  d << 1,2,3,4,5;

  // ======================
  fv.activateFilter ( d, d, 0.1, 1e-3, d);
  
  for(size_t i=0;i<cycles; i++)
  {
    //d.setRandom();
    fv.update(d);
  } 
  // ======================
}



// Declare a test
TEST(TestSuite, FilteredVector6dPerformance)
{
  eigen_control_toolbox::FilteredVector6d fv;
  Eigen::Matrix<double, 6,1> d;
  d << 1,2,3,4,5,6;

  // ======================
  fv.activateFilter ( d, d, 0.1, 1e-3, d);
  
  for(size_t i=0;i<cycles; i++)
  {
    //d.setRandom();
    fv.update(d);
  } 
  // ======================
}


TEST(TestSuite, FilteredVector6XdPerformance)
{
  eigen_control_toolbox::FilteredVectorXd fv;
  Eigen::Matrix<double, 6,1> d;
  d << 1,2,3,4,5,6;

  // ======================
  fv.activateFilter ( d, d, 0.1, 1e-3, d);
  
  for(size_t i=0;i<cycles; i++)
  {
    //d.setRandom();
    fv.update(d);
  }
  // ======================
}

TEST(TestSuite, FilteredVector6XPtrdPerformance)
{
  eigen_control_toolbox::FilteredVectorXdPtr fv(new eigen_control_toolbox::FilteredVectorXd());
  Eigen::Matrix<double, 6,1> d;
  d << 1,2,3,4,5,6;

  // ======================
  fv->activateFilter ( d, d, 0.1, 1e-3, d);
  
  for(size_t i=0;i<cycles; i++)
  {
    //d.setRandom();
    fv->update(d);
  }
  // ======================
}

// Declare a test
TEST(TestSuite, FilteredVector6dPtrPerformance)
{
  eigen_control_toolbox::FilteredVector6dPtr fv(new eigen_control_toolbox::FilteredVector6d());
  Eigen::Matrix<double, 6,1> d;
  d << 1,2,3,4,5,6;

  // ======================
  fv->activateFilter ( d, d, 0.1, 1e-3, d);
  
  for(size_t i=0;i<cycles; i++)
  {
    //d.setRandom();
    fv->update(d);
  } 
  // ======================
}


// Run all the tests that were declared with TEST()
int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
