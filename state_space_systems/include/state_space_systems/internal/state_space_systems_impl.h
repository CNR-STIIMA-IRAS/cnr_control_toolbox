#pragma once   // workaraund qtcreator ctidy-lang

#ifndef   state_space_systems_state_space_systems_impl_h
#define   state_space_systems_state_space_systems_impl_h

//#include <rosparam_utilities/rosparam_utilities.h>
#include <eigen_matrix_utils/eigen_matrix_utils.h>
#include <eigen_matrix_utils/overloads.h>

#include <state_space_systems/state_space_systems.h>

/**
 * 
 *
 * 
 * 
 * 
 */
namespace eigen_control_toolbox 
{



template<typename MatrixA, typename MatrixC> 
inline bool computeObservabilityMatrix(double&       out, 
                                       const MatrixA&   A,    // S x S
                                       const MatrixC&   C,    // O x S
                                       const int&       N)
{
  const int S = eigen_utils::rows(A);
  const int O = eigen_utils::cols(C);
  const int K = (N > S ? N : S);
  const int R = K*O;
  if(!eigen_utils::resize(out, R,S)) // resize the derived object
  {
    std::cerr << __PRETTY_FUNCTION__<<":" << __LINE__ << ": OBS, with " << eigen_utils::rows(out) << "x" << eigen_utils::cols(out) << std::endl;
    return false;
  } 
  eigen_utils::setZero(out);
  
  MatrixA pow_a;
  if(!eigen_utils::resize(pow_a, S, S))
  {
    std::cerr << __PRETTY_FUNCTION__<<":" << __LINE__ << ": Error in Resizing A^1, with S=" << S << std::endl;
    return false;
  } 
  eigen_utils::setIdentity(pow_a);
  
  for (int k=0;k<K;k++)
  {
    if(!eigen_utils::copy_to_block(out, C * pow_a, k*O, 0, O,S))
    {
      std::cerr << __PRETTY_FUNCTION__<<":" << __LINE__ << ": Error in C A^n, with n=" << k << " out of N" << std::endl;
      std::cerr << "obs: " << eigen_utils::rows(out) << "x" << eigen_utils::cols(out) << std::endl;
      std::cerr << "C * pow_a: " << eigen_utils::rows(C * pow_a) << "x" << eigen_utils::cols(C * pow_a) << std::endl;
    } // [ [C]; [CA]; ... [CA^(n-1)] ]
    pow_a = pow_a*A;
  }
  return ( eigen_utils::rank(out) == eigen_utils::rows(A) );
}


template<typename MatrixA, typename MatrixC, typename MatrixObs> 
inline bool computeObservabilityMatrix(Eigen::MatrixBase<MatrixObs>& out, 
                                       const MatrixA&                 A,    // S x S
                                       const MatrixC&                 C,    // O x S
                                       const int&                     N)
{
  const int S = eigen_utils::rows(A);
  const int O = eigen_utils::rows(C);
  const int K = (N > S ? N : S);
  const int R = K*O;
      
  if(!eigen_utils::resize(out, R,S)) // resize the derived object
  {
    std::cerr << __PRETTY_FUNCTION__<<":" << __LINE__ << ":  OBS, with " << eigen_utils::rows(out) << "x" << eigen_utils::cols(out) << "->" << R << "x" << S << std::endl;
    std::cerr << "Resize not possibile, the matrix is not fully dynamically instantiated" << std::endl;
    return false;
  } 
  eigen_utils::setZero(out);
  
  MatrixA pow_a;
  if(!eigen_utils::resize(pow_a, S, S))
  {
    std::cerr << __PRETTY_FUNCTION__<<":" << __LINE__ << ": Error in Resizing A^1, with S=" << S << std::endl;
    return false;
  } 
  eigen_utils::setIdentity(pow_a);
  
  for (int k=0;k<K;k++)
  {
    if(!eigen_utils::copy_to_block(out, C * pow_a, k*O, 0, O,S))
    {
      std::cerr << __PRETTY_FUNCTION__<<":" << __LINE__ << ": Error in C A^n, with n=" << k << " out of N" << std::endl;
      std::cerr << "S: " << S << ", O: " << O<< ", K: " << K << ", R: " << R << std::endl;
      std::cerr << "obs: " << eigen_utils::rows(out) << "x" << eigen_utils::cols(out) << std::endl;
      std::cerr << "C * pow_a: " << eigen_utils::rows(C * pow_a) << "x" << eigen_utils::cols(C * pow_a) << std::endl;
      return false;
    } // [ [C]; [CA]; ... [CA^(n-1)] ]
    pow_a = pow_a*A;
  }
  return ( eigen_utils::rank(out) == eigen_utils::rows(A) );
}

template<typename MatrixA, typename MatrixC, typename MatrixCtrl> 
inline bool computeControllabilityMatrix( MatrixCtrl&      out, 
                                          const MatrixA&   A,    // S x S
                                          const MatrixC&   B,    // S x I
                                          const int&       N)
{
  const int S = eigen_utils::rows(A);
  const int I = eigen_utils::cols(B);
  eigen_utils::resize(out,S, N*I); // resize the derived object
  eigen_utils::setZero(out);
  
  MatrixA pow_a;
  eigen_utils::resize(pow_a, S, S);
  eigen_utils::setIdentity(pow_a);
  
  for (int k=0;k<N;k++)
  {
    if(!eigen_utils::copy_to_block(out, pow_a * B, 0, (N-1-k)*I, S, I))
    {
      std::cerr << __PRETTY_FUNCTION__<<":" << __LINE__ << ": Error in A^n B, with n=" << k << " out of N" << std::endl;
    } // [ A^(k-1)B ... AB, B]
    pow_a = pow_a*A;
  }
  return ( eigen_utils::rank(out) == eigen_utils::rows(A) );
}


template<typename State0, typename MatrixA, typename MatrixB, typename MatrixC, typename MatrixD>
inline int estimateState0(State0& x0,
                          const MatrixA& A,
                          const MatrixB& B,
                          const MatrixC& C,
                          const MatrixD& D,
                          const Eigen::VectorXd& inputs,
                          const Eigen::VectorXd& outputs,
                          std::string& msg)
{
  /*
   * 
   * y(0) = C*x(0)         + D u(0)
   * y(1) = C*A*x(0)       + D u(1) + C*B*u(0)
   * y(2) = C*A^2*x(0)     + D u(2) + C*B*u(1)+C*A*B*u(1)
   * ......
   * y(n) = C*A^(n-1)*x(0) + D u(n) + sum(j=0:n-1) C*A^j*B*u(n-1-j)
   *
   * Y=[y(0) ... y(n)]
   * U=[u(0) ... u(n)]
   * Y=obsv(A,C)*x(0)+ioMatrix(A,B,C,D)*U 
   * x(0)= obsv(A,C) \ ( Y-ioMatrix(A,B,C,D)*U )
   */
  const int S  = eigen_utils::rows(A);
  const int I  = eigen_utils::cols(B);
  const int O  = eigen_utils::rows(C);
  if(inputs.rows() % I != 0 )
  {
    msg +="The argumnet 'inputs' size is not a multiple of the system input dimension";
    return -1;
  }
  if(outputs.rows() % O != 0 )
  {
    msg +="The argument 'outputs' size is not a multiple of the system output dimension";
    return -1;
  }
  const int ni = inputs.rows()  / I;
  const int no = outputs.rows() / O;
  if( ni!=no )
  {
    msg +="The argument 'inputs' and 'outputs' have incorrect dimensions";
    return -1;
  }
  const int N = ni;
  if( N <S )
  {
    msg +="The arguments are not sufficient to estimate the state";
    return -1;
  }
  Eigen::MatrixXd obs(N*O, S);
  if(!computeObservabilityMatrix(obs, A,C, N))
  {
    msg +="The arguments are not indipendent, the input will be wrongly be estimated (the observability matrix is rank-deficient)";
    return 0;
  }

  Eigen::MatrixXd i2o(N*O, N*I);
  Eigen::MatrixXd i2o_ctrl(N*O, N*I);
  Eigen::MatrixXd i2o_d(N*O, N*I);
  i2o.setZero();
  i2o_ctrl.setZero();
  i2o_d.setZero();
  for(int n=0;n<N;n++)
  {
    Eigen::MatrixXd ctrl_n(S, (n+1)*I);
    computeControllabilityMatrix( ctrl_n, A, B, n+1 );
    // C -> O x S
    // C x ctrl_n -> O x S x S x ((n+1)xI) -> O x ((n+1)xI)
    i2o_ctrl.block(n*O, 0, O, (n+1)*I) = C * ctrl_n;
  }

  for(int n=0;n<N;n++)
  {
    if(!eigen_utils::copy_to_block(i2o_d, D, n*O, n*I, O, I))
    {
      std::cerr << __PRETTY_FUNCTION__<<":" << __LINE__ << ": Error in I2O matrix filling, n=" << n << " out of N" << std::endl;
    }; // block diagonal
  }
  i2o = i2o_ctrl + i2o_d;

  // Y = obs * x0 + i2o * U --> x0 = obs \ (Y - i2o * U)
  Eigen::JacobiSVD< Eigen::MatrixXd > svd(obs, Eigen::ComputeThinU | Eigen::ComputeThinV);
  if(svd.rank()<obs.cols())
  {
    msg +="The observations are linearly dependent, and not sufficient to estimate the state.";
    return -1;
  }
  auto _x0 = svd.solve(outputs - i2o * inputs); // state at the begin of initialization interval
  if(!eigen_utils::copy_to_block(x0, _x0,0,0, S,1))
  {
    msg += "Error in x0 assignement.";
    return -1;
  }

  return 1;
}


inline 
void BaseStateSpace::setSamplingPeriod(const double& sampling_period)
{
  m_sampling_period=sampling_period;
}

inline 
double BaseStateSpace::getSamplingPeriod() const
{
  return m_sampling_period;
}




inline ::std::ostream& operator<<(::std::ostream& os, const eigen_control_toolbox::BaseStateSpace& bss)
{
    os << to_string(bss);
    return os;
}

inline std::string to_string(const eigen_control_toolbox::BaseStateSpace& bss)
{
  std::stringstream ret;
  ret << "System with "
      << bss.uDim() << " inputs, " << bss.xDim() << " states, and " << bss.yDim() << "outputs" << std::endl;
  ret << "A:\n"<< eigen_utils::to_string(bss.getA()) << std::endl;
  ret << "B:\n"<< eigen_utils::to_string(bss.getB()) << std::endl;
  ret << "C:\n"<< eigen_utils::to_string(bss.getC()) << std::endl;
  ret << "D:\n"<< eigen_utils::to_string(bss.getD()) << std::endl;
  ret << "output:\n"<< eigen_utils::to_string(bss.getY()) << std::endl;
  ret << "state:\n" << eigen_utils::to_string(bss.getX()) << std::endl;
  return ret.str();
}


}  // namespace eigen_control_toolbox


#endif  // state_space_systems_impl_201811280956
