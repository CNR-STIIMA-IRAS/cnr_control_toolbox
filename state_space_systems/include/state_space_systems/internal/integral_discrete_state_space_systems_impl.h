#pragma once   // workaraund qtcreator ctidy-lang

#ifndef   EIGEN_STATE_SPACE_SYSTEMS_INTEGRAL_DISCRETE_STATE_SPACE_SYSTEMS_IMPL_H
#define   EIGEN_STATE_SPACE_SYSTEMS_INTEGRAL_DISCRETE_STATE_SPACE_SYSTEMS_IMPL_H

//#include <rosparam_utilities/rosparam_utilities.h>
#include <eigen_matrix_utils/eigen_matrix_utils.h>
#include <eigen_matrix_utils/overloads.h>

#include <state_space_systems/integral_discrete_state_space_systems.h>

namespace eigen_control_toolbox
{

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
template<typename D> // proper resize of matrix A
inline bool defaultIntegralDiscreteStateSpaceMatrixA(Eigen::MatrixBase<D> const & matA, int order, int dof, double dt)
{
  if((order < 1)|| (dof <1))
    return false;

  Eigen::Matrix<double,-1,-1> eye;
  if(!eigen_utils::resize(eye,dof,dof))
    return false;

  if(!eigen_utils::resize(matA,(order+1)*dof,(order+1)*dof))
    return false;

  eigen_utils::setZero(matA);
  for(int ro=0;ro<order+1;ro++)
  {
    matA.block(ro*dof, ro*dof, dof, dof).setIdentity();
    for(int co=ro+1;co<order+1;co++)
    {
      matA.block(ro*dof, co*dof, dof, dof) = (dt / double(eigen_utils::factorial(co))) * eye;
    }
  }
}

template<typename D> // proper resize of matrix A
inline bool defaultIntegralDiscreteStateSpaceMatrixB(Eigen::MatrixBase<D> const & matB, int order, int dof, double dt)
{
  if((order < 1)|| (dof <1))
    return false;

  Eigen::Matrix<double,-1,-1> eye;
  if(!eigen_utils::resize(eye,dof,dof))
    return false;

  if(!eigen_utils::resize(matB,(order+1)*dof,dof))
    return false;

  eigen_utils::setZero(matB);
  for(int ro=0;ro<order+1;ro++)
  {
    matB.block(ro*dof, 0, dof, dof) = (dt / double(eigen_utils::factorial( (order+1-ro) ))) * eye;
  }
}


template<typename D> // proper resize of matrix A
inline bool defaultIntegralDiscreteStateSpaceMatrixC(Eigen::MatrixBase<D> const & matC, int order, int dof)
{
  if((order < 1)|| (dof <1))
    return false;

  if(!eigen_utils::resize(matC,dof, (order+1)*dof))
    return false;

  eigen_utils::setZero(matC);
  matC.block(0, 0, dof, dof).setIdentity();
}

template<typename D> // proper resize of matrix A
inline bool defaultIntegralDiscreteStateSpaceMatrixD(Eigen::MatrixBase<D> const & matD, int order, int dof)
{
  if((order < 1)|| (dof <1))
    return false;

  if(!eigen_utils::resize(matD,dof,dof))
    return false;

  eigen_utils::setZero(matD);
}
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


template<int K, int N, int MK, int MN>
inline IntegralDiscreteStateSpace<K,N,MK,MN>::IntegralDiscreteStateSpace()
  :  dt_(-1.0)
{

}

template<int K, int N, int MK, int MN>
inline IntegralDiscreteStateSpace<K,N,MK,MN>::IntegralDiscreteStateSpace(const IntegralDiscreteStateSpaceArgs& args)
{
  std::string what;
  int ok = this->setMatrices(args, what);
  if(ok!=1)
  {
    if(ok==0)
    {
      std::cerr<<__PRETTY_FUNCTION__<<":"<<__LINE__<<": " << what << std::endl;
    }
    else
    {
      throw std::invalid_argument(("Error in memory management: "+what).c_str());
    }
  }
}



template<int K, int N, int MK, int MN>
inline bool IntegralDiscreteStateSpace<K, N, MK, MN>::setMatrices(const BaseStateSpaceArgs& args, std::string& msg)
{
  const IntegralDiscreteStateSpace& _iargs = dynamic_cast<const IntegralDiscreteStateSpace&>(args);

   DiscreteStateSpaceArgs< IntegralStateSpaceSymbols<K,N,MK,MN>::S, N, N,
       IntegralStateSpaceSymbols<K,N,MK,MN>::MaxS, MN, MN> _oargs;

  int order = K==-1 ? _iargs.order : K;
  int dof = N==-1 ? _iargs.degrees_of_freedom : N;
  if(!defaultIntegralDiscreteStateSpaceMatrixA(_oargs.A,order,dof,_iargs.dt))
  {
    msg += std::string(__PRETTY_FUNCTION__) + ":" + std::to_string( __LINE__ ) + " error in setting A";
    return false;
  }

  if(!defaultIntegralDiscreteStateSpaceMatrixB(_oargs.B,order,dof,_iargs.dt))
  {
    msg += std::string(__PRETTY_FUNCTION__) + ":" + std::to_string( __LINE__ ) + " error in setting B";
    return false;
  }

  if(!defaultIntegralDiscreteStateSpaceMatrixC(_oargs.C,order,dof))
  {
    msg += std::string(__PRETTY_FUNCTION__) + ":" + std::to_string( __LINE__ ) + " error in setting C";
    return false;
  }

  if(!defaultIntegralDiscreteStateSpaceMatrixD(_oargs.D,order,dof))
  {
    msg + std::string(__PRETTY_FUNCTION__) + ":" + std::to_string( __LINE__ ) + " error in setting D";
    return false;
  }
  return DiscreteStateSpace<
    IntegralStateSpaceSymbols<K,N,MK,MN>::S, N, N,IntegralStateSpaceSymbols<K,N,MK,MN>::MaxS, MN, MN
      >::setMatrices(_oargs,msg);
}




}  // namespace eigen_control_toolbox

#endif  // state_space_systems_impl_201811280956
