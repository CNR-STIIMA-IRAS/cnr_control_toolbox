#pragma once   // workaraund qtcreator ctidy-lang

#ifndef   EIGEN_STATE_SPACE_SYSTEMS__INTEGRAL_STATE_SPACE_SYSTEMS_IMPL_H
#define   EIGEN_STATE_SPACE_SYSTEMS__INTEGRAL_STATE_SPACE_SYSTEMS_IMPL_H

//#include <rosparam_utilities/rosparam_utilities.h>
#include <eigen_matrix_utils/eigen_matrix_utils.h>
#include <eigen_matrix_utils/overloads.h>

#include <state_space_systems/integral_state_space_systems.h>

namespace eigen_control_toolbox
{

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
template<typename Derived> // proper resize of matrix A
inline bool defaultIntegralStateSpaceMatrixA(Eigen::MatrixBase<Derived> const & matA, int order, int dof)
{
  if((order < 1)|| (dof <1))
    return false;

  if(!eigen_utils::resize(matA,(order+1)*dof,(order+1)*dof))
    return false;

  eigen_utils::setZero(matA);
  for(int o=0;o<order;o++)
  {
    matA.block(o*dof, (o+1)*dof, dof, dof).setIdentity();
  }
}

template<typename Derived> // proper resize of matrix A
inline bool defaultIntegralStateSpaceMatrixB(Eigen::MatrixBase<Derived> const & matB, int order, int dof)
{
  if((order < 1)|| (dof <1))
    return false;

  if(!eigen_utils::resize(matB,(order+1)*dof,dof))
    return false;

  eigen_utils::setZero(matB);
  matB.block(order*dof, 0, dof, dof).setIdentity();
}


template<typename Derived> // proper resize of matrix A
inline bool defaultIntegralStateSpaceMatrixC(Eigen::MatrixBase<Derived> const & matC, int order, int dof)
{
  if((order < 1)|| (dof <1))
    return false;

  if(!eigen_utils::resize(matC,dof, (order+1)*dof))
    return false;

  eigen_utils::setZero(matC);
  matC.block(0, 0, dof, dof).setIdentity();
}

template<typename Derived> // proper resize of matrix A
inline bool defaultIntegralStateSpaceMatrixD(Eigen::MatrixBase<Derived> const & matD, int order, int dof)
{
  if((order < 1)|| (dof <1))
    return false;

  if(!eigen_utils::resize(matD,dof,dof))
    return false;

  eigen_utils::setZero(matD);
}
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^









template<int K, int N, int MK, int MN>
inline IntegralStateSpace<K,N,MK,MN>::IntegralStateSpace()
{
  std::string what;
  IntegralStateSpaceArgs args;
  args.order = K;
  args.degrees_of_freedom = N;
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

/**
 * 
 * 
 * 
 * 
 */
template<int K, int N, int MK, int MN>
template<int k,int n, std::enable_if_t<(k==-1)||(n==-1),int> >
inline IntegralStateSpace<K,N,MK,MN>::IntegralStateSpace(const IntegralStateSpaceArgs& args)
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
inline bool IntegralStateSpace<K, N, MK, MN>::setMatrices(const BaseStateSpaceArgs& args, std::string& msg)
{
  const IntegralStateSpaceArgs& _iargs = dynamic_cast<const IntegralStateSpaceArgs&>(args);
  DiscreteStateSpaceArgs<IntegralStateSpaceSymbols<K,N,MK,MN>::S, N, N,
                            IntegralStateSpaceSymbols<K,N,MK,MN>::MaxS, MN, MN> _oargs;
  std::string what;

  /////////////////////////////////// Check dimension (if matA is static allocated, unless, do nothing)
  typename IntegralStateSpaceSymbols<K,N,MK,MN>::MatrixA matA;
  if((eigen_utils::rows(matA)>0) && ((_iargs.order!=K)||(_iargs.degrees_of_freedom!=N)))
  {
    msg += std::string(__PRETTY_FUNCTION__) + ":" + std::to_string( __LINE__ )
          + " You are trying to resize a static-allocated system with a different number of order and/or dof. Abort.";
    return false;
  }
  ///////////////////////////////////

  if(!defaultIntegralStateSpaceMatrixA(_oargs.A, _iargs.order,_iargs.degrees_of_freedom))
  {
    msg += std::string(__PRETTY_FUNCTION__) + ":" + std::to_string( __LINE__ ) + " error in setting A";
    return false;
  }
  if(!defaultIntegralStateSpaceMatrixB(_oargs.B,_iargs.order,_iargs.degrees_of_freedom))
  {
    msg += std::string(__PRETTY_FUNCTION__) + ":" + std::to_string( __LINE__ ) + " error in setting B";
    return false;
  }
  if(!defaultIntegralStateSpaceMatrixC(_oargs.C,_iargs.order,_iargs.degrees_of_freedom))
  {
    msg += std::string(__PRETTY_FUNCTION__) + ":" + std::to_string( __LINE__ ) + " error in setting C";
    return false;
  }
  if(!defaultIntegralStateSpaceMatrixD(_oargs.D,_iargs.order,_iargs.degrees_of_freedom))
  {
    msg + std::string(__PRETTY_FUNCTION__) + ":" + std::to_string( __LINE__ ) + " error in setting D";
    return false;
  }

  // call the father function, that copy the matrices inside base class
  return DiscreteStateSpace<IntegralStateSpaceSymbols<K,N,MK,MN>::S, N, N,
      IntegralStateSpaceSymbols<K,N,MK,MN>::MaxS,MN,MN>::setMatrices(_oargs,msg);
}

template<int K, int N, int MK, int MN>
inline int IntegralStateSpace<K,N,MK,MN>::qDim() const
{
  return this->initialized() ? this->xDim()/2 : -1;
}

template<int K, int N, int MK, int MN>
inline void IntegralStateSpace<K,N,MK,MN>::setQk(const IntegralStateSpace<K,N,MK,MN>::Value& q, int order)
{
  auto _x = this->x();
  eigen_utils::setZero(_x);
  eigen_utils::copy_to_block(_x,q,order*this->qDim(),0,this->qDim(),1);
  this->setState(_x);
}

template<int K, int N, int MK, int MN>
inline const typename IntegralStateSpace<K,N,MK,MN>::Value&
IntegralStateSpace<K,N,MK,MN>::qk(int order) const
{
  static typename IntegralStateSpace<K,N,MK,MN>::Value ret;
  eigen_utils::resize(ret, this->qDim());
  const typename IntegralStateSpace<K,N,MK,MN>::State& state = this->x();
  eigen_utils::copy_from_block(ret,state,order*qDim(),0,qDim(),1);
  return ret;
}


/*
template<int K, int N, int MK, int MN>
inline typename IntegralStateSpace<K,N,MK,MN>::Value&
IntegralStateSpace<K,N,MK,MN>::update(const IntegralStateSpace<K,N,MK,MN>::Value& input, bool skip_dimension_check)
{
  return impl_.update(input,skip_dimension_check);
}


template<int K, int N, int MK, int MN>
inline void IntegralStateSpace<K,N,MK,MN>::set_x(const IntegralStateSpace<K,N,MK,MN>::State& state)
{
  impl_.setState(state);
}



template<int K, int N, int MK, int MN>
inline bool IntegralStateSpace<K,N,MK,MN>::setStateFromIO(
    const Eigen::VectorXd& past_inputs,
    const Eigen::VectorXd& past_outputs)
{
  return impl_.setStateFromIO(past_inputs,past_outputs);
}

template<int K, int N, int MK, int MN>
inline bool IntegralStateSpace<K,N,MK,MN>::setStateFromLastIO(const Value& inputs, const Value& outputs)
{
  return impl_.setStateFromLastIO(inputs,outputs);
}

template<int K, int N, int MK, int MN>
inline bool IntegralStateSpace<K,N,MK,MN>::initialized()
{
  return xDim() > 0 && uDim() > 0 && yDim() > 0;
}



template<int K, int N, int MK, int MN>
inline const typename IntegralStateSpace<K,N,MK,MN>::MatrixA&
IntegralStateSpace<K,N,MK,MN>::A() const
{
  return impl_.A();
}

template<int K, int N, int MK, int MN>
inline const typename IntegralStateSpace<K,N,MK,MN>::MatrixB&
IntegralStateSpace<K,N,MK,MN>::B() const
{
  return impl_.B();
}


template<int K, int N, int MK, int MN>
inline const typename IntegralStateSpace<K,N,MK,MN>::MatrixC&
IntegralStateSpace<K,N,MK,MN>::C() const
{
  return impl_.C();
}

template<int K, int N, int MK, int MN>
inline const typename IntegralStateSpace<K,N,MK,MN>::MatrixD&
IntegralStateSpace<K,N,MK,MN>::D() const
{
  return impl_.D();
}


template<int K, int N, int MK, int MN>
inline const typename IntegralStateSpace<K,N,MK,MN>::Value&
IntegralStateSpace<K, N, MK, MN>::u() const
{
  return impl_.u();
}


template<int K, int N, int MK, int MN>
inline const typename IntegralStateSpace<K,N,MK,MN>::Value&
IntegralStateSpace<K,N,MK,MN>::y() const
{
  return impl_.y();
}


template<int K, int N, int MK, int MN>
inline const typename IntegralStateSpace<K,N,MK,MN>::State&
IntegralStateSpace<K,N,MK,MN>::x() const
{
  return impl_.x();
}

template<int K, int N, int MK, int MN>
inline const typename IntegralStateSpace<K,N,MK,MN>::Value&
IntegralStateSpace<K,N,MK,MN>::qk(int order) const
{
  static typename IntegralStateSpace<K,N,MK,MN>::Value ret;
  eigen_utils::resize(ret, qDim());
  const typename IntegralStateSpace<K,N,MK,MN>::State& state = impl_.state();
  eigen_utils::copy_from_block(ret,state,order*qDim(),0,qDim(),1);
  return ret;
}


template<int K, int N, int MK, int MN>
inline int IntegralStateSpace<K,N,MK,MN>::xDim() const
{
  return initialized() ? eigen_utils::rows(impl_.x()) : -1;
}

template<int K, int N, int MK, int MN>
inline int IntegralStateSpace<K,N,MK,MN>::qDim() const
{
  return initialized() ? eigen_utils::rows(impl_.x())/2 : -1;
}

template<int K, int N, int MK, int MN>
inline int IntegralStateSpace<K,N,MK,MN>::uDim() const
{
  return eigen_utils::rows(impl_.u());
}

template<int K, int N, int MK, int MN>
inline int IntegralStateSpace<K,N,MK,MN>::yDim() const
{
  return eigen_utils::rows(impl_.y());
}
*/

}  // namespace eigen_control_toolbox

#endif  // state_space_systems_impl_201811280956
