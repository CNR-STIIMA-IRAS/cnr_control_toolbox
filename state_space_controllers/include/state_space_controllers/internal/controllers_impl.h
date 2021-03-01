#pragma once  //workaroud qtcreator clang-tidy

#ifndef STATE_SPACE_CONTROLLERS__CONTROLLERS_IMPL_H
#define STATE_SPACE_CONTROLLERS__CONTROLLERS_IMPL_H

#include <ros/console.h>
#include <rosparam_utilities/rosparam_utilities.h>
#include <state_space_controllers/controllers.h>

namespace eu = eigen_utils;

namespace eigen_control_toolbox
{


template<int N, int MN>
inline int Controller<N,MN>::setAntiWindupMatrix(const typename Controller<N,MN>::MatrixN& Baw, std::string& what)
{
  if(this->xDim()==0)
  {
    what = std::string(__PRETTY_FUNCTION__) + std::string(":") + std::to_string(__LINE__) + std::string(":")
         + " The system has not been yet initialized. Abort.";
    return -1;
  }
  
  if(eu::rows(Baw)!=this->xDim())
  {
    what = std::string(__PRETTY_FUNCTION__) + std::string(":") + std::to_string(__LINE__) + std::string(":")
         + " The input dimension mismatches with the system dimension. Abort.";
    return -1;
  }
  
  if(eu::rows(Baw)!=eu::cols(Baw))
  {
    what = std::string(__PRETTY_FUNCTION__) + std::string(":") + std::to_string(__LINE__) + std::string(":")
         + " The input matrix must be a square matrix.";
    return -1;
  }  
  
  m_Baw = Baw;
  return 1;
}

template<int N, int MN>
inline int Controller<N,MN>::setMatrices(const BaseStateSpaceArgs& args, std::string& what)
{
  what = "";
  int ret = eigen_control_toolbox::DiscreteStateSpace<N,N,N,MN,MN,MN>::setMatrices(args, what);
  if(ret!=1)
  {
    what += "\n" + std::string(__PRETTY_FUNCTION__) + ":" + std::to_string(__LINE__) + ":\n" +  what;
    if(ret==-1)
      return -1;
  }

  const ControllerStateSpaceArgs<N,MN>& _args = dynamic_cast< const ControllerStateSpaceArgs<N,MN>& >(args);
  m_Baw = _args.Baw;

  return what.size() > 0 ? 0 : 1;
}


template<int N, int MN>
inline void Controller<N,MN>::antiwindup(const Controller<N,MN>::Value& saturated_output, 
                                         const Controller<N,MN>::Value& unsaturated_output)
{
  this->m_aw = saturated_output-unsaturated_output;
  this->m_state+=this->m_Baw * m_aw;
}



template<int N, int MN>
inline int Controller<N,MN>::setPI(const Controller<N,MN>::MatrixN &Kp,
                                   const Controller<N,MN>::MatrixN &Ki,
                                   const double& sampling_period,
                                   std::string& what)
{
  ControllerStateSpaceArgs<N,MN> args;
  what = "";
  if(N==-1)
  {
    if(eu::rows(Kp)!=eu::rows(Kp))
    {
      what += "\n" + std::string(__PRETTY_FUNCTION__) + ":" + std::to_string(__LINE__) + ": Kp matrix must be square";
      return -1;
    }
    if(eu::rows(Ki)!=eu::cols(Ki))
    {
      what += "\n" + std::string(__PRETTY_FUNCTION__) + ":" + std::to_string(__LINE__) + ": Ki matrix must be square";
      return -1;
    }
    if(eu::rows(Kp)!=eu::cols(Ki))
    {
      what += "\n" + std::string(__PRETTY_FUNCTION__) + ":" + std::to_string(__LINE__) + ": Input matrixes are of different dimension";
      return -1;
    }
    eu::resize(args.A  , eu::rows(Kp), eu::cols(Kp));
    eu::resize(args.B  , eu::rows(Kp), eu::cols(Kp));
    eu::resize(args.C  , eu::rows(Kp), eu::cols(Kp));
    eu::resize(args.D  , eu::rows(Kp), eu::cols(Kp));
    eu::resize(args.Baw, eu::rows(Kp), eu::cols(Kp));
  }

  this->m_D = Kp;

  if(eu::norm(Ki)==0.0)
  {
    eu::setZero(args.A);
    eu::setZero(args.B);
    eu::setZero(args.C);
    eu::setZero(args.Baw);
  }
  else
  {
    MatrixN Ti_inv;
    if(eu::norm(Kp)==0.0)
    {
      eu::setDiagonal(Ti_inv,1);
    }
    else
    {
     Ti_inv = eu::div(Ki,Kp); //Ki/Kp <== (Ki=Kp/Ti -> 1/Ti=Ki/kp)
    }
    eu::setDiagonal(args.A,1.0);
    eu::setDiagonal(args.C,1.0);

    args.B = sampling_period*Ki;
    args.Baw = sampling_period*Ti_inv;
    args.D = Kp;
  }

  int ret = this->setMatrices(args,what);
  if(ret!=1)
  {
     what += std::string(__PRETTY_FUNCTION__) + std::string(":") + std::to_string(__LINE__)
          + ": Failed in setting the matrices.";
  }
  return ret;
}

}  // namespace  eigen_control_toolbox


#endif  // STATE_SPACE_CONTROLLERS__CONTROLLERS_IMPL_H
