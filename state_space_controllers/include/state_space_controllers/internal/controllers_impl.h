#pragma once  //workaroud qtcreator clang-tidy

#ifndef STATE_SPACE_CONTROLLERS__CONTROLLERS_IMPL_H
#define STATE_SPACE_CONTROLLERS__CONTROLLERS_IMPL_H

#include <ros/console.h>
//#include <rosparam_utilities/rosparam_utilities.h>
#include <state_space_controllers/controllers.h>

namespace eu = eigen_utils;

namespace eigen_control_toolbox
{


template<int N, int MN>
inline bool Controller<N,MN>::setAntiWindupMatrix(const typename Controller<N,MN>::MatrixN& Baw, std::string& what)
{
  if(this->xDim()==0)
  {
    what = std::string(__PRETTY_FUNCTION__) + std::string(":") + std::to_string(__LINE__) + std::string(":")
         + " The system has not been yet initialized. Abort.";
    return false;
  }
  
  if(eu::rows(Baw)!=this->xDim())
  {
    what = std::string(__PRETTY_FUNCTION__) + std::string(":") + std::to_string(__LINE__) + std::string(":")
         + " The input dimension mismatches with the system dimension. Abort.";
    return false;
  }
  
  if(eu::rows(Baw)!=eu::cols(Baw))
  {
    what = std::string(__PRETTY_FUNCTION__) + std::string(":") + std::to_string(__LINE__) + std::string(":")
         + " The input matrix must be a square matrix.";
    return false;
  }  
  
  m_Baw = Baw;
  return true;
}

template<int N, int MN>
inline bool Controller<N, MN>::setMatrices(const BaseStateSpaceArgs& args, std::string& what)
{
  try
  {
    what = "";
    bool ok = eigen_control_toolbox::DiscreteStateSpace<N,N,N,MN,MN,MN>::setMatrices(args, what);
    if(!ok)
    {
      return false;
    }

    const ControllerStateSpaceArgs<N,MN>& _args = dynamic_cast< const ControllerStateSpaceArgs<N,MN>& >(args);
    m_Baw = _args.Baw;
  }
  catch(std::exception& e)
  {
    what += std::string(__PRETTY_FUNCTION__) + ": Caught an exception: " + std::string(e.what());
    return false;
  }
  return true;
}


template<int N, int MN>
inline void Controller<N,MN>::antiwindup(const Controller<N,MN>::Value& saturated_output, 
                                         const Controller<N,MN>::Value& unsaturated_output)
{
  this->m_aw = saturated_output-unsaturated_output;
  this->m_state+=this->m_Baw * m_aw;
}

template<int N, int MN>
inline bool Controller<N,MN>::setStateFromLastIO(const Controller<N,MN>::Value& inputs,
                                                 const Controller<N,MN>::Value& outputs)
{
  eu::checkInputDimAndThrowEx("setStateFromLastIO - Inputs", this->m_input, eu::rows(inputs), eu::cols(inputs));
  eu::checkInputDimAndThrowEx("setStateFromLastIO - Outputs", this->m_output, eu::rows(outputs), eu::cols(outputs));

  if(eu::norm(this->m_C)>0.0)
  {
    if(!eu::solve(this->m_state, this->m_C, outputs - this->m_D * inputs ) )
    {
      std::cerr << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": Pseudo inv failed." << std::endl;
      return false;
    }
  }
  else
  {
    eu::setZero(this->m_state);
  }

  this->m_input = inputs;
  this->m_output = outputs;
  return true;
}

template<int N, int MN>
inline bool Controller<N,MN>::setPI(const Controller<N,MN>::MatrixN &Kp,
                                   const Controller<N,MN>::MatrixN &Ki,
                                   const double& sampling_period,
                                   std::string& what)
{
  bool ret = true;
  what = "";

  ControllerStateSpaceArgs<N,MN> args;

  std::vector<std::pair<bool, std::string>> checks =
  {
    { (N==-1) && (eu::rows(Kp)==eu::rows(Kp)), "Kp matrix is not squared" },
    { (N==-1) && (eu::rows(Ki)==eu::rows(Ki)), "Ki matrix is not squared" },
    { (N==-1) && (eu::rows(Kp)==eu::cols(Ki)), "Kp and Ki matrixes are of different dimension" },
    { (N==-1) && eu::resize(args.A  , eu::rows(Kp), eu::cols(Kp)), "Error in resizing A  . Check memory allcoation"},
    { (N==-1) && eu::resize(args.B  , eu::rows(Kp), eu::cols(Kp)), "Error in resizing B  . Check memory allcoation"},
    { (N==-1) && eu::resize(args.C  , eu::rows(Kp), eu::cols(Kp)), "Error in resizing C  . Check memory allcoation"},
    { (N==-1) && eu::resize(args.D  , eu::rows(Kp), eu::cols(Kp)), "Error in resizing D  . Check memory allcoation"},
    { (N==-1) && eu::resize(args.Baw, eu::rows(Kp), eu::cols(Kp)), "Error in resizing Baw. Check memory allcoation"},
  };

  for(auto c : checks)
  {
    ret &= c.first;
    what += c.first ? "" : (what.size()>0?"\n":"") + c.second;
  }
  if(!ret)
  {
    return false;
  }

  args.D = Kp;
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

  std::string msg;
  ret = this->setMatrices(args,msg);
  what = ret ? msg : std::string(__PRETTY_FUNCTION__) + std::string(":") + std::to_string(__LINE__)
          + ": Failed in setting the matrices:" + msg;
  return ret;
}

}  // namespace  eigen_control_toolbox


#endif  // STATE_SPACE_CONTROLLERS__CONTROLLERS_IMPL_H
