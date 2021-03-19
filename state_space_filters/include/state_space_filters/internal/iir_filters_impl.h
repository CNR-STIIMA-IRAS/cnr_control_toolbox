#pragma once // workaround qtcreator clang-tidy

#ifndef   STATE_SPACE_FILTERS__IIR_FILTERS_IMPL_H
#define   STATE_SPACE_FILTERS__IIR_FILTERS_IMPL_H

#include <type_traits>
#include <state_space_filters/iir_filters.h>

namespace eigen_control_toolbox
{

inline bool importMatricesFromParam(const ros::NodeHandle& nh, 
                                    const std::string& name, 
                                    double& natural_frequency,
                                    double& sampling_period,
                                    int&    channels)
{
  if(nh.hasParam(name+"/natural_frequency"))
  {
    if (!nh.getParam(name+"/natural_frequency",natural_frequency))
    {
      ROS_ERROR("%s/natural_frequency is not a double",name.c_str());
      return false;
    }
    if (natural_frequency<0)
    {
      ROS_ERROR("%s/natural_frequency should be positive",name.c_str());
      return false;
    }
  }
  else if (nh.hasParam(name+"/frequency"))
  {
    double frequency;
    if (!nh.getParam(name+"/frequency",frequency))
    {
      ROS_ERROR("%s/frequency is not a double",name.c_str());
      return false;
    }
    if (frequency<0)
    {
      ROS_ERROR("%s/frequency should be positive",name.c_str());
      return false;
    }
    natural_frequency=2*M_PI*frequency;
  }
  else 
  {
    ROS_ERROR("Neither %s/natural_frequency nor %s/frequency are defined",name.c_str(),name.c_str());
    return false;
  }
  
  if (nh.hasParam(name+"/sample_period"))
  {
    if (!nh.getParam(name+"/sample_period",sampling_period))
    {
      ROS_ERROR("%s/sample_period is not a double",name.c_str());
      return false;
    }
    if (sampling_period<0)
    {
      ROS_ERROR("%s/sample_period should be positive",name.c_str());
      return false;
    }
  }
  else 
  {
    ROS_ERROR("%s/sample_period is not defined",name.c_str());
    return false;
  }

  if (nh.hasParam(name+"/channels"))
  {
    if (!nh.getParam(name+"/channels",channels))
    {
      ROS_ERROR("%s/channels is not an int",name.c_str());
      return false;
    }
    if((channels<-1)||(channels==0))
    {
      ROS_ERROR("%s/channles should be -1, if to be allocated dynamically through the code, or >=1",name.c_str());
      return false;
    }
  }
  else 
  {
    channels = -1;
  }
  
  return true;
}



//========================================================
//
//
// template<int N, int MaxN>
// FirstOrderLowPass<N,MaxN>::FirstOrderLowPass()
//
//========================================================
template<int N, int MaxN>
inline FirstOrderLowPass<N,MaxN>::FirstOrderLowPass()
{
  this->m_natural_frequency=1;
  this->m_sampling_period=1;
  this->m_channels=N;
}

template<int N, int MaxN>
template<int n, std::enable_if_t< n==-1, int> >
inline FirstOrderLowPass<N,MaxN>::FirstOrderLowPass(const double& natural_frequency,
                                                    const double& sampling_period,
                                                    const int&    channels)
{
  if(!init(natural_frequency, sampling_period, channels))
  {
    throw std::runtime_error(
      ("Error in the ctor of the FirstOrderLowPass<N,MaxN> with N=" + std::to_string(N)).c_str());
  }
}

template<int N, int MaxN>
template<int n, std::enable_if_t< n!=-1&& n!=0, int> >
inline FirstOrderLowPass<N,MaxN>::FirstOrderLowPass(const double& natural_frequency,
                                                    const double& sampling_period)
{
  if(!init(natural_frequency, sampling_period, N))
  {
    throw std::runtime_error(
      ("Error in the ctor of the FirstOrderLowPass<N,MaxN> with N=" + std::to_string(N)).c_str());
  }
}

template<int N, int MaxN>
inline bool FirstOrderLowPass<N,MaxN>::init(const double& natural_frequency, const double& sampling_period, const int& channels)
{
  if(!initParam(natural_frequency, sampling_period))
  {
    return false;
  }  

  int ch = channels;
  if((N==-1)&&(channels<1))
  {
    std::cerr << __PRETTY_FUNCTION__ <<":" << __LINE__ << ":" << 
          "The dimension of the filter is undefined, and it is not solved by the input argument 'channels'. Abort." 
              << std::endl;
    return false;
  }
  else if((N!=-1)&&(channels!=N))
  {
    std::cerr << __PRETTY_FUNCTION__ <<":" << __LINE__ << ":" << 
          "The templated dimension of the filter mismatch with the input argument. Input argument 'channels' is ignored." 
              << std::endl;
    ch = N;
  }
  
  this->m_channels = ch;
  
  return computeMatrices();
}


template<int N, int MaxN>
inline bool FirstOrderLowPass<N,MaxN>::initParam(const double& natural_frequency, const double& sampling_period)
{
  if(sampling_period<=0)
  {
    std::cerr << __PRETTY_FUNCTION__ <<":" << __LINE__ << ":" << 
          "The sampling periodcannot be less than or equal to zero ." 
              << std::endl;
    return false;
  }
  else if(natural_frequency<=0)
  {
    std::cerr << __PRETTY_FUNCTION__ <<":" << __LINE__ << ":" << 
          "The natural frequency cannot be less than or equal to zero ." 
              << std::endl;
    return false;
  }
  else if(natural_frequency>=M_PI/sampling_period)
  {
    std::cerr << __PRETTY_FUNCTION__ <<":" << __LINE__ << ":" << 
          "The natural frequency cannot be less than or equal to zero ." 
              << std::endl;
    return false;
  }
  this->m_natural_frequency=natural_frequency;
  this->m_sampling_period=sampling_period;
  return true;
}

template<int N, int MaxN>
inline bool FirstOrderLowPass<N,MaxN>::computeMatrices()
{
  DiscreteStateSpaceArgs<N,N,N,MaxN,MaxN,MaxN> args;
  eu::resize(args.A,this->m_channels,this->m_channels);
  eu::resize(args.B,this->m_channels,this->m_channels);
  eu::resize(args.C,this->m_channels,this->m_channels);
  eu::resize(args.D,this->m_channels,this->m_channels);
  
  eu::setIdentity(args.A);
  eu::setIdentity(args.B);
  eu::setIdentity(args.C);
  eu::setZero(args.D);

  // xn=coef*x+(1-coef)*u
  // y =x;
  // Dy=(xn-x)/st=(coef-1)/st*x+(1-coef)/st*u
  
  double coef=std::exp(-this->m_sampling_period*this->m_natural_frequency);
  args.A *= coef;
  args.B *= (1-coef);
  
  std::string what;
  int ok = this->setMatrices(args,what);
  if(ok!=1)
  {
    std::cerr << __PRETTY_FUNCTION__ <<":"<<__LINE__<<":\n" << what << std::endl;
    if(ok==-1)
      return false;
  }
  return true;
}

template<int N, int MaxN>
inline bool FirstOrderLowPass<N,MaxN>::importMatricesFromParam(const ros::NodeHandle& nh, const std::string& name)
{
  double natural_frequency;
  double sampling_period;
  int channels;

  if(!eigen_control_toolbox::importMatricesFromParam(nh, name, natural_frequency, sampling_period, channels))
  {
    return false;
  }
  
  if(!this->init(natural_frequency, sampling_period, channels))
  {
    return false;
  }
  
  return computeMatrices( );
}


template<int N, int MaxN>
inline bool FirstOrderLowPass<N,MaxN>::setStateFromLastInput(const typename FirstOrderLowPass<N,MaxN>::Input& input)
{
  eu::checkInputDimAndThrowEx("setStateFromLastIO - Inputs", this->m_input, eu::rows(input), eu::cols(input));
  
  this->m_state = input;
  this->m_input = input;
  this->m_output = input;
  return true;
}



//========================================================
//
//
//
//========================================================
template<int N, int MaxN>
inline FirstOrderHighPass<N,MaxN>::FirstOrderHighPass()
{
  this->m_natural_frequency=1;
  this->m_sampling_period=1;
  this->m_channels=N;
}

template<int N, int MaxN>
inline FirstOrderHighPass<N,MaxN>::FirstOrderHighPass(const double& natural_frequency,
                                                      const double& sampling_period,
                                                      const int& channels)
{
  if(!init(natural_frequency, sampling_period, channels))
  {
    throw std::runtime_error(
      ("Error in the ctor of the FirstOrderLowPass<N,MaxN> with N=" + std::to_string(N)).c_str());
  }
}

template<int N, int MaxN>
inline bool FirstOrderHighPass<N,MaxN>::computeMatrices( )
{
  DiscreteStateSpaceArgs<N,N,N,MaxN,MaxN,MaxN> args;
  eu::resize(args.A,this->m_channels,this->m_channels);
  eu::resize(args.B,this->m_channels,this->m_channels);
  eu::resize(args.C,this->m_channels,this->m_channels);
  eu::resize(args.D,this->m_channels,this->m_channels);

  eu::setIdentity(args.A);
  eu::setIdentity(args.B);
  eu::setIdentity(args.C);
  eu::setZero(args.D);
  
  // xn=coef*x+(1-coef)*u
  // y=x;
  // Dy=(xn-x)/st=(coef-1)/st*x + (1-coef)/st*u
  
  double coef=std::exp(-this->m_sampling_period*this->m_natural_frequency);
  args.A *= coef;
  args.B *= (1-coef);
  args.C *= -coef/this->m_sampling_period;
  args.D *=  coef/this->m_sampling_period;
  
  std::string what;
  int ok = this->setMatrices(args,what);
  if(ok!=1)
  {
    std::cerr << __PRETTY_FUNCTION__ <<":"<<__LINE__<<":\n" << what << std::endl;
    if(ok==-1)
      return false;
  }
  return true;
}


template<int N, int MaxN>
inline bool FirstOrderHighPass<N,MaxN>::init( const double& natural_frequency,
                                              const double& sampling_period,
                                              const int& channels)
{
  if((N==-1)&&(channels==-1)) 
  {
    std::cerr << __PRETTY_FUNCTION__ <<":" << __LINE__ << ":\n\t\t" << 
          "The dimension of the filter is undefined, and it is not solved by the input argument 'channels'. Abort." 
              << std::endl;
    return false;
  }
  else if((N>0)&&(channels!=N)) 
  {
    std::cerr << __PRETTY_FUNCTION__ <<":" << __LINE__ << ":\n\t\t" << 
          "The input dimension differ from the templated dimension. The input dimension will be ignored." 
              << std::endl;
  }
  else if(sampling_period<=0)
  {
    std::cerr << __PRETTY_FUNCTION__ <<":" << __LINE__ << ":" << 
          "The sampling periodcannot be less than or equal to zero ." 
              << std::endl;
    return false;
  }
  else if(natural_frequency<=0)
  {
    std::cerr << __PRETTY_FUNCTION__ <<":" << __LINE__ << ":\n\t\t" << 
          "The natural frequency cannot be less than or equal to zero ." 
              << std::endl;
    return false;
  }

  this->m_natural_frequency=natural_frequency;
  this->m_sampling_period=sampling_period;
  this->m_channels = N > 0 ? N : channels;
  return computeMatrices( );
}


template<int N, int MaxN>
inline bool FirstOrderHighPass<N,MaxN>::importMatricesFromParam(const ros::NodeHandle& nh, const std::string& name)
{
  double natural_frequency;
  double sampling_period;
  int channels;

  if(!eigen_control_toolbox::importMatricesFromParam(nh, name, natural_frequency, sampling_period, channels))
  {
    return false;
  }
  if(!init(natural_frequency, sampling_period, channels))
  {
    return false;
  }

  return computeMatrices( );
}


template<int N, int MaxN>
inline bool FirstOrderHighPass<N,MaxN>::setStateFromLastInput(const typename FirstOrderHighPass<N,MaxN>::Input& input)
{
  eu::checkInputDimAndThrowEx("setStateFromLastIO - Inputs", this->m_input, eu::rows(input), eu::cols(input));
  
  this->m_state = input;
  this->m_input = input;
  this->m_output = input;
  return true;
}


}

#endif  // STATE_SPACE_FILTERS__IIR_FILTERS_IMPL_H

