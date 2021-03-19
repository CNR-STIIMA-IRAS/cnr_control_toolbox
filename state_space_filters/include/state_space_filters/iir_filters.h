#ifndef   STATE_SPACE_FILTERS__IIR_FILTERS_H
#define   STATE_SPACE_FILTERS__IIR_FILTERS_H

#include <type_traits>
#include <memory>
#include <Eigen/Core>
#include <ros/node_handle.h>
#include <state_space_systems/symbols.h>
#include <state_space_systems/discrete_state_space_systems.h>

namespace eigen_control_toolbox
{


bool importMatricesFromParam( const ros::NodeHandle& nh, 
                              const std::string& name, 
                              double& natural_frequency,
                              double& sampling_period,
                              int&    channels);
/*
 *     FirstOrderLowPass( const double& natural_frequency,
 *                        const double& sampling_period);
 */
template<int N, int MaxN = N>
class FirstOrderLowPass: public DiscreteStateSpace<N,N,N,MaxN,MaxN,MaxN>
{
protected:
  bool   initParam(const double& natural_frequency, const double& sampling_period);
  bool   computeMatrices();
  double m_natural_frequency;
  int    m_channels;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Symbols = StateSpaceSymbols<N,N,N,MaxN,MaxN,MaxN>;
  using Input   = typename Symbols::Input;
  using Output  = typename Symbols::Output;

  
  typedef std::shared_ptr<FirstOrderLowPass<N,MaxN>> Ptr;
  typedef std::shared_ptr<FirstOrderLowPass<N,MaxN> const> ConstPtr;

  FirstOrderLowPass();
  virtual ~FirstOrderLowPass() = default;

  template<int n=N, std::enable_if_t< n==-1, int> = 0>
  FirstOrderLowPass(const double& natural_frequency, const double& sampling_period, const int& channels);

  template<int n=N, std::enable_if_t< n!=-1 && n!=0, int> = 0>
  FirstOrderLowPass(const double& natural_frequency, const double& sampling_period);

  bool init(const double& natural_frequency, const double& sampling_period, const int& channels);

  [[deprecated("Use the Ctor, o the function 'init'. The dependency from ROS will be removed in the future")]]
  virtual bool importMatricesFromParam(const ros::NodeHandle& nh, const std::string& name);

  double getNaturalFrequency()const {return m_natural_frequency;}
  double getChannels() const {return m_channels;}

  [[deprecated("setStateFromLastIO: since it is a filter, you should call setStateFromLastInput()")]]
  virtual bool setStateFromLastIO(const Input& inputs, const Output& outputs) final 
  {
    return DiscreteStateSpace<N,N,N,MaxN,MaxN,MaxN>::setStateFromLastIO(inputs,outputs);
  }
  
  virtual bool setStateFromLastInput(const Input& inputs);
};

//!
template<int N, int MaxN = N> using FirstOrderLowPassPtr = typename FirstOrderLowPass<N, MaxN>::Ptr;
template<int N, int MaxN = N> using FirstOrderLowPassConstPtr = typename FirstOrderLowPass<N, MaxN>::ConstPtr;

//! 
typedef FirstOrderLowPass<-1> FirstOrderLowPassX;
typedef FirstOrderLowPass<-1>::Ptr FirstOrderLowPassXPtr;
typedef FirstOrderLowPass<-1>::ConstPtr FirstOrderLowPassXConstPtr;



/**
 * template
 */
template<int N, int MaxN = N>
class FirstOrderHighPass: public DiscreteStateSpace<N,N,N,MaxN,MaxN,MaxN>
{
protected:
  bool   computeMatrices();
  double m_natural_frequency;
  int    m_channels;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Symbols = StateSpaceSymbols<N,N,N,MaxN,MaxN,MaxN>;
  using Input   = typename Symbols::Input;
  using Output  = typename Symbols::Output;

  FirstOrderHighPass();
  ~FirstOrderHighPass() = default;
  
  FirstOrderHighPass(const double& natural_frequency, const double& sampling_period, const int& channels = N);
  
  virtual bool init(const double& natural_frequency, const double& sampling_period, const int& channels = N);

  [[deprecated("Use the Ctor, o the function 'init'. The dependency from ROS will be removed in the future")]]
  virtual bool importMatricesFromParam(const ros::NodeHandle& nh, const std::string& name);

  double getNaturalFrequency(){return m_natural_frequency;};

  [[deprecated("setStateFromLastIO: since it is a filter, you should call setStateFromLastInput()")]]
  virtual bool setStateFromLastIO(const Input& inputs, const Output& outputs) final 
  {
    return DiscreteStateSpace<N,N,N,MaxN,MaxN,MaxN>::setStateFromLastIO(inputs,outputs);
  }
  
  virtual bool setStateFromLastInput(const Input& inputs);
}; 


//!
template<int N, int MaxN> using FirstOrderHighPassPtr = typename FirstOrderHighPass<N, MaxN>::Ptr;
template<int N, int MaxN> using FirstOrderHighPassConstPtr = typename FirstOrderHighPass<N, MaxN>::ConstPtr;

//! 
typedef FirstOrderLowPass<-1> FirstOrderHighPassX;
typedef FirstOrderLowPass<-1>::Ptr FirstOrderHighPassXPtr;
typedef FirstOrderLowPass<-1>::ConstPtr FirstOrderHighPassXConstPtr;

}

#include <state_space_filters/internal/iir_filters_impl.h>

#endif  // STATE_SPACE_FILTERS__IIR_FILTERS_H
