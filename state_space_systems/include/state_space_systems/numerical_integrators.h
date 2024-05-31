#pragma once


#include <state_space_systems/dynamical_systems.h>


/*!
 *  \addtogroup eigen_control_toolbox
 *  @{
 */

//! @brief The implementation of all the classes and functions for the management of a Discreate Space System
namespace eigen_control_toolbox

{

class Integrator : std::enable_shared_from_this<DynamicSystem>
{
public:
  typedef std::shared_ptr<Integrator> Ptr;
  typedef std::shared_ptr<Integrator const> ConstPtr;
  Integrator(){};
  virtual Eigen::VectorXd integrate(const ContinuousDynamicSystem& system,
                                    const Eigen::VectorXd& state,
                                    const Eigen::VectorXd& input,
                                    const double& step_time)=0;
};

class RungeKutta4: public Integrator
{
public:
  RungeKutta4(){};
  virtual Eigen::VectorXd integrate(const ContinuousDynamicSystem& system,
                                    const Eigen::VectorXd& state,
                                    const Eigen::VectorXd& input,
                                    const double& step_time)
  {
    Eigen::VectorXd der1=system.dynamicFcn(state                    , input);
    Eigen::VectorXd der2=system.dynamicFcn(state+der1*step_time*0.5 , input);
    Eigen::VectorXd der3=system.dynamicFcn(state+der2*step_time*0.5 , input);
    Eigen::VectorXd der4=system.dynamicFcn(state+der3*step_time     , input);
    Eigen::VectorXd out=state+step_time/6.0*(der1+2.0*der2+2.0*der3+der4);
    return out;

  }
};


class DiscretizedDynamicSystem: DiscreteDynamicSystem
{
protected:
  ContinuousDynamicSystem::Ptr continuous_system_;
  Integrator::Ptr integrator_;
  double step_time_;
public:


  DiscretizedDynamicSystem(const ContinuousDynamicSystem::Ptr& continuous_system,
                           const Integrator::Ptr& integrator,
                           const double& step_time):
    continuous_system_(continuous_system),
    integrator_(integrator),
    step_time_(step_time)
  {}

  Eigen::VectorXd dynamicFcn(const Eigen::VectorXd &x, const Eigen::VectorXd &u) const override
  {
    return integrator_->integrate(*continuous_system_,x,u,step_time_);
  }
  Eigen::VectorXd outputFcn(const Eigen::VectorXd &x, const Eigen::VectorXd &u) const override
  {
    return continuous_system_->outputFcn(x,u);
  }
   virtual Eigen::VectorXd regularizeState(const Eigen::VectorXd& x) const
  {
    return continuous_system_->regularizeState(x);
  }
};

}




/*! @} End of Doxygen Groups*/


