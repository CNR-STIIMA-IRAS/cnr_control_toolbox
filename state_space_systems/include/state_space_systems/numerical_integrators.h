#pragma once

#include <state_space_systems/dynamical_systems.h>

/*!
 *  \addtogroup eigen_control_toolbox
 *  @{
 */

//! @brief The implementation of all the classes and functions for the management of a Discrete Space System
namespace eigen_control_toolbox
{

/**
 * @brief Abstract base class for an Integrator.
 */
class Integrator : std::enable_shared_from_this<DynamicSystem>
{
public:
  typedef std::shared_ptr<Integrator> Ptr; ///< Shared pointer type
  typedef std::shared_ptr<Integrator const> ConstPtr; ///< Shared pointer to const type

  /**
   * @brief Default constructor for Integrator.
   */
  Integrator() = default;

  /**
   * @brief Virtual method to integrate a continuous dynamic system.
   * @param system Continuous dynamic system.
   * @param state Current state.
   * @param input System input.
   * @param step_time Time step for integration.
   * @return Integrated state.
   */
  virtual Eigen::VectorXd integrate(const ContinuousDynamicSystem& system,
                                    const Eigen::VectorXd& state,
                                    const Eigen::VectorXd& input,
                                    const double& step_time) = 0;
};

/**
 * @brief Implementation of the Runge-Kutta 4th order integrator.
 */
class RungeKutta4 : public Integrator
{
public:
  /**
   * @brief Default constructor for RungeKutta4.
   */
  RungeKutta4() = default;

  /**
   * @brief Integrate the continuous dynamic system using the Runge-Kutta 4th order method.
   * @param system Continuous dynamic system.
   * @param state Current state.
   * @param input System input.
   * @param step_time Time step for integration.
   * @return Integrated state.
   */
  virtual Eigen::VectorXd integrate(const ContinuousDynamicSystem& system,
                                    const Eigen::VectorXd& state,
                                    const Eigen::VectorXd& input,
                                    const double& step_time) override
  {
    Eigen::VectorXd der1 = system.dynamicFcn(state, input);
    Eigen::VectorXd der2 = system.dynamicFcn(state + der1 * step_time * 0.5, input);
    Eigen::VectorXd der3 = system.dynamicFcn(state + der2 * step_time * 0.5, input);
    Eigen::VectorXd der4 = system.dynamicFcn(state + der3 * step_time, input);
    Eigen::VectorXd out = state + step_time / 6.0 * (der1 + 2.0 * der2 + 2.0 * der3 + der4);
    return out;
  }
};

/**
 * @brief Class to discretize a continuous dynamic system.
 */
class DiscretizedDynamicSystem : public DiscreteDynamicSystem
{
protected:
  ContinuousDynamicSystem::Ptr continuous_system_; ///< Pointer to the continuous dynamic system
  Integrator::Ptr integrator_; ///< Pointer to the integrator
  double step_time_; ///< Time step for discretization

public:
  /**
   * @brief Constructor for DiscretizedDynamicSystem.
   * @param continuous_system Pointer to the continuous dynamic system.
   * @param integrator Pointer to the integrator.
   * @param step_time Time step for discretization.
   */
  DiscretizedDynamicSystem(const ContinuousDynamicSystem::Ptr& continuous_system,
                           const Integrator::Ptr& integrator,
                           const double& step_time)
    : continuous_system_(continuous_system),
      integrator_(integrator),
      step_time_(step_time)
  {
    udim_ = continuous_system->uDim();
    xdim_ = continuous_system->xDim();
    ydim_ = continuous_system->yDim();
  }

  /**
   * @brief Compute the dynamic function for the discretized system.
   * @param x State vector.
   * @param u Input vector.
   * @return Result of the dynamic function.
   */
  Eigen::VectorXd dynamicFcn(const Eigen::VectorXd &x, const Eigen::VectorXd &u) const override
  {
    return integrator_->integrate(*continuous_system_, x, u, step_time_);
  }

  /**
   * @brief Compute the output function for the discretized system.
   * @param x State vector.
   * @param u Input vector.
   * @return Result of the output function.
   */
  Eigen::VectorXd outputFcn(const Eigen::VectorXd &x, const Eigen::VectorXd &u) const override
  {
    return continuous_system_->outputFcn(x, u);
  }

  /**
   * @brief Regularize the state vector.
   * @param x State vector.
   * @return Regularized state vector.
   */
  virtual Eigen::VectorXd regularizeState(const Eigen::VectorXd& x) const
  {
    return continuous_system_->regularizeState(x);
  }
};

}

/*! @} End of Doxygen Groups*/
