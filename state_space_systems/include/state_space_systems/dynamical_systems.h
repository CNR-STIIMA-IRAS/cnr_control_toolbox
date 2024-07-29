#pragma once

#include <memory>
#include <Eigen/Core>
#include <eigen_matrix_utils/eigen_matrix_utils.h>

/*!
 *  \addtogroup eigen_control_toolbox
 *  @{
 */

//! @brief The implementation of all the classes and functions for the management of a Discrete Space System
namespace eigen_control_toolbox
{

/**
 * @brief Abstract base class for a Dynamic System.
 */
class DynamicSystem : std::enable_shared_from_this<DynamicSystem>
{
private:

protected:
  std::string name_; ///< system name
  int xdim_; ///< Dimension of the state
  int udim_; ///< Dimension of the input
  int ydim_; ///< Dimension of the output
  Eigen::VectorXd state_; ///< State vector

public:
  typedef std::shared_ptr<DynamicSystem> Ptr; ///< Shared pointer type
  typedef std::shared_ptr<DynamicSystem const> ConstPtr; ///< Shared pointer to const type

  /**
   * @brief Default constructor for DynamicSystem.
   */
  DynamicSystem()=default;

  /**
   * @brief Virtual destructor for DynamicSystem.
   */
  virtual ~DynamicSystem() = default;

  /**
   * @brief Get the dimension of the state.
   * @return Dimension of the state.
   */
  int xDim() const {return xdim_;}

  /**
   * @brief Get the dimension of the input.
   * @return Dimension of the input.
   */
  int uDim() const {return udim_;}

  /**
   * @brief Get the dimension of the output.
   * @return Dimension of the output.
   */
  int yDim() const {return ydim_;}

  /**
   * @brief Get the name of the system.
   * @return Name of the system.
   */
  std::string getName() const {return name_;}

  /**
   * @brief Set the name of the system.
   * @param str Name of the system.
   */
  void setName(const std::string& str){name_=str;}

  /**
   * @brief Compute the dynamic function.
   * @param x State vector.
   * @param u Input vector.
   * @return Result of the dynamic function.
   */
  virtual Eigen::VectorXd dynamicFcn(const Eigen::VectorXd& x,
                                     const Eigen::VectorXd& u
                                     ) const =0;

  /**
   * @brief Compute the output function y=f(x,u).
   * @param x State vector.
   * @param u Input vector.
   * @return Result of the output function.
   */
  virtual Eigen::VectorXd outputFcn(const Eigen::VectorXd& x,
                                    const Eigen::VectorXd& u
                                    ) const =0;

  /**
   * @brief Regularize the state vector.
   * @param x State vector.
   * @return Regularized state vector.
   */
  virtual Eigen::VectorXd regularizeState(const Eigen::VectorXd& x) const =0;

  /**
   * @brief Set the state vector.
   * @param x State vector.
   */
  void setState(const Eigen::VectorXd& x)
  {
    assert(("State dimension is wrong",xdim_!=x.size()));

    state_=regularizeState(x);
  };

  /**
   * @brief Get the state vector.
   * @return State vector.
   */
  const Eigen::VectorXd& getState() const
  {
    return state_;
  }

  /**
   * @brief Print the matrices of the system.
   * @param os Output stream.
   * @param bss Dynamic system.
   * @return Output stream.
   */
  friend std::ostream& operator<<(std::ostream&, const DynamicSystem&);

  /**
   * @brief Convert the dynamic system to a string.
   * @param bss Dynamic system.
   * @return String representation of the dynamic system.
   */
  friend std::string to_string(DynamicSystem&);
};

//! Shared Ptr definition
typedef DynamicSystem::Ptr DynamicSystemPtr;

//! Shared Ptr definition
typedef DynamicSystem::ConstPtr DynamicSystemConstPtr;

/**
 * @brief Output stream operator for DynamicSystem.
 * @param os Output stream.
 * @param bss Dynamic system.
 * @return Output stream.
 */
::std::ostream& operator<<(::std::ostream&, const eigen_control_toolbox::DynamicSystem&);

/**
 * @brief Convert DynamicSystem to string.
 * @param bss Dynamic system.
 * @return String representation.
 */
std::string to_string(const eigen_control_toolbox::DynamicSystem&);

inline ::std::ostream& operator<<(::std::ostream& os, const eigen_control_toolbox::DynamicSystem& bss)
{
  os << to_string(bss);
  return os;
}

inline std::string to_string(const eigen_control_toolbox::DynamicSystem& bss)
{
  std::stringstream ret;
  ret << std::string("System ") << bss.getName() << " with "
      << bss.uDim() << " inputs, "
      << bss.xDim() << " states, and "
      << bss.yDim() << "outputs" << std::endl;
  ret << "state:\n" << eigen_utils::to_string(bss.getState()) << std::endl;
  return ret.str();
}

/**
 * @brief The ContinuousDynamicSystem is a template for continuous dynamic systems.
 */
class ContinuousDynamicSystem : public DynamicSystem
{
private:

protected:
public:
  typedef std::shared_ptr<ContinuousDynamicSystem> Ptr; ///< Shared pointer type
  typedef std::shared_ptr<ContinuousDynamicSystem const> ConstPtr; ///< Shared pointer to const type

  /**
   * @brief Default constructor for ContinuousDynamicSystem.
   */
  ContinuousDynamicSystem()=default;

  /**
   * @brief Virtual destructor for ContinuousDynamicSystem.
   */
  virtual ~ContinuousDynamicSystem() = default;

  /**
   * @brief Compute the dynamic function dx/dt=f(x,u).
   * @param x State vector.
   * @param u Input vector.
   * @return Result of the dynamic function.
   */
  virtual Eigen::VectorXd dynamicFcn(const Eigen::VectorXd& x,
                                     const Eigen::VectorXd& u
                                     )const =0;


};

/**
 * @brief The DiscreteDynamicSystem is a template for discrete dynamic systems.
 */
class DiscreteDynamicSystem : public DynamicSystem
{
protected:
public:
  typedef std::shared_ptr<DiscreteDynamicSystem> Ptr; ///< Shared pointer type
  typedef std::shared_ptr<DiscreteDynamicSystem const> ConstPtr; ///< Shared pointer to const type

  /**
   * @brief Default constructor for DiscreteDynamicSystem.
   */
  DiscreteDynamicSystem()=default;

  /**
   * @brief Virtual destructor for DiscreteDynamicSystem.
   */
  virtual ~DiscreteDynamicSystem() = default;

  /**
   * @brief Compute the dynamic function x_new=f(x,u).
   * @param x State vector.
   * @param u Input vector.
   * @return Result of the dynamic function.
   */
  virtual Eigen::VectorXd dynamicFcn(const Eigen::VectorXd& x,
                                     const Eigen::VectorXd& u
                                     )const =0;


};

/**
 * @brief Class representing a first-order continuous system.
 */
class FirstOrderContinuousSystem: public ContinuousDynamicSystem
{
protected:
  double regressive_coef_; ///< Regressive coefficient

public:
  /**
   * @brief Constructor for FirstOrderContinuousSystem.
   * @param regressive_coef Regressive coefficient.
   */
  FirstOrderContinuousSystem(const double& regressive_coef)
  {
    xdim_=1;
    state_.resize(xdim_);
    udim_=1;
    ydim_=1;
    regressive_coef_=regressive_coef;
  }

  /**
   * @brief Implementation of the dynamic function.
   * @param x State vector.
   * @param u Input vector.
   * @return Result of the dynamic function.
   */
  Eigen::VectorXd dynamicFcn(const Eigen::VectorXd &x, const Eigen::VectorXd &u) const override
  {
    assert(x.size()==xdim_);
    assert(u.size()==udim_);

    return regressive_coef_*(x-u);
  }

  /**
   * @brief Regularize the state vector.
   * @param x State vector.
   * @return Regularized state vector.
   */
  virtual Eigen::VectorXd regularizeState(const Eigen::VectorXd& x) const
  {
    return x;
  }

  /**
   * @brief Implementation of the output function.
   * @param x State vector.
   * @param u Input vector.
   * @return Result of the output function.
   */
  Eigen::VectorXd outputFcn(const Eigen::VectorXd &x, const Eigen::VectorXd &u) const override
  {
    assert(x.size()==xdim_);
    assert(u.size()==udim_);
    return x;
  }

};

}

/*! @} End of Doxygen Groups */
