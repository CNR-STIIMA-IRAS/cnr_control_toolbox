#pragma once


#include <memory>
#include <Eigen/Core>
#include <eigen_matrix_utils/eigen_matrix_utils.h>



/*!
 *  \addtogroup eigen_control_toolbox
 *  @{
 */

//! @brief The implementation of all the classes and functions for the management of a Discreate Space System
namespace eigen_control_toolbox

{



/** 
 * @brief The DynamicSystem is the base class for future extension of the library
 */
class DynamicSystem : std::enable_shared_from_this<DynamicSystem>
{
private:

protected:
  Eigen::VectorXd state_;
  std::string name_;

  int xdim_;
  int ydim_;
  int udim_;


public:
  typedef std::shared_ptr<DynamicSystem> Ptr;
  typedef std::shared_ptr<DynamicSystem const> ConstPtr;
  
  DynamicSystem()=default;
  virtual ~DynamicSystem() = default;

  int xDim() const {return xdim_;}
  int uDim() const {return udim_;}
  int yDim() const {return ydim_;}

  std::string getName() const {return name_;}
  void setName(const std::string& str){name_=str;}

  /**
   * @brief Compute the dynamic function
   * \tparam x state
   * \tparam u input
   */
  virtual Eigen::VectorXd dynamicFcn(const Eigen::VectorXd& x,
                                     const Eigen::VectorXd& u
                                     ) const =0;


  /**
   * @brief Compute the outputfunction y=f(x,u)
   * \tparam x state
   * \tparam u input
   */
  virtual Eigen::VectorXd outputFcn(const Eigen::VectorXd& x,
                                    const Eigen::VectorXd& u
                                    ) const =0;


  virtual Eigen::VectorXd regularizeState(const Eigen::VectorXd& x) const =0;

  void setState(const Eigen::VectorXd& x)
  {
    assert(("State dimension is wrong",xdim_!=x.size()));

    state_=regularizeState(x);
  };
  const Eigen::VectorXd& getState() const
  {
    return state_;
  }


  //! print the matricies of the system
  friend std::ostream& operator<<(std::ostream&, const DynamicSystem&);
  friend std::string to_string(DynamicSystem&);
};

//! Shared Ptr definition
typedef DynamicSystem::Ptr DynamicSystemPtr;

//! Shared Ptr definition
typedef DynamicSystem::ConstPtr DynamicSystemConstPtr;


::std::ostream& operator<<(::std::ostream&, const eigen_control_toolbox::DynamicSystem&);
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
 * @brief The ContinuousDynamicSystem is a template for continuous dynamic system
 */
class ContinuousDynamicSystem : public DynamicSystem
{
private:

protected:
public:
  typedef std::shared_ptr<ContinuousDynamicSystem> Ptr;
  typedef std::shared_ptr<ContinuousDynamicSystem const> ConstPtr;

  ContinuousDynamicSystem()=default;
  virtual ~ContinuousDynamicSystem() = default;



  /**
   * @brief Compute the dynamic function dx/dt=f(x,u)
   * \tparam x state
   * \tparam u input
   */
  virtual Eigen::VectorXd dynamicFcn(const Eigen::VectorXd& x,
                                     const Eigen::VectorXd& u
                                     )const =0;


  /**
   * @brief Compute the outputfunction y=f(x,u)
   * \tparam x state
   * \tparam u input
   */
  virtual Eigen::VectorXd outputFcn(const Eigen::VectorXd& x,
                                    const Eigen::VectorXd& u
                                    )const =0;

};


/**
 * @brief The DiscreteDynamicSystem is a template for discrete dynamic system
 */
class DiscreteDynamicSystem : DynamicSystem
{
protected:
public:
  typedef std::shared_ptr<DiscreteDynamicSystem> Ptr;
  typedef std::shared_ptr<DiscreteDynamicSystem const> ConstPtr;

  DiscreteDynamicSystem()=default;
  virtual ~DiscreteDynamicSystem() = default;



  /**
   * @brief Compute the dynamic function x_new=f(x,u)
   * \tparam x state
   * \tparam u input
   */
  virtual Eigen::VectorXd dynamicFcn(const Eigen::VectorXd& x,
                                     const Eigen::VectorXd& u
                                     )const =0;


  /**
   * @brief Compute the outputfunction y=f(x,u)
   * \tparam x state
   * \tparam u input
   */
  virtual Eigen::VectorXd outputFcn(const Eigen::VectorXd& x,
                                    const Eigen::VectorXd& u
                                    )const =0;

};


class FirstOrderContinuousSystem: public ContinuousDynamicSystem
{
protected:
  double regressice_coef_;

public:
  FirstOrderContinuousSystem(const double& regressice_coef)
  {
    xdim_=1;
    state_.resize(xdim_);
    udim_=1;
    ydim_=1;
    regressice_coef_=regressice_coef;
  }

  Eigen::VectorXd dynamicFcn(const Eigen::VectorXd &x, const Eigen::VectorXd &u) const override
  {
    assert(x.size()==xdim_);
    assert(u.size()==udim_);

    return regressice_coef_*(x-u);
  }

  virtual Eigen::VectorXd regularizeState(const Eigen::VectorXd& x) const
  {
    return x;
  }

  Eigen::VectorXd outputFcn(const Eigen::VectorXd &x, const Eigen::VectorXd &u) const override
  {
    assert(x.size()==xdim_);
    assert(u.size()==udim_);
    return x;
  }

};

}




/*! @} End of Doxygen Groups*/


