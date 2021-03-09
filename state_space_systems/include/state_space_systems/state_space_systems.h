#ifndef   state_space_systems__state_space_systems_h
#define   state_space_systems__state_space_systems_h

#include <memory>
#include <Eigen/Core>
/*!
 *  \addtogroup eigen_control_toolbox
 *  @{
 */

//! @brief The implementation of all the classes and functions for the management of a Discreate Space System
namespace eigen_control_toolbox
{

template<typename MatrixA, typename MatrixC> 
bool computeObservabilityMatrix(double&          out,
                                const MatrixA&   A,    // S x S
                                const MatrixC&   C,    // O x S
                                const int&       N);

template<typename MatrixA, typename MatrixC, typename MatrixObs> 
bool computeObservabilityMatrix(Eigen::MatrixBase<MatrixObs>& out, 
                                const MatrixA&   A,    // S x S
                                const MatrixC&   C,    // O x S
                                const int&       N);

template<typename MatrixA, typename MatrixC, typename MatrixCtrl> 
bool computeControllabilityMatrix(MatrixCtrl&      out, 
                                  const MatrixA&   A,    // S x S
                                  const MatrixC&   B,    // S x I
                                  const int&       N);

/**
 * @return -1, if any error is raised (stored in msg), 0, if any warning is raised (stored in msg), 1 if ok 
 */
template<typename State0, typename MatrixA, typename MatrixB, typename MatrixC, typename MatrixD>
inline int estimateState0(State0& x0, 
                           const MatrixA& A,
                           const MatrixB& B,
                           const MatrixC& C,
                           const MatrixD& D,
                           const Eigen::VectorXd& inputs,
                           const Eigen::VectorXd& outputs,
                           std::string& msg);

struct BaseStateSpaceArgs
{
  typedef std::shared_ptr<BaseStateSpaceArgs> Ptr;
  typedef std::shared_ptr<BaseStateSpaceArgs const> ConstPtr;

  BaseStateSpaceArgs() = default;
  virtual ~BaseStateSpaceArgs() = default;
};


/** 
 * @brief The BaseStateSpace is the base class for future extension of the library
 * At this moment, only one class is inherited from the BaseStateSpace
 * The class implements the basic operations, without caring about the mathematical 
 * dimension and description of the State, Inputs and Outputs 
 */
class BaseStateSpace : std::enable_shared_from_this<BaseStateSpace>
{
private:
  BaseStateSpace(const BaseStateSpace&) = delete;
  BaseStateSpace& operator=(const BaseStateSpace&)= delete;
  BaseStateSpace(BaseStateSpace&&) = delete;
  BaseStateSpace& operator=(BaseStateSpace&&) = delete;

protected:
  double m_sampling_period;
  
public:
  typedef std::shared_ptr<BaseStateSpace> Ptr;
  typedef std::shared_ptr<BaseStateSpace const> ConstPtr;
  
  BaseStateSpace()=default;
  virtual ~BaseStateSpace() = default;

  virtual bool setMatrices(const BaseStateSpaceArgs& args, std::string& msg) = 0;

  virtual Eigen::Map<Eigen::VectorXd const> getX()  const = 0;
  virtual Eigen::Map<Eigen::VectorXd const> getY()  const = 0;
  virtual Eigen::Map<Eigen::VectorXd const> getU()  const = 0;
  virtual Eigen::Map<Eigen::MatrixXd const> getA()  const = 0;
  virtual Eigen::Map<Eigen::MatrixXd const> getB()  const = 0;
  virtual Eigen::Map<Eigen::MatrixXd const> getC()  const = 0;
  virtual Eigen::Map<Eigen::MatrixXd const> getD()  const = 0;

  //! accessor to the order (state dimension)
  [[deprecated("Use the xDim(), shorter :)")]]
  int getOrder() const { return  xDim(); }
  virtual int xDim() const = 0;

  //! accessor to the input dimension
  [[deprecated("Use the uDim(), shorter :)")]]
  int getNumberOfInputs() const { return uDim(); }
  virtual int uDim() const = 0;

  //! accessor to the output dimension
  [[deprecated("Use the yDim(), shorter :)")]]
  int getNumberOfOutputs()  const { return yDim(); }
  virtual int yDim() const = 0;

  [[deprecated("Use the dt(), shorter :)")]]
  void setSamplingPeriod(const double& sampling_period);      //!< Set the sampling period of the dicrete system
  const double& dt() const { return m_sampling_period; }

  [[deprecated("Use the dt(), shorter :)")]]
  double getSamplingPeriod() const;                           //!< Get the sampling period of the dicrete system
  double& dt() { return m_sampling_period; }

  //! print the matricies of the system
  friend std::ostream& operator<<(std::ostream&, const BaseStateSpace&);
  friend std::string to_string(BaseStateSpace&);
};

//! Shared Ptr definition
typedef BaseStateSpace::Ptr BaseStateSpacePtr;

//! Shared Ptr definition
typedef BaseStateSpace::ConstPtr BaseStateSpaceConstPtr;


::std::ostream& operator<<(::std::ostream&, const eigen_control_toolbox::BaseStateSpace&);
std::string to_string(const eigen_control_toolbox::BaseStateSpace&);

}




/*! @} End of Doxygen Groups*/


#include <state_space_systems/internal/state_space_systems_impl.h>

#endif
