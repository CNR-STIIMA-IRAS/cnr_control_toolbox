#ifndef   EIGEN_STATE_SPACE_SYSTEMS_INTEGRAL_DISCRETE_STATE_SPACE_SYSTEMS__H
#define   EIGEN_STATE_SPACE_SYSTEMS_INTEGRAL_DISCRETE_STATE_SPACE_SYSTEMS__H

#include <memory>
#include <Eigen/Core>
#include <state_space_systems/integral_state_space_systems.h>

/*!
 *  \addtogroup eigen_control_toolbox
 *  @{
 */

//! @brief The implementation of all the classes and functions for the management of a Discreate Space System
namespace eigen_control_toolbox
{

struct IntegralDiscreteStateSpaceArgs : private IntegralStateSpaceArgs
{
  typedef std::shared_ptr<IntegralDiscreteStateSpaceArgs> Ptr;
  typedef std::shared_ptr<IntegralDiscreteStateSpaceArgs const> ConstPtr;

  IntegralDiscreteStateSpaceArgs() = default;
  virtual ~IntegralDiscreteStateSpaceArgs() = default;

  int order = -1;  //Default values;
  int degrees_of_freedom = -1; //Default values;
  int dt = 0; //Default values;
};

template<typename D> bool defaultIntegralDiscreteStateSpaceMatrixA(Eigen::MatrixBase<D> const & matA, int order, int dof, double dt);
template<typename D> bool defaultIntegralDiscreteStateSpaceMatrixB(Eigen::MatrixBase<D> const & matB, int order, int dof, double dt);
template<typename D> bool defaultIntegralDiscreteStateSpaceMatrixC(Eigen::MatrixBase<D> const & matC, int order, int dof);
template<typename D> bool defaultIntegralDiscreteStateSpaceMatrixD(Eigen::MatrixBase<D> const & matD, int order, int dof);

/** 
 * @brief The Kinematic Space System describe the mathematical equations
 * 
 * q   = [I 0 0 ...] x
 * qd  = [0 I 0 ...] x
 * qdd = [0 0 I ...] x
 *
 * A  = [ I dt*I ]
 *      [ 0   I  ]
 * B  = [ 1/2*dt^2*I ]
 *
 * x(k) = A * x(k-1) + B*u(k);
 * 
 * y(k) = C * x(k)   + D*u(k);
 * 
 * where 'X', 'y', and 'u' are the state, the output and the input respectively
 * \tparam S State Dimension
 * \tparam I Input Dimension
 * \tparam O Output Dimension
 * \tparam MaxS = S if specified, it pre-allocates the max usable memory in the stack 
 * \tparam MaxI = I if specified, it pre-allocates the max usable memory in the stack 
 * \tparam MaxO = O if specified, it pre-allocates the max usable memory in the stack 
 */
template< int K, int N, int MaxK = K, int MaxN = N >  // g.t N if specified, it pre-allocates the max usable memory in the stack
class IntegralDiscreteStateSpace : public IntegralStateSpace<K,N,MaxK,MaxN>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Symbols = IntegralDiscreteStateSpaceSymbols<K,N,MaxK,MaxN>;

  using State   = typename Symbols::State;
  using Value   = typename Symbols::Value;
  using MatrixA = typename Symbols::MatrixA;
  using MatrixB = typename Symbols::MatrixB;
  using MatrixC = typename Symbols::MatrixC;
  using MatrixD = typename Symbols::MatrixD;

  IntegralDiscreteStateSpace();

  virtual ~IntegralDiscreteStateSpace() = default;
  IntegralDiscreteStateSpace(const IntegralDiscreteStateSpace&) = delete;
  IntegralDiscreteStateSpace& operator=(const IntegralDiscreteStateSpace&)= delete;
  IntegralDiscreteStateSpace(IntegralDiscreteStateSpace&&) = delete;
  IntegralDiscreteStateSpace& operator=(IntegralDiscreteStateSpace&&) = delete;

  IntegralDiscreteStateSpace(const IntegralDiscreteStateSpaceArgs& args);

  virtual bool setMatrices(const BaseStateSpaceArgs& args, std::string& msg) override;

protected:
  double dt_;
};

template<int N,int MN=N>
using IntegralDiscreteStateSpacePtr = std::shared_ptr<IntegralDiscreteStateSpace<N,MN> >;

typedef IntegralDiscreteStateSpace<-1,-1> IntegralDiscreteStateSpaceX;







}

/*! @} End of Doxygen Groups*/


#include <state_space_systems/internal/integral_discrete_state_space_systems_impl.h>

#endif
