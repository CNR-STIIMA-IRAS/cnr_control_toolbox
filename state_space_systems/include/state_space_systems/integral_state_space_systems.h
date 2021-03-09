#ifndef   EIGEN_STATE_SPACE_SYSTEMS_INTEGRAL_STATE_SPACE_SYSTEMS__H
#define   EIGEN_STATE_SPACE_SYSTEMS_INTEGRAL_STATE_SPACE_SYSTEMS__H

#include <memory>
#include <Eigen/Core>
#include <ros/node_handle.h>
#include <state_space_systems/state_space_systems.h>
#include <state_space_systems/discrete_state_space_systems.h>

/*!
 *  \addtogroup eigen_control_toolbox
 *  @{
 */

//! @brief The implementation of all the classes and functions for the management of a Discreate Space System
namespace eigen_control_toolbox
{

struct IntegralStateSpaceArgs : private BaseStateSpaceArgs
{
  typedef std::shared_ptr<IntegralStateSpaceArgs> Ptr;
  typedef std::shared_ptr<IntegralStateSpaceArgs const> ConstPtr;

  IntegralStateSpaceArgs() = default;
  virtual ~IntegralStateSpaceArgs() = default;

  int order = -1;  //Default values;
  int degrees_of_freedom = -1; //Default values;
};

template<typename D> bool defaultIntegralStateSpaceMatrixA(Eigen::MatrixBase<D> const & matA, int order, int dof);
template<typename D> bool defaultIntegralStateSpaceMatrixB(Eigen::MatrixBase<D> const & matB, int order, int dof);
template<typename D> bool defaultIntegralStateSpaceMatrixC(Eigen::MatrixBase<D> const & matC, int order, int dof);
template<typename D> bool defaultIntegralStateSpaceMatrixD(Eigen::MatrixBase<D> const & matD, int order, int dof);


/** 
 * @brief The function extract the Matrices descring the Integral Space System.
 * The Integral Space System describe the mathematical equations
 * 
 * X(k) = A * X(k-1) + B*u(k);
 * 
 * Y(k) = C * X(k)   + D*u(k);
 * 
 * where 'X', 'y', and 'u' are the state, the output and the input respectively
 * \tparam S State Dimension
 * \tparam I Input Dimension
 * \tparam O Output Dimension
 * \tparam MaxS = S if specified, it pre-allocates the max usable memory in the stack 
 * \tparam MaxI = I if specified, it pre-allocates the max usable memory in the stack 
 * \tparam MaxO = O if specified, it pre-allocates the max usable memory in the stack 
 */
template< int K,          // order of the system
          int N,          // State Dimension
          int MaxK = K,   //
          int MaxN = N >  //
class IntegralStateSpace :
    public DiscreteStateSpace<IntegralStateSpaceSymbols<K,N,MaxK,MaxN>::S, N, N,
                                IntegralStateSpaceSymbols<K,N,MaxK,MaxN>::MaxS, MaxN, MaxN>
{
public:

//  using StateSpace = DiscreteStateSpace<IntegralStateSpaceSymbols<K,N,MaxK,MaxN>::S, N, N,
//                                          IntegralStateSpaceSymbols<K,N,MaxK,MaxN>::MaxS, MaxN, MaxN>;
  using Symbols = IntegralStateSpaceSymbols<K,N,MaxK,MaxN>;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Value     = typename Symbols::Value;
  using State     = typename Symbols::State;
  using MatrixA   = typename Symbols::MatrixA;
  using MatrixB   = typename Symbols::MatrixB;
  using MatrixC   = typename Symbols::MatrixC;
  using MatrixD   = typename Symbols::MatrixD;

  IntegralStateSpace();

  virtual ~IntegralStateSpace() = default;
  IntegralStateSpace(const IntegralStateSpace&) = delete;
  IntegralStateSpace& operator=(const IntegralStateSpace&)= delete;
  IntegralStateSpace(IntegralStateSpace&&) = delete;
  IntegralStateSpace& operator=(IntegralStateSpace&&) = delete;

  /**
   *  @brief The constructor get dynamic allocated matrixes as input
   *  In the case the template is dynamic (e.g., S==-1, or I==-1, or O==-1)
   *  the dimension of the matrix will define the dimension of the system
   *  
   *  In the case the template is static, the dimension of the matricies is checked 
   */
  template<int k=K,int n=N, std::enable_if_t<(k==-1)||(n==-1),int> =0 >
  IntegralStateSpace(const IntegralStateSpaceArgs& args);

  /**
   *  @brief The constructor get dynamic allocated matrixes as input
   *  In the case the template is dynamic (e.g., S==-1, or I==-1, or O==-1)
   *  the dimension of the matrix will define the dimension of the system
   *  
   *  In the case the template is static, the dimension of the matricies is checked 
   * 
   * @return -1 if any error (reported in msg), 0 if any warnings (reported in msg), 1 if ok
   */
  virtual bool setMatrices(const BaseStateSpaceArgs& args, std::string& msg) override;

  int qDim() const;

  void  setQk(const Value& q, int order);
  const Value& qk(int order) const;

};

template<int K,int N,int MK=K,int MN=N>
using IntegralStateSpacePtr = std::shared_ptr<IntegralStateSpace<K,N,MK,MN> >;

typedef IntegralStateSpace<-1,-1> IntegralStateSpaceX;

}

/*! @} End of Doxygen Groups*/


#include <state_space_systems/internal/integral_state_space_systems_impl.h>

#endif
