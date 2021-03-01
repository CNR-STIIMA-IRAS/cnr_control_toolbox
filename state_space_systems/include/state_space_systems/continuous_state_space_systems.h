#ifndef   EIGEN_STATE_SPACE_SYSTEMS__CONTINUOUS_STATE_SPACE_SYSTEMS_H
#define   EIGEN_STATE_SPACE_SYSTEMS__CONTINUOUS_STATE_SPACE_SYSTEMS_H

#include <memory>
#include <Eigen/Core>
#include <ros/node_handle.h>
#include <state_space_systems/discrete_state_space_systems.h>

/*!
 *  \addtogroup eigen_control_toolbox
 *  @{
 */

//! @brief The implementation of all the classes and functions for the management of a Discreate Space System
namespace eigen_control_toolbox
{


/** 
 * @brief The function extract the Matrices descring the Continuous Space System.
 * The Continuous Space System describe the mathematical equations
 * 
 * X'(k) = A * X  + B*u(k);
 * Y(k) = C * X(k)+ D*u(k);
 * 
 * where 'X', 'y', and 'u' are the state, the output and the input respectively
 * \tparam S State Dimension
 * \tparam I Input Dimension
 * \tparam O Output Dimension
 * \tparam MaxS = S if specified, it pre-allocates the max usable memory in the stack 
 * \tparam MaxI = I if specified, it pre-allocates the max usable memory in the stack 
 * \tparam MaxO = O if specified, it pre-allocates the max usable memory in the stack 
 */
template< int S,          // State Dimension
          int I,          // Input Dimension  
          int O,          // Output Dimension
          int MaxS = S,   // g.t S if specified, it pre-allocates the max usable memory in the stack 
          int MaxI = I,   // g.t I if specified, it pre-allocates the max usable memory in the stack 
          int MaxO = O >  // g.t O if specified, it pre-allocates the max usable memory in the stack 
using ContinuousStateSpace = DiscreteStateSpace<S,I,O,MaxS,MaxI,MaxO>;

template<int S,int I,int O,int MS=S,int MI=I,int MO=O>
using ContinuousStateSpacePtr = std::shared_ptr<ContinuousStateSpace<S,I,O,MS,MI,MO> >;

typedef ContinuousStateSpace<-1,-1,-1> ContinuousStateSpaceX;

}

/*! @} End of Doxygen Groups*/


#endif
