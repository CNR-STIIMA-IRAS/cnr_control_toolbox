#ifndef STATE_SPACE_ROS__ROS_PARAMS_H
#define STATE_SPACE_ROS__ROS_PARAMS_H

#include <Eigen/Dense>
#include <ros/node_handle.h>
#include <state_space_systems/discrete_state_space_systems.h>
#include <state_space_systems/integral_state_space_systems.h>
#include <state_space_systems/integral_discrete_state_space_systems.h>
#include <state_space_controllers/controllers.h>

//! @brief The implementation of all the classes and functions for the management of a Discreate Space System
namespace eigen_control_toolbox
{

/**
 * @brief The function extract the Matrices descring the Discrete Space System.
 * The Discrete Space System describe the mathematical equations
 *
 * X(k) = A * X(k-1) + B*u(k)
 *
 * Y(k) = C * X(k-1) + D*u(k)
 *
 * where 'X', 'y', and 'u' are the state, the output and the input respectively
 *
 * NOTE: the function will be moved to an "utility" library, in order to remove the
 *       ROS dependency to the lib
 * @param[in] nh Node Handle
 * @param[in] name name of the namespace for the
 * @param[out] A Matrix A
 * @param[out] B Matrix B
 * @param[out] C Matrix C
 * @param[out] D Matrix D
 * @param[out] what a string with the error, if something wrong happen inside
 * @return bool false if the params are not found in the param server, true otherwise
 * */
bool importMatricesFromParam( const ros::NodeHandle&  nh,
                              const std::string&      name,
                              Eigen::MatrixXd&        A,
                              Eigen::MatrixXd&        B,
                              Eigen::MatrixXd&        C,
                              Eigen::MatrixXd&        D,
                              std::string&            what);

template<int S, int I, int O, int MS, int MI, int MO>
bool setMatricesFromParam(DiscreteStateSpace<S,I,O,MS,MI,MO>& out,
                          const ros::NodeHandle& nh,
                          const std::string& name,
                          std::string& what);

template<int K,int N,int MaxK=K,int MaxN=N>
bool setMatricesFromParam(IntegralStateSpace<K,N,MaxK,MaxN>& out,
                          const ros::NodeHandle& nh,
                          const std::string& name,
                          std::string& what);

template<int K,int N,int MaxK=K,int MaxN=N>
bool setMatricesFromParam(IntegralDiscreteStateSpace<K,N,MaxK,MaxN>& out,
                          const ros::NodeHandle& nh,
                          const std::string& name,
                          std::string& what);

template<int N, int MaxN=N>
bool setMatricesFromParam(Controller<N,MaxN>& out,
                          const ros::NodeHandle& nh,
                          const std::string& name,
                          std::string& what);
}



#include <state_space_ros/internal/ros_params_impl.h>

#endif // STATE_SPACE_ROS__ROS_PARAMS_H
