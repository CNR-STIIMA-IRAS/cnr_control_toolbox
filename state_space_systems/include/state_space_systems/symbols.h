#ifndef EIGEN_STATE_SPACE_SYSTEMS__SYMBOLS_H
#define EIGEN_STATE_SPACE_SYSTEMS__SYMBOLS_H

#include <memory>
#include <type_traits>
#include <Eigen/Core>

namespace eigen_control_toolbox
{


/**
 * @brief The function extract the Matrices descring the Discrete Space System.
 * The Discrete Space System describe the mathematical equations
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
template< int S,          // State Dimension
          int I,          // Input Dimension
          int O,          // Output Dimension
          int MaxS = S,   // g.t S if specified, it pre-allocates the max usable memory in the stack
          int MaxI = I,   // g.t I if specified, it pre-allocates the max usable memory in the stack
          int MaxO = O >  // g.t O if specified, it pre-allocates the max usable memory in the stack
struct StateSpaceSymbols
{
  // some useful constant at compile time are introduced
  enum { S_RC  = 0, // Row or Col Major of the State vector, it is always Eigen::ColMajor
         I_RC  = 0, // Row or Col Major of the Input vector, it is always Eigen::ColMajor
         O_RC  = 0, // Row or Col Major of the Output vector, it is always Eigen::ColMajor
         A_RC  = 0, // Row or Col Major of the A Matrix, it is always Eigen::ColMajor
         B_RC  = S==1 && (I>1||I==Eigen::Dynamic) ? Eigen::RowMajor : Eigen::ColMajor,
         C_RC  = O==1 && (S>1||S==Eigen::Dynamic) ? Eigen::RowMajor : Eigen::ColMajor,
         D_RC  = O==1 && (I>1||I==Eigen::Dynamic) ? Eigen::RowMajor : Eigen::ColMajor,
         S_MAX  = S ==Eigen::Dynamic ? MaxS : S,
         I_MAX  = I ==Eigen::Dynamic ? MaxI : I,
         O_MAX  = O ==Eigen::Dynamic ? MaxO : O,
         // ==================================================
         IW    = (I > 0 && S > 0 ? I*S : Eigen::Dynamic),               // dimension of the input window
         OW    = (O > 0 && S > 0 ? O*S : Eigen::Dynamic),               // dimension of the output window
         IW_RC = 0, // it is always Eigen::ColMajor
         OW_RC = 0, // it is always Eigen::ColMajor
         OBS_RC = OW==1 && (S >1||S ==Eigen::Dynamic) ? Eigen::RowMajor : Eigen::ColMajor, //
         CTRL_RC= S ==1 && (IW>1||IW==Eigen::Dynamic) ? Eigen::RowMajor : Eigen::ColMajor,
         I2O_RC = OW==1 && (IW>1||IW==Eigen::Dynamic) ? Eigen::RowMajor : Eigen::ColMajor,
        };

  //! State: dimension S, if S==1, then the State type is a double
  using State = typename std::conditional<S==1,
                    double, Eigen::Matrix<double,S,1,S_RC,S_MAX>>::type;

  //! Input: dimension I, if I==1, then the Input type is a double
  using Input   = typename std::conditional<I==1,
                    double, Eigen::Matrix<double,I,1,I_RC,I_MAX>>::type;

  //! Output: dimension O, if O==1, then the Output type is a double
  using Output  = typename std::conditional<O==1,
                    double, Eigen::Matrix<double,O,1,O_RC,O_MAX,1>>::type;

  //! A: dimension S x S, if S==1, then the matrix A type is a double
  using MatrixA = typename std::conditional<S==1,
                    double, Eigen::Matrix<double,S,S,A_RC,S_MAX,S_MAX>>::type;

  //! B: dimension S x I, if S==1 and I==1, then the matrix B type is a double
  using MatrixB = typename std::conditional<S==1 && I==1,
                    double, Eigen::Matrix<double,S,I,B_RC,S_MAX,I_MAX>>::type;

  //! C: dimension O x S, if O==1 and S==1, then the matrix C type is a double
  using MatrixC = typename std::conditional<O==1 && S==1,
                    double, Eigen::Matrix<double,O,S,C_RC,O_MAX,S_MAX>>::type;

  //! D: dimension O x I, if O==1 and I==1, then the matrix D type is a double
  using MatrixD = typename std::conditional<O==1 && I==1,
                    double, Eigen::Matrix<double,O,I,D_RC,O_MAX, I_MAX>>::type;

  //! Obesrvation Matrix: dimension (OxS) x I, if O==1, I==1 and S==1, then it reduces to a double
  //! Note: No static pre-allocation if S==-1 or IW==-1, since the matrix could be huge, and
  //! therefore it could easily overcome the stack maximum allocation given by Eigen
  using MatrixObs = typename std::conditional<OW==1 && S==1,
                    double, Eigen::Matrix<double,OW,S,OBS_RC>>::type;

  //! Controllability Matrix: dimension (OxS) x I, if O==1, I==1 and S==1, then it reduces to a double
  //! Note: No static pre-allocation if S==-1 or IW==-1, since the matrix could be huge, and
  //! therefore it could easily overcome the stack maximum allocation given by Eigen
  using MatrixCtrl = typename std::conditional<OW==1 && IW==1,
                    double, Eigen::Matrix<double,S,IW,CTRL_RC>>::type;

  //! Output to Input Matrix: dimension (OxS) x (IxS), if O==1, I==1 and S==1, then it reduces to a double
  //! Note: No static pre-allocation if S==-1 or IW==-1, since the matrix could be huge, and
  //! therefore it could easily overcome the stack maximum allocation given by Eigen
  using MatrixI2O = typename std::conditional<OW==1 && IW==1,
                    double, Eigen::Matrix<double,OW,IW,I2O_RC>>::type;
};

/**
 * @brief The function extract the Matrices descring the Discrete Space System.
 * The Discrete Space System describe the mathematical equations
 *
 * q = lagrangian variable of the system
 *
 * X  = [ q^T, qd^T, qdd^T, qddd^T ... qd^(k)^T ]^T   k-th order state of the system
 *
 * Xd(k) = A * X(k) + B*u(k);
 *
 * y(k)  = C * X(k) + D*u(k);
 *
 * y(k) == q(k)
 *
 * D(k) == 0
 * where 'X', 'y', and 'u' are the state, the output and the input respectively
 * \tparam K Order of the System
 * \tparam N Degrees of freedom of the system
 * \tparam O Output Dimension
 * \tparam MaxK = K if specified, it pre-allocates the max usable memory in the stack
 * \tparam MaxN = N if specified, it pre-allocates the max usable memory in the stack
 */
template< int K,          // system order >= 1
          int N,          // Degrees of freedom of the system -1 or >0
          int MaxK = K,   // g.t S if specified, it pre-allocates the max usable memory in the stack
          int MaxN = N    // g.t I if specified, it pre-allocates the max usable memory in the stack
>
struct IntegralStateSpaceSymbols
{
  // some useful constant at compile time are introduced
  enum { OK = ((K==-1)||(K>0)) && ((N==-1)||(N>0)),
         S  = (K==-1) || (N==-1) ? -1 : (K+1) * N,
         S_RC  = 0, // Row or Col Major of the State vector, it is always Eigen::ColMajor
         N_RC  = 0, // Row or Col Major of the Output vector, it is always Eigen::ColMajor
         A_RC  = 0, // Row or Col Major of the A Matrix, it is always Eigen::ColMajor
         B_RC  = S==1 && (N>1||N==Eigen::Dynamic) ? Eigen::RowMajor : Eigen::ColMajor,
         C_RC  = N==1 && (S>1||S==Eigen::Dynamic) ? Eigen::RowMajor : Eigen::ColMajor,
         N_MAX  = N ==Eigen::Dynamic ? MaxN : N,
         K_MAX  = K ==Eigen::Dynamic ? MaxK : K,
         S_MAX  = (K_MAX==-1) || (N_MAX==-1) ? -1 : (K_MAX+1) * N_MAX,
        };

  //! Vector of dimension equal to the degrees of freedom
  using Value  = typename std::conditional<N==1,
                    double, Eigen::Matrix<double,N,1,N_RC,N_MAX,1>>::type;


  typedef Eigen::Matrix<double,S,1,S_RC,S_MAX> State;                                   //!< State: dimension (K+1) * N
  typedef Eigen::Matrix<double,S,S,A_RC,S_MAX,S_MAX> MatrixA;                                   //!< A: dimension S x S
  typedef Eigen::Matrix<double,S,N,B_RC,S_MAX,N_MAX> MatrixB;                                   //!< B: dimension S x N
  typedef Eigen::Matrix<double,N,S,C_RC,N_MAX,S_MAX> MatrixC;                                   //!< C: dimension N x S
  typedef Eigen::Matrix<double,N,N,Eigen::ColMajor,N_MAX,N_MAX> MatrixD;           //! D: dimension N x N, ALWAYS ZERO!
};



/**
 * @brief The function extract the Matrices descring the Discrete Space System.
 * The Discrete Space System describe the mathematical equations
 *
 * q = lagrangian variable of the system
 *
 * X  = [ q^T, qd^T, qdd^T, qddd^T ... qd^(k)^T ]^T   k-th order state of the system
 *
 * X(k+1) = A * X(k) + B*u(k);
 *
 * y(k)  = C * X(k) + D*u(k);
 *
 * y(k) == q(k)
 *
 * D(k) == 0
 * where 'X', 'y', and 'u' are the state, the output and the input respectively
 * \tparam K Order of the System
 * \tparam N Degrees of freedom of the system
 * \tparam O Output Dimension
 * \tparam MaxK = K if specified, it pre-allocates the max usable memory in the stack
 * \tparam MaxN = N if specified, it pre-allocates the max usable memory in the stack
 */
template< int K,          // system order >= 1
          int N,          // Degrees of freedom of the system -1 or >0
          int MaxK = K,   // g.t S if specified, it pre-allocates the max usable memory in the stack
          int MaxN = N    // g.t I if specified, it pre-allocates the max usable memory in the stack
>
using IntegralDiscreteStateSpaceSymbols = IntegralStateSpaceSymbols<K,N,MaxK,MaxN>;



}  // namespace eigen_control_toolbox

#endif // EIGEN_STATE_SPACE_SYSTEMS__SYMBOLS_H


