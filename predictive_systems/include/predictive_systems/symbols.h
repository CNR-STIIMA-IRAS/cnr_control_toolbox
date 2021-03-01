#ifndef EIGEN_PREDICTIVE_SYSTEMS__SYMBOLS_H
#define EIGEN_PREDICTIVE_SYSTEMS__SYMBOLS_H

#include <memory>
#include <type_traits>
#include <Eigen/Core>

namespace eigen_control_toolbox
{


/**
 * @brief The function extract the Matrices descring the Discrete Space System.
 * The Discrete Space System describe the mathematical equations
 *
 * y(k) = A * y(k-1) + B*u(k);
 *
 * y(k) = C * y(k)   + D*u(k);
 *
 *     | y(k)    |
 * Y = |         |
 *     | y(k+Np) |
 *
 *
 *     | u(k)    |
 * U = |         |
 *     | u(k+Np) |
 *
 *
 * Y = MatrixObservability * x(k) + H * U
 *
 * G is called Free Response
 * H is called Forced Response
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
          int Np,         // Number of Predictions
          int Nc,         // Number of Control inputs
          int MaxS = S,   // g.t S if specified, it pre-allocates the max usable memory in the stack
          int MaxI = I,   // g.t I if specified, it pre-allocates the max usable memory in the stack
          int MaxO = O,   // g.t O if specified, it p re-allocates the max usable memory in the stack
          int MaxNp= Np,  // g.t Np if specified, it pre-allocates the max usable memory in the stack
          int MaxNc= Nc>  // g.t Nc if specified, it pre-allocates the max usable memory in the stack
struct PredictiveModelSymbols
{
  // some useful constant at compile time are introduced
  enum { S_RC  = Eigen::ColMajor,
         I_RC  = Eigen::ColMajor,
         O_RC  = Eigen::ColMajor,
         A_RC  = Eigen::ColMajor,
         B_RC  = S==1 && (I>1||I==Eigen::Dynamic) ? Eigen::RowMajor : Eigen::ColMajor,
         C_RC  = O==1 && (S>1||S==Eigen::Dynamic) ? Eigen::RowMajor : Eigen::ColMajor,
         D_RC  = O==1 && (I>1||I==Eigen::Dynamic) ? Eigen::RowMajor : Eigen::ColMajor,
         S_MAX = S==Eigen::Dynamic ? MaxS : S,
         I_MAX = I==Eigen::Dynamic ? MaxI : I,
         O_MAX = O==Eigen::Dynamic ? MaxO : O,
         Np_MAX= Np==Eigen::Dynamic ? MaxNp : Np,
         Nc_MAX= Nc==Eigen::Dynamic ? MaxNc : Nc,
         // ==================================================
         IW    = (I  > 0 && Nc > 0 ? I*Nc : Eigen::Dynamic),                 // dimension of the input window
         OW    = (O  > 0 && Np > 0 ? O*Np : Eigen::Dynamic),                 // dimension of the output window
         IW_MAX= (I_MAX > 0 && Nc_MAX > 0 ? I_MAX*Nc_MAX : Eigen::Dynamic),  // dimension of the input window
         OW_MAX= (O_MAX > 0 && Np_MAX > 0 ? O_MAX*Np_MAX : Eigen::Dynamic),  // dimension of the output window
         IW_RC = Eigen::ColMajor,
         OW_RC = Eigen::ColMajor,
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

  //! Inputs: dimension I x Nc, if IW==1, then the Input type is a double
  using Inputs  = typename std::conditional<IW==1,
                    double, Eigen::Matrix<double,IW,1,I_RC,IW_MAX>>::type;

  //! Output: dimension O, if O==1, then the Output type is a double
  using Output  = typename std::conditional<O==1,
                    double, Eigen::Matrix<double,O,1,O_RC,O_MAX,1>>::type;

  //! Predictions: dimension O x Np, if O x Np ==1, then the Output type is a double
  using Predictions  = typename std::conditional<OW==1,
                    double, Eigen::Matrix<double,OW,1,OW_RC,OW_MAX,1>>::type;

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
                      double, Eigen::Matrix<double,OW,S,OBS_RC,OW_MAX,S_MAX>>::type;

  using MatrixFreeResponse = MatrixObs;

  //! Controllability Matrix: dimension (OxS) x I, if O==1, I==1 and S==1, then it reduces to a double
  //! Note: No static pre-allocation if S==-1 or IW==-1, since the matrix could be huge, and
  //! therefore it could easily overcome the stack maximum allocation given by Eigen
  using MatrixCtrl = typename std::conditional<OW==1 && IW==1,
                    double, Eigen::Matrix<double,S,IW,CTRL_RC>>::type;

  //! H: dimension (OxNp) x (IxNc)
  using MatrixForcedResponse = typename std::conditional<OW==1 && IW==1,
                                  double, Eigen::Matrix<double,OW,IW,I2O_RC,OW_MAX,IW_MAX>>::type;
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
          int Np,         // NUmber of Predictions
          int Nc,         // NUmber of Predictions
          int MaxK = K,   // g.t S if specified, it pre-allocates the max usable memory in the stack
          int MaxN = N,   // g.t I if specified, it pre-allocates the max usable memory in the stack
          int MaxNp= Np,  // g.t Np if specified, it pre-allocates the max usable memory in the stack
          int MaxNc= Nc   // g.t Nc if specified, it pre-allocates the max usable memory in the stack
>
struct IntegralPredictiveModelSymbols
{
  // some useful constant at compile time are introduced
  enum { OK    = ((K==-1)||(K>0)) && ((N==-1)||(N>0)),                              //!< check if template inputs are ok
         S     = (K==-1) || (N==-1) ? -1 : (K+1) * N,                                               //!< state dimension
         IW    = (N> 0 && Nc>0 ? N*Nc : Eigen::Dynamic),                              //!< dimension of the input window
         OW    = (N> 0 && Np>0 ? N*Np : Eigen::Dynamic),                             //!< dimension of the output window
         N_MAX = N ==Eigen::Dynamic ? MaxN : N,                                              //!< Max Degrees of Freedom
         K_MAX = K ==Eigen::Dynamic ? MaxK : K,                                                           //!< Max Order
         S_MAX = K_MAX==-1 || N_MAX==-1 ? -1 : (K_MAX+1) * N_MAX,                               //!< Max State Dimension
         Np_MAX= Np==Eigen::Dynamic ? MaxNp : Np,
         Nc_MAX= Nc==Eigen::Dynamic ? MaxNc : Nc,
         IW_MAX= (N_MAX > 0 && Nc_MAX > 0 ? N_MAX*Nc_MAX : Eigen::Dynamic),  // dimension of the input window
         OW_MAX= (N_MAX > 0 && Np_MAX > 0 ? N_MAX*Np_MAX : Eigen::Dynamic),  // dimension of the output window
         //==========================================================================
         B_RC  = S==1 && (N>1||N==Eigen::Dynamic) ? Eigen::RowMajor : Eigen::ColMajor,
         C_RC  = N==1 && (S>1||S==Eigen::Dynamic) ? Eigen::RowMajor : Eigen::ColMajor,
         OBS_RC = OW==1 && (S >1||S ==Eigen::Dynamic) ? Eigen::RowMajor : Eigen::ColMajor,
         CTRL_RC= S ==1 && (IW>1||IW==Eigen::Dynamic) ? Eigen::RowMajor : Eigen::ColMajor,
         I2O_RC = OW==1 && (IW>1||IW==Eigen::Dynamic) ? Eigen::RowMajor : Eigen::ColMajor,
  };

  //! Input && Output are Vectors of dimension equal to the degrees of freedom
  using Value  = typename std::conditional<N==1,
                    double, Eigen::Matrix<double,N,1,Eigen::ColMajor,N_MAX,1>>::type;

  //! Inputs: dimension I x Nc, if IW==1, then the Input type is a double
  using Inputs  = typename std::conditional<IW==1,
                    double, Eigen::Matrix<double,IW,1,Eigen::ColMajor,IW_MAX>>::type;

  //! Predictions: dimension O x Np, if O x Np ==1, then the Output type is a double
  using Predictions  = typename std::conditional<OW==1,
                    double, Eigen::Matrix<double,OW,1,Eigen::ColMajor,OW_MAX,1>>::type;

  using State   = Eigen::Matrix<double,S,1,Eigen::ColMajor,S_MAX>;                      //!< State: dimension (K+1) * N
  using MatrixA = Eigen::Matrix<double,S,S,Eigen::ColMajor,S_MAX,S_MAX>;                        //!< A: dimension S x S
  using MatrixB = Eigen::Matrix<double,S,N,B_RC,S_MAX,N_MAX>;                                   //!< B: dimension S x N
  using MatrixC = Eigen::Matrix<double,N,S,C_RC,N_MAX,S_MAX>;                                   //!< C: dimension N x S
  using MatrixD = Eigen::Matrix<double,N,N,Eigen::ColMajor,N_MAX,N_MAX>;          //!< D: dimension N x N, ALWAYS ZERO!

  using MatrixObs = Eigen::Matrix<double,OW,S,OBS_RC,OW_MAX,S_MAX>;
  using MatrixFreeResponse = MatrixObs;

  using MatrixCtrl = typename std::conditional<OW==1 && IW==1,
                    double, Eigen::Matrix<double,S,IW,CTRL_RC>>::type;

  //! H: dimension (OxNp) x (IxNc)
  using MatrixForcedResponse = typename std::conditional<OW==1 && IW==1,
                                  double, Eigen::Matrix<double,OW,IW,I2O_RC,OW_MAX,IW_MAX>>::type;
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
          int Np,         // NUmber of Predictions
          int Nc,         // NUmber of Predictions
          int MaxK = K,   // g.t S if specified, it pre-allocates the max usable memory in the stack
          int MaxN = N,   // g.t I if specified, it pre-allocates the max usable memory in the stack
          int MaxNp= Np,  // g.t Np if specified, it pre-allocates the max usable memory in the stack
          int MaxNc= Nc   // g.t Nc if specified, it pre-allocates the max usable memory in the stack
>
using IntegralDiscretePredictiveModelSymbols = IntegralPredictiveModelSymbols<K,N,Np,Nc,MaxK,MaxN,MaxNp,MaxNc>;



}  // namespace eigen_control_toolbox

#endif // EIGEN_PREDICTIVE_SYSTEMS__SYMBOLS_H


