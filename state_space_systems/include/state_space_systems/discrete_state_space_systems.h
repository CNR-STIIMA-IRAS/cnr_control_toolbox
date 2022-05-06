#ifndef   state_space_systems_discrete_state_space_systems__h
#define   state_space_systems_discrete_state_space_systems__h

#include <memory>
#include <Eigen/Core>
#include <ros/node_handle.h>
#include <state_space_systems/symbols.h>
#include <state_space_systems/state_space_systems.h>

/*!
 *  \addtogroup eigen_control_toolbox
 *  @{
 */

//! @brief The implementation of all the classes and functions for the management of a Discreate Space System
namespace eigen_control_toolbox
{

template< int S,          // State Dimension
          int I,          // Input Dimension
          int O,          // Output Dimension
          int MaxS = S,   // g.t S if specified, it pre-allocates the max usable memory in the stack
          int MaxI = I,   // g.t I if specified, it pre-allocates the max usable memory in the stack
          int MaxO = O >  // g.t O if specified, it pre-allocates the max usable memory in the stack
struct DiscreteStateSpaceArgs : public BaseStateSpaceArgs
{
  typedef std::shared_ptr<DiscreteStateSpaceArgs> Ptr;
  typedef std::shared_ptr<DiscreteStateSpaceArgs const> ConstPtr;

  DiscreteStateSpaceArgs() = default;
  virtual ~DiscreteStateSpaceArgs() = default;

  typename StateSpaceSymbols<S,I,O,MaxS,MaxI,MaxO>::MatrixA A;
  typename StateSpaceSymbols<S,I,O,MaxS,MaxI,MaxO>::MatrixB B;
  typename StateSpaceSymbols<S,I,O,MaxS,MaxI,MaxO>::MatrixC C;
  typename StateSpaceSymbols<S,I,O,MaxS,MaxI,MaxO>::MatrixD D;
};


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
class DiscreteStateSpace : public BaseStateSpace
{

public:
  using Symbols = StateSpaceSymbols<S,I,O,MaxS,MaxI,MaxO>;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using State     = typename Symbols::State;
  using Input     = typename Symbols::Input;
  using Output    = typename Symbols::Output;
  using MatrixA   = typename Symbols::MatrixA;
  using MatrixB   = typename Symbols::MatrixB;
  using MatrixC   = typename Symbols::MatrixC;
  using MatrixD   = typename Symbols::MatrixD;
  using MatrixCtrl= typename Symbols::MatrixCtrl;
  using MatrixObs = typename Symbols::MatrixObs;
  using MatrixI2O = typename Symbols::MatrixI2O;

  DiscreteStateSpace()=default;
  virtual ~DiscreteStateSpace() = default;
  DiscreteStateSpace(const DiscreteStateSpace&) = delete;
  DiscreteStateSpace& operator=(const DiscreteStateSpace&)= delete;
  DiscreteStateSpace(DiscreteStateSpace&&) = delete;
  DiscreteStateSpace& operator=(DiscreteStateSpace&&) = delete;

  /**
   *  @brief The constructor get dynamic allocated matrixes as input
   *  In the case the template is dynamic (e.g., S==-1, or I==-1, or O==-1)
   *  the dimension of the matrix will define the dimension of the system
   *  
   *  In the case the template is static, the dimension of the matricies is checked 
   */
  DiscreteStateSpace(const DiscreteStateSpaceArgs<S,I,O,MaxS,MaxI,MaxO>& args);

  /**
   *  @brief The constructor get dynamic allocated matrixes as input
   *  In the case the template is dynamic (e.g., S==-1, or I==-1, or O==-1)
   *  the dimension of the matrix will define the dimension of the system
   *  
   *  In the case the template is static, the dimension of the matricies is checked 
   * 
   * @return -1 if any error (reported in msg), 0 if any warnings (reported in msg), 1 if ok
   */
  virtual bool setMatrices (const BaseStateSpaceArgs& args, std::string& msg) override;
  virtual Eigen::Map<Eigen::MatrixXd const> getA() const override;
  virtual Eigen::Map<Eigen::MatrixXd const> getB() const override;
  virtual Eigen::Map<Eigen::MatrixXd const> getC() const override;
  virtual Eigen::Map<Eigen::MatrixXd const> getD() const override;
  virtual Eigen::Map<Eigen::VectorXd const> getX() const override;
  virtual Eigen::Map<Eigen::VectorXd const> getY() const override;
  virtual Eigen::Map<Eigen::VectorXd const> getU() const override;

  //! accessor
  [[deprecated("Use the A(), shorter :)")]]
  const MatrixA& getAMatrix() const;
  const MatrixA& A() const;

  //! accessor
  [[deprecated("Use the B(), shorter :)")]]
  const MatrixB& getBMatrix() const;
  const MatrixB& B() const;

  //! accessor
  [[deprecated("Use the getC(), shorter :)")]]
  const MatrixC& getCMatrix() const;
  const MatrixC& C() const;

  //! accessor
  [[deprecated("Use the D(), shorter :)")]]
  const MatrixD& getDMatrix() const;
  const MatrixD& D() const;

  //! accessor to last input
  [[deprecated("Use the u(), shorter :)")]]
  const Input& getInput()  const;
  const Input& u() const;

  //! accessor to last comupted output
  [[deprecated("Use the y(), shorter :)")]]
  const Output& getOutput() const;
  const Output& y() const;

  //! accessor to the internal state
  [[deprecated("Use the x(), shorter :)")]]
  const State& getState() const;
  const State& x() const;

  //! accessor to the internal state
  const MatrixObs&  getObservability() const;

  //! accessor to the internal state
  const MatrixCtrl&  getControllability() const;

  //! accessor to the order (state dimension)
  [[deprecated("Use the xDim(), shorter :)")]]
  int getOrder() const;
  int xDim() const override;

  //! accessor to the input dimension
  [[deprecated("Use the uDim(), shorter :)")]]
  int getNumberOfInputs() const;
  int uDim() const override;

  //! accessor to the output dimension
  [[deprecated("Use the yDim(), shorter :)")]]
  int getNumberOfOutputs()  const;
  int yDim() const override;
  
  /**
   * @brief set the state. The dimension is checked
   * NOTE: the input can be a 'double' or a Eigen::Matrix according to the template definition
   * If the State dimension is 1, the tyep 'State' refers to a double
   */
  void  setState(const State& state);

  //! print the matricies of the system
  void print();

  /**
   * @brief Compute state from past inputs and outputs during a time window large as the state
   * @param past_inputs the vector should be longer thatn S x I
   * @param past_outputs the vector should be longer than S x O, and equally long as past_inputs
   * @return true if everithing is ok, false if the argumnet dimension is wrong, of the inputs/outpus are
   *         linearlly dependent and so it is not possibile to estimate the inner state.
   */
  virtual bool setStateFromIO(const Eigen::VectorXd& past_inputs, const Eigen::VectorXd& past_outputs);
  
  /**
   * @brief Compute state from input and the output
   */
  bool setStateFromLastIO(const Input& inputs, const Output& outputs);
  
  /**
   * @brief update the system, to be called at each sampling iteration
   * @param input The type of the input is an Eigen::Matrix<double,I,1> if I is different from 1
   *              while is a 'doube' if I is equal to 1
   * @return output The type of the input is an Eigen::Matrix<double,I,1> if I is different from 1
   *              while is a 'doube' if I is equal to 1
   */
  virtual Output& update(const Input& input, bool skipSymbols_check = true);
  
  //! @brief compute the input to output matrix
  const MatrixI2O& computeInputToOutputMatrix( );

  bool initialized();

protected:
  State  m_state;   //< To store the state, and update at each cycle
  Input  m_input;   //< To store only the dimension of the input.
  Output m_output;  //< To store only the dimension of the output.
  
  MatrixA m_A;
  MatrixB m_B;
  MatrixC m_C;
  MatrixD m_D;
  //====
  //Obs = | C         |
  //      | C A       | 
  //      | ...       | 
  //      | C A^(n-1) | with n>=order / size of the state
  //====
  MatrixObs m_Obs;
  MatrixCtrl m_Ctrl;
  MatrixI2O m_i2o;
};

template<int S,int I,int O,int MS=S,int MI=I,int MO=O>
using DiscreteStateSpacePtr = std::shared_ptr<DiscreteStateSpace<S,I,O,MS,MI,MO> >;

typedef DiscreteStateSpace<-1,-1,-1> DiscreteStateSpaceX;




}

/*! @} End of Doxygen Groups*/


#include <state_space_systems/internal/discrete_state_space_systems_impl.h>

#endif
