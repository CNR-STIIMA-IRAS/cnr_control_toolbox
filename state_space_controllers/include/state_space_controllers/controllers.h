#ifndef STATE_SPACE_CONTROLLERS__CONTROLLERS_H
#define STATE_SPACE_CONTROLLERS__CONTROLLERS_H

#include <eigen_matrix_utils/overloads.h>
#include <state_space_systems/discrete_state_space_systems.h>

namespace eigen_control_toolbox
{

template< int N,          // Degrees of freedom
          int MaxN = N >  // g.t N if specified, it pre-allocates the max usable memory in the stack
struct ControllerStateSpaceArgs : public DiscreteStateSpaceArgs<N,N,N,MaxN,MaxN,MaxN>
{
  typedef std::shared_ptr<ControllerStateSpaceArgs > Ptr;
  typedef std::shared_ptr<ControllerStateSpaceArgs  const> ConstPtr;

  ControllerStateSpaceArgs () = default;
  virtual ~ControllerStateSpaceArgs () = default;

  typename StateSpaceSymbols<N,N,N,MaxN,MaxN,MaxN>::MatrixB Baw;
};

template< int N,
          int MaxN = N>  // g.t O if specified, it pre-allocates the max usable memory in the stack 
class Controller: public DiscreteStateSpace<N,N,N,MaxN,MaxN,MaxN>
{
public:
  using MatrixN = typename DiscreteStateSpace<N,N,N,MaxN,MaxN,MaxN>::MatrixA;
  using Value   = typename DiscreteStateSpace<N,N,N,MaxN,MaxN,MaxN>::Input;

  Controller() = default;
  Controller(const ControllerStateSpaceArgs<N,MaxN>& args);

  bool setMatrices(const BaseStateSpaceArgs& args, std::string&  what) override;

  //! If the dimension of the system if dynamic-size allocation, 
  //! the Baw dimension must be equalt to the system dimension
  //! if the system dimension is not yet defined, an error is returned
  //! @return false if errors, true if ok. Error(s)/warning(s) are reported in 'what'
  bool setAntiWindupMatrix(const MatrixN& Baw, std::string& what);

  bool setPI(const MatrixN& Kp,const MatrixN& Ki, const double& sampling_period, std::string& what);

  bool setStateFromLastIO(const Value& inputs, const Value& outputs);
  void antiwindup(const Value& saturated_output, const Value& unsaturated_output);

protected:
  MatrixN m_Baw;
  Value   m_aw;
};

template<int N, int MaxN>
using Value = typename eigen_control_toolbox::Controller<N,MaxN>::Value;

template<int N, int MaxN>
using MatrixN = typename eigen_control_toolbox::Controller<N,MaxN>::MatrixN;


}

#include <state_space_controllers/internal/controllers_impl.h>

#endif  // STATE_SPACE_CONTROLLERS__CONTROLLERS_H
