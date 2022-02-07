#pragma once   // workaraund qtcreator ctidy-lang

#ifndef   state_space_systems_discrete_state_space_systems_impl_h
#define   state_space_systems_discrete_state_space_systems_impl_h

//#include <rosparam_utilities/rosparam_utilities.h>
#include <eigen_matrix_utils/eigen_matrix_utils.h>
#include <eigen_matrix_utils/overloads.h>

#include <state_space_systems/discrete_state_space_systems.h>


namespace eu = eigen_utils;
/**
 * 
 *
 * 
 * 
 * 
 */
namespace eigen_control_toolbox 
{


/**
 * 
 * 
 * 
 * 
 */
template<int S, int I, int O, int MS, int MI, int MO> 
inline DiscreteStateSpace<S,I,O,MS,MI,MO>::DiscreteStateSpace(const DiscreteStateSpaceArgs<S,I,O,MS,MI,MO>& args)
{
  std::string what;
  int ok = this->setMatrices(args,what);
  if(ok!=1)
  {
    if(ok==0)
    {
      std::cerr<<__PRETTY_FUNCTION__<<":"<<__LINE__<<": " << what << std::endl;
    }
    else
    {
      throw std::invalid_argument(("Error in memory management: "+what).c_str());
    }
  }
}


template<int S, int I, int O, int MS, int MI, int MO> 
inline bool DiscreteStateSpace<S, I, O, MS, MI, MO>::setMatrices(const BaseStateSpaceArgs& args, std::string& msg)
{
  try
  {
    msg ="";

    const DiscreteStateSpaceArgs<S,I,O,MS,MI,MO>& _args
        = dynamic_cast<const DiscreteStateSpaceArgs<S,I,O,MS,MI,MO>&>(args);

    // Check input coherence 
    if((S==Eigen::Dynamic)||(I==Eigen::Dynamic)||(O==Eigen::Dynamic))
    {
      size_t _xDim = eu::rows(_args.A);
      size_t _uDim = eu::cols(_args.B);
      size_t _yDim = eu::rows(_args.C);               
      if(!eu::checkInputDim("Matrix A", _args.A, _xDim, _xDim, msg) 
      || !eu::checkInputDim("Matrix B", _args.B, _xDim, _uDim, msg)
      || !eu::checkInputDim("Matrix C", _args.C, _yDim, _xDim, msg)
      || !eu::checkInputDim("Matrix D", _args.D, _yDim, _uDim, msg))
      {
        msg="Preliminary Check: inputs mismatch"+msg;
        return false; 
      }
    }
    
    if(S==Eigen::Dynamic)
    {
      eu::resize(m_state, eu::rows(_args.A), 1);
    }
    if(I==Eigen::Dynamic)
    {
      eu::resize(m_input, eu::cols(_args.B), 1);
    }
    if(O==Eigen::Dynamic)
    {
      eu::resize(m_output, eu::rows(_args.C), 1);
    }

    if(!eu::checkInputDim("Matrix A", _args.A, xDim(), xDim(), msg)) return false;
    if(!eu::checkInputDim("Matrix B", _args.B, xDim(), uDim(), msg)) return false;
    if(!eu::checkInputDim("Matrix C", _args.C, yDim(), xDim(), msg)) return false;
    if(!eu::checkInputDim("Matrix D", _args.D, yDim(), uDim(), msg)) return false;

    // it may change the dimension of the problem if the matrixes are dynamically allocated
    eu::copy(m_A, _args.A); //A is S x S
    eu::copy(m_B, _args.B); //B is S x I
    eu::copy(m_C, _args.C); //C is O x S
    eu::copy(m_D, _args.D); //D is O x I

    eu::resize(m_Obs, yDim()*xDim(), xDim()); // Obs is (OxS) x S
    eu::resize(m_i2o, yDim()*xDim(), uDim()*xDim() );  // i2o is (OxS) x (IxS))

    msg ="";
    if(!computeObservabilityMatrix(m_Obs, m_A, m_C, xDim() ))
    {
      msg += "The observability matrix is Rank-deficient.";
    }

    if(!computeControllabilityMatrix(m_Ctrl, m_A, m_B, xDim()))
    {
      msg += "The controllability matrix is Rank-deficient.";
    }
  }
  catch(std::exception& e)
  {
    msg += __PRETTY_FUNCTION__ + std::string(": Caught an exception: ") + e.what();
    return false;
  }
  return true;
}

template<int S, int I, int O, int MS, int MI, int MO> 
inline typename DiscreteStateSpace<S,I,O,MS,MI,MO>::Output& 
  DiscreteStateSpace<S,I,O,MS,MI,MO>::update(const DiscreteStateSpace<S,I,O,MS,MI,MO>::Input& input, bool skip_dimension_check)
{
  if(!DiscreteStateSpace<S,I,O,MS,MI,MO>::initialized())
  {
    throw std::runtime_error("The Discrete State Space has not been yet initialized (i.e., the matrices are undefined). Abort.");  
  }

  if(!skip_dimension_check)
    eu::checkInputDimAndThrowEx("Input", m_input, eu::rows(input), 1);
  m_input  = input;  
  m_output = m_C*m_state + m_D*m_input;
  m_state  = m_A*m_state + m_B*m_input;
  return m_output;
}

template<int S, int I, int O, int MS, int MI, int MO> 
inline void DiscreteStateSpace<S,I,O,MS,MI,MO>::setState(const DiscreteStateSpace<S,I,O,MS,MI,MO>::State& state)
{
  if(!DiscreteStateSpace<S,I,O,MS,MI,MO>::initialized())
  {
    throw std::runtime_error("The Discrete State Space has not been yet initialized (i.e., the matrices are undefined). Abort.");  
  }
  
  eu::checkInputDimAndThrowEx("State", m_state, eu::rows(state), 1);
  
  m_state = state;
}

template<int S, int I, int O, int MS, int MI, int MO> 
inline bool DiscreteStateSpace<S,I,O,MS,MI,MO>::setStateFromIO(
    const Eigen::VectorXd& past_inputs,
    const Eigen::VectorXd& past_outputs)
{
  std::string error;
  if(!estimateState0(m_state, m_A, m_B, m_C, m_D, past_inputs, past_outputs, error))
  {
    std::cerr << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": " << error << std::endl;
    return false;
  }
  for(int istep=0;istep< (past_inputs.rows() / xDim() );istep++)  
  {
    if(!eu::copy_from_block(m_input, past_inputs, istep*uDim(),0,uDim(),1))
    {
      std::cerr<<__PRETTY_FUNCTION__<<":"<<__LINE__<<":" << "Error in copying the past input in the input." << std::endl;
      return false;
    }
    update( m_input );
  }
  return true;
}


template<int S, int I, int O, int MS, int MI, int MO> 
inline bool DiscreteStateSpace<S,I,O,MS,MI,MO>::setStateFromLastIO(const Input& inputs, const Output& outputs)
{
  eu::checkInputDimAndThrowEx("setStateFromLastIO - Inputs", m_input, eu::rows(inputs), eu::cols(inputs));
  eu::checkInputDimAndThrowEx("setStateFromLastIO - Outputs", m_output, eu::rows(outputs), eu::cols(outputs));

  if(!eu::solve(m_state, m_C, outputs - m_D * inputs ) )
  {
    std::cerr << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": Pseudo inv failed." << std::endl;
    return false;
  }
  m_input = inputs;
  m_output = outputs;
  return true;
}
template<int S, int I, int O, int MS, int MI, int MO> inline 
const typename DiscreteStateSpace<S,I,O,MS,MI,MO>::MatrixI2O& 
DiscreteStateSpace<S,I,O,MS,MI,MO>::computeInputToOutputMatrix()
{
  eu::setZero(m_i2o);
  for (unsigned int idx=0;idx<xDim();idx++)
  {
    if(!eu::copy_to_block(m_i2o, m_D,
      idx*yDim(),idx*uDim(), yDim(),uDim()))
      {
        std::cerr << __PRETTY_FUNCTION__ <<":"<<__LINE__<<": Error in i2o assignement." << std::endl;
      }
  }
  
  MatrixA powA;
  eu::resize(powA,xDim(), xDim());
  eu::setIdentity(powA);
  
  for (unsigned int idx=1;idx<xDim();idx++)
  {
    for (unsigned int idx2=0;idx2<(xDim()-idx);idx2++)
    {
      if(!eu::copy_to_block(m_i2o, m_C*powA*m_B,
        (idx+idx2)*yDim(),(idx2)*uDim(), yDim(),uDim()))
        {
          std::cerr << __PRETTY_FUNCTION__ <<":"<<__LINE__<<": Error in x0 assignement." << std::endl;
        }
    }
    powA*=m_A;
  }
  return m_i2o;
}


template<int S, int I, int O, int MS, int MI, int MO> 
inline bool DiscreteStateSpace<S,I,O,MS,MI,MO>::initialized()
{
  return xDim() > 0 && uDim() > 0 && yDim() > 0;
}

template<int S, int I, int O, int MS, int MI, int MO> 
inline const typename DiscreteStateSpace<S,I,O,MS,MI,MO>::MatrixA&
DiscreteStateSpace<S,I,O,MS,MI,MO>::getAMatrix() const
{
  return m_A;
}

template<int S, int I, int O, int MS, int MI, int MO> 
inline const typename DiscreteStateSpace<S,I,O,MS,MI,MO>::MatrixA&
DiscreteStateSpace<S,I,O,MS,MI,MO>::A() const
{
  return m_A;
}

template<int S, int I, int O, int MS, int MI, int MO>
inline Eigen::Map<Eigen::MatrixXd const>
DiscreteStateSpace<S,I,O,MS,MI,MO>::getA() const
{
  return Eigen::Map<Eigen::MatrixXd const>(eu::data(m_A), eu::rows(m_A), eu::cols(m_A));
}

template<int S, int I, int O, int MS, int MI, int MO> 
inline const typename DiscreteStateSpace<S,I,O,MS,MI,MO>::MatrixB&
DiscreteStateSpace<S,I,O,MS,MI,MO>::getBMatrix() const
{
  return m_B;
}

template<int S, int I, int O, int MS, int MI, int MO> 
inline const typename DiscreteStateSpace<S,I,O,MS,MI,MO>::MatrixB&
DiscreteStateSpace<S,I,O,MS,MI,MO>::B() const
{
  return m_B;
}

template<int S, int I, int O, int MS, int MI, int MO>
inline Eigen::Map<Eigen::MatrixXd const>
DiscreteStateSpace<S,I,O,MS,MI,MO>::getB() const
{
  return Eigen::Map<Eigen::MatrixXd const>(eu::data(m_B), eu::rows(m_B), eu::cols(m_B));
}

template<int S, int I, int O, int MS, int MI, int MO> 
inline const typename DiscreteStateSpace<S,I,O,MS,MI,MO>::MatrixC&
DiscreteStateSpace<S,I,O,MS,MI,MO>::getCMatrix() const
{
  return m_C;
}

template<int S, int I, int O, int MS, int MI, int MO> 
inline const typename DiscreteStateSpace<S,I,O,MS,MI,MO>::MatrixC&
DiscreteStateSpace<S,I,O,MS,MI,MO>::C() const
{
  return m_C;
}

template<int S, int I, int O, int MS, int MI, int MO>
inline Eigen::Map<Eigen::MatrixXd const>
DiscreteStateSpace<S,I,O,MS,MI,MO>::getC() const
{
  return Eigen::Map<Eigen::MatrixXd const>(eu::data(m_C), eu::rows(m_C), eu::cols(m_C));
}

template<int S, int I, int O, int MS, int MI, int MO> 
inline const typename DiscreteStateSpace<S,I,O,MS,MI,MO>::MatrixD&
DiscreteStateSpace<S,I,O,MS,MI,MO>::getDMatrix() const
{
  return m_D;
}

template<int S, int I, int O, int MS, int MI, int MO> 
inline const typename DiscreteStateSpace<S,I,O,MS,MI,MO>::MatrixD&
DiscreteStateSpace<S,I,O,MS,MI,MO>::D() const
{
  return m_D;
}

template<int S, int I, int O, int MS, int MI, int MO>
inline Eigen::Map<Eigen::MatrixXd const>
DiscreteStateSpace<S,I,O,MS,MI,MO>::getD() const
{
  return Eigen::Map<Eigen::MatrixXd const>(eu::data(m_D), eu::rows(m_D), eu::cols(m_D));
}


//! accessor to last input
template<int S, int I, int O, int MS, int MI, int MO> 
inline const typename DiscreteStateSpace<S,I,O,MS,MI,MO>::Input&
DiscreteStateSpace<S,I,O,MS,MI,MO>::getInput()  const
{
  return m_input;
}

template<int S, int I, int O, int MS, int MI, int MO> 
inline const typename DiscreteStateSpace<S,I,O,MS,MI,MO>::Input&
DiscreteStateSpace<S,I,O,MS,MI,MO>::u() const
{
  return m_input;
}

template<int S, int I, int O, int MS, int MI, int MO>
inline Eigen::Map<Eigen::VectorXd const>
DiscreteStateSpace<S,I,O,MS,MI,MO>::getU() const
{
  return Eigen::Map<Eigen::VectorXd const>(eu::data(m_input), eu::rows(m_input), eu::cols(m_input));
}


template<int S, int I, int O, int MS, int MI, int MO> 
inline const typename DiscreteStateSpace<S,I,O,MS,MI,MO>::Output&
DiscreteStateSpace<S,I,O,MS,MI,MO>::getOutput()  const
{
  return m_output;
}

template<int S, int I, int O, int MS, int MI, int MO> 
inline const typename DiscreteStateSpace<S,I,O,MS,MI,MO>::Output&
DiscreteStateSpace<S,I,O,MS,MI,MO>::y() const
{
  return m_output;
}

template<int S, int I, int O, int MS, int MI, int MO>
inline Eigen::Map<Eigen::VectorXd const>
DiscreteStateSpace<S,I,O,MS,MI,MO>::getY() const
{
  return Eigen::Map<Eigen::VectorXd const>(eu::data(m_output), eu::rows(m_output), eu::cols(m_output));
}


template<int S, int I, int O, int MS, int MI, int MO> 
inline const typename DiscreteStateSpace<S,I,O,MS,MI,MO>::State&
DiscreteStateSpace<S,I,O,MS,MI,MO>::getState() const
{
  return m_state;
}

template<int S, int I, int O, int MS, int MI, int MO> 
inline const typename DiscreteStateSpace<S,I,O,MS,MI,MO>::State& 
DiscreteStateSpace<S,I,O,MS,MI,MO>::x() const
{
  return m_state;
}

template<int S, int I, int O, int MS, int MI, int MO>
inline Eigen::Map<Eigen::VectorXd const>
DiscreteStateSpace<S,I,O,MS,MI,MO>::getX() const
{
  return Eigen::Map<Eigen::VectorXd const>(eu::data(m_state), eu::rows(m_state));
}

  
template<int S, int I, int O, int MS, int MI, int MO> 
inline const typename DiscreteStateSpace<S,I,O,MS,MI,MO>::MatrixObs& 
DiscreteStateSpace<S,I,O,MS,MI,MO>::getObservability() const
{
  return m_Obs;
}

template<int S, int I, int O, int MS, int MI, int MO> 
inline const typename DiscreteStateSpace<S,I,O,MS,MI,MO>::MatrixCtrl& 
DiscreteStateSpace<S,I,O,MS,MI,MO>::getControllability() const
{
  return m_Ctrl;
}

template<int S, int I, int O, int MS, int MI, int MO> 
inline int DiscreteStateSpace<S,I,O,MS,MI,MO>::xDim() const
{
  return eu::rows(m_state);
}

template<int S, int I, int O, int MS, int MI, int MO> 
inline int DiscreteStateSpace<S,I,O,MS,MI,MO>::uDim() const
{
  return eu::rows(m_input);
}

template<int S, int I, int O, int MS, int MI, int MO> 
inline int DiscreteStateSpace<S,I,O,MS,MI,MO>::yDim() const
{
  return eu::rows(m_output);
}


}  // namespace eigen_control_toolbox

#endif  // state_space_systems_impl_201811280956
