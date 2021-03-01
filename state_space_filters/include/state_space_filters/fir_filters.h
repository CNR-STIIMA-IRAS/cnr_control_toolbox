#ifndef   STATE_SPACE_FILTERS__FIR_FILTERS_H
#define   STATE_SPACE_FILTERS__FIR_FILTERS_H

#include <state_space_systems/discrete_state_space_systems.h>

namespace eigen_control_toolbox
{

  template< int N, int MaxN = N>
  class FirFilter: public DiscreteStateSpace<N,1,N,MaxN,1,MaxN>
  {
  protected:
  public: 

    typedef Eigen::Matrix<double, N, N, 0, MaxN, MaxN> MatrixCoeff;
    
    typedef typename DiscreteStateSpace<N,1,N,MaxN,1,MaxN>::Input Input;
    typedef typename DiscreteStateSpace<N,1,N,MaxN,1,MaxN>::Output Output;
    typedef typename DiscreteStateSpace<N,1,N,MaxN,1,MaxN>::InputWindow InputWindow;
    typedef typename DiscreteStateSpace<N,1,N,MaxN,1,MaxN>::OutputWindow OutputWindow;
    
    FirFilter();
    FirFilter(const MatrixCoeff& coeffs);
    void computeMatrices(const MatrixCoeff& coeffs);
    
    virtual void setStateFromIO(const InputWindow& past_inputs, const OutputWindow& OutputWindow);
    virtual const Output& update(const Input& input);    
  };
  
  
  
}

#include <state_space_filters/internal/fir_filters_impl.h>

#endif // STATE_SPACE_FILTERS__FIR_FILTERS_H
