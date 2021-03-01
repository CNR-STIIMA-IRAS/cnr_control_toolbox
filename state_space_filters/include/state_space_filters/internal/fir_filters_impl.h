#pragma once //qtcreator, ctidy workaround

#ifndef   EIGEN_STATE_SPACE_SYSTEM__FIR_FILTERS_IMPL_H
#define   EIGEN_STATE_SPACE_SYSTEM__FIR_FILTERS_IMPL_H

#include <state_space_filters/fir_filters.h>

namespace eigen_control_toolbox
{

template<int N, int MaxN> 
inline FirFilter<N,MaxN>::FirFilter()
{

}

template<int N, int MaxN> 
inline FirFilter<N,MaxN>::FirFilter(const FirFilter<N,MaxN>::MatrixCoeff& coeffs)
{
  computeMatrices(coeffs);
}

template<int N, int MaxN> 
inline void FirFilter<N,MaxN>::computeMatrices(const FirFilter<N,MaxN>::MatrixCoeff& coeffs)
{

  this->initState(coeffs.cols());
  this->initOutput(coeffs.rows());
  
  this->initMatrixA(coeffs.cols());
  this->initMatrixB(coeffs.cols(),1);
  this->initMatrixC(coeffs.rows(),coeffs.cols());
  this->initMatrixD(coeffs.rows(),1);
  
  this->m_A.setZero();
  this->m_A.block(1,0,this->xDim()-1,this->xDim()-1).setIdentity();
  this->m_B.setZero();
  this->m_B(0)=1;
  this->m_C=coeffs.block(0,1,this->yDim(),this->xDim()-1);
  this->m_D=coeffs.block(0,0,this->yDim(),1);
}

template<int N, int MaxN> 
inline const typename FirFilter<N,MaxN>::Output&
 FirFilter<N,MaxN>::update(const FirFilter<N,MaxN>::Input& input)
{
  this->checkInput(input.rows());

  this->m_output = this->m_C*this->m_state + this->m_D*input;
  this->m_state.block(1,0,this->xDim()-1,1)=this->m_state.block(0,0,this->xDim()-1,1);
  this->m_state(0)=input(0);
  
  return this->m_output;
}

template<int N, int MaxN> 
inline void FirFilter<N,MaxN>::setStateFromIO(const FirFilter<N,MaxN>::InputWindow& past_inputs, 
                                                  const FirFilter<N,MaxN>::OutputWindow& past_outputs)
{
  this->checkOutput( past_outputs.rows() );
//  this->m_state = past_inputs; // ?? //
}





}

#endif  // EIGEN_STATE_SPACE_SYSTEM__FIR_FILTERS_IMPL_H
