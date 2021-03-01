#pragma once // qtcreator workaround, clang-tidy

#ifndef   STATE_SPACE_FILTERS__SG_FILTERS_IMPL_H
#define   STATE_SPACE_FILTERS__SG_FILTERS_IMPL_H

#include <state_space_filters/sg_filter.h>

namespace eigen_control_toolbox
{

template<int DIM, int MaxDIM>
inline SavitkyGolay<DIM,MaxDIM>::SavitkyGolay(const double& natural_frequency,
                                              const double& sample_period,
                                              const unsigned int& polynomial_order,
                                              const unsigned int& output_size
)
{
  initSavitkyGolay(natural_frequency,sample_period,polynomial_order,output_size);
}

template<int DIM, int MaxDIM>
inline SavitkyGolay<DIM,MaxDIM>::SavitkyGolay(const unsigned int& window, const double& sample_period, const unsigned int& polynomial_order, const unsigned int& output_size)
{
  initSavitkyGolay(window,sample_period,polynomial_order,output_size);
}

template<int DIM, int MaxDIM>
inline void SavitkyGolay<DIM,MaxDIM>::initSavitkyGolay(const double& natural_frequency, const double& sample_period, const unsigned int& polynomial_order, const unsigned int& output_size)
{
  
  assert(sample_period>0);
  assert(natural_frequency>0);
  assert(output_size>(polynomial_order+1));
  this->m_sampling_period=sample_period;
  double shannon_freq=2*M_PI/sample_period;
  double adim_freq=natural_frequency/shannon_freq;
  
  // adim_freq~=(polynomial_order+1)/(3.2*window-4.6) from "On the frequency-domain properties of Savitzsky-Golay filters" R.W. Schafer, HP laboratories
  unsigned int half_window=std::ceil((4.6+(polynomial_order+1)/adim_freq)/3.2);
  unsigned int window=2*half_window+1;
  
  computeCoeffs(polynomial_order,window,output_size);
}

template<int DIM, int MaxDIM>
inline void SavitkyGolay<DIM,MaxDIM>::initSavitkyGolay(const unsigned int& window, const double& sample_period, const unsigned int& polynomial_order, const unsigned int& output_size)
{
  assert(window>polynomial_order);
  assert(sample_period>0);
  assert(output_size>(polynomial_order+1));
  computeCoeffs(polynomial_order,window,output_size);
}

template<int DIM, int MaxDIM>
inline void SavitkyGolay<DIM,MaxDIM>::computeCoeffs(const unsigned int& polynomial_order, const unsigned int& window, const unsigned int& output_size)
{
  Eigen::VectorXd time(window);
  unsigned int half_window=(window-1)/2;
  for (unsigned int idx=0;idx<window;idx++)
    time(idx)=(half_window-idx)*this->m_sampling_period;
  
  Eigen::MatrixXd regr(window,polynomial_order+1);
  for (unsigned int idx=0;idx<(polynomial_order+1);idx++)
    regr.col(idx)=time.array().pow(idx);
  
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(regr, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::MatrixXd coeffs=svd.solve(Eigen::MatrixXd::Identity(window,window));
  coeffs=coeffs.block(0,0,output_size,coeffs.cols());
  this->computeMatrices(coeffs);
  
}

template<int DIM, int MaxDIM>
inline CausalSavitkyGolay<DIM,MaxDIM>::CausalSavitkyGolay(const double& natural_frequency,
                                                          const double& sample_period,
                                                          const unsigned int& polynomial_order,
                                                          const unsigned int& output_size)
{
  initCausalSavitkyGolay(natural_frequency,sample_period,polynomial_order,output_size);
}

template<int DIM, int MaxDIM>
inline CausalSavitkyGolay<DIM,MaxDIM>::CausalSavitkyGolay(const unsigned int& window, const double& sample_period, const unsigned int& polynomial_order, const unsigned int& output_size)
{
  initCausalSavitkyGolay(window,sample_period,polynomial_order,output_size);
}

template<int DIM, int MaxDIM>
inline void CausalSavitkyGolay<DIM,MaxDIM>::initCausalSavitkyGolay(const double& natural_frequency, const double& sample_period, const unsigned int& polynomial_order, const unsigned int& output_size)
{

  assert(sample_period>0);
  assert(natural_frequency>0);
  assert(output_size>(polynomial_order+1));
  this->m_sampling_period=sample_period;
  double shannon_freq=2*M_PI/sample_period;
  double adim_freq=natural_frequency/shannon_freq;
  
  // adim_freq~=(polynomial_order+1)/(3.2*window-4.6) from "On the frequency-domain properties of Savitzsky-Golay filters" R.W. Schafer, HP laboratories
  unsigned int half_window=std::ceil((4.6+(polynomial_order+1)/adim_freq)/3.2);
  unsigned int window=2*half_window+1;
  
  computeCoeffs(polynomial_order,window,output_size);
}

template<int DIM, int MaxDIM>
inline void CausalSavitkyGolay<DIM,MaxDIM>::initCausalSavitkyGolay(const unsigned int& window, const double& sample_period, const unsigned int& polynomial_order, const unsigned int& output_size)
{
  assert(window>polynomial_order);
  assert(sample_period>0);
  assert(output_size>(polynomial_order+1));
  computeCoeffs(polynomial_order,window,output_size);
}

template<int DIM, int MaxDIM>
inline void CausalSavitkyGolay<DIM,MaxDIM>::computeCoeffs(const unsigned int& polynomial_order, 
                                                          const unsigned int& window, 
                                                          const unsigned int& output_size)
{
  Eigen::VectorXd time(window);
  for (unsigned int idx=0;idx<window;idx++)
    time(idx) = -(idx*this->m_sampling_period );
  
  Eigen::MatrixXd regr(window,polynomial_order+1);
  for (unsigned int idx=0;idx<(polynomial_order+1);idx++)
  {
    regr.col(idx)=time.array().pow(idx);
  }
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(regr, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::MatrixXd coeffs=svd.solve(Eigen::MatrixXd::Identity(window,window));
  coeffs=coeffs.block(0,0,output_size,coeffs.cols());
  this->computeMatrices(coeffs);
  
}


}

#endif  // STATE_SPACE_FILTERS__SG_FILTERS_IMPL_H


