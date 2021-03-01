#ifndef STATE_SPACE_FILTERS__SG_FILTER__H
#define STATE_SPACE_FILTERS__SG_FILTER__H

#include <state_space_filters/fir_filters.h>

namespace eigen_control_toolbox
{
  
  template<int DIM, int MaxDIM>
  class SavitkyGolay: public FirFilter<DIM,MaxDIM>
  {
  protected:
    void computeCoeffs(const unsigned int& polynomial_order,
                       const unsigned int& window,
                       const unsigned int& output_size);
    
    void initSavitkyGolay(
      const double& natural_frequency,
      const double& sample_period,
      const unsigned int& polynomial_order,
      const unsigned int& output_size=1
    );
    
    void initSavitkyGolay(
      const unsigned int& window,
      const double& sample_period,
      const unsigned int& polynomial_order,
      const unsigned int& output_size=1
    );
    
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    SavitkyGolay(){};
    SavitkyGolay( const double& natural_frequency,
                        const double& sample_period,
                        const unsigned int& polynomial_order,
                        const unsigned int& output_size
    );
    SavitkyGolay( const unsigned int& window,
                        const double& sample_period,
                        const unsigned int& polynomial_order,
                        const unsigned int& output_size
    );
    
  };
  
  
  template<int DIM, int MaxDIM>
  class CausalSavitkyGolay: public FirFilter<DIM,MaxDIM>
  {
  protected:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    void computeCoeffs(const unsigned int& polynomial_order,
                       const unsigned int& window,
                       const unsigned int& output_size);
    
    void initCausalSavitkyGolay(
      const double& natural_frequency,
      const double& sample_period,
      const unsigned int& polynomial_order,
      const unsigned int& output_size=1
    );
    
    void initCausalSavitkyGolay(
      const unsigned int& window,
      const double& sample_period,
      const unsigned int& polynomial_order,
      const unsigned int& output_size=1
    );
    
  public:
    CausalSavitkyGolay(){};
    CausalSavitkyGolay( const double& natural_frequency,
                        const double& sample_period,
                        const unsigned int& polynomial_order,
                        const unsigned int& output_size
                      );
    CausalSavitkyGolay( const unsigned int& window,
                        const double& sample_period,
                        const unsigned int& polynomial_order,
                        const unsigned int& output_size
    );
    
  };
  
}

#include <state_space_filters/internal/sg_filter_impl.h>

#endif  // STATE_SPACE_FILTERS__SG_FILTER__H

