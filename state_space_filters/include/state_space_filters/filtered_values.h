#ifndef STATE_SPACE_FILTERS__FILTERED_VALUES__H
#define STATE_SPACE_FILTERS__FILTERED_VALUES__H

#include <memory>
#include <Eigen/Core>
#include <state_space_filters/iir_filters.h>

namespace eigen_control_toolbox
{

/**
 * @class FilteredValue
 * @brief The class is a wrap for the templated-value and the filter provied bu eigen_common_filter. 
 * The template is designed in order to use as raw data std::vector or Eigen::VectorXd, or static dimensioned Eigen::Matrix<double,N,1>
 * The specialization for Eigen::VectorXd, Eigen:Matrix<double,6,1>, and Eigen:Matrix<double,7,1> is provided.
 */
template<int N, int MaxN=N>
class FilteredValue
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef  std::shared_ptr<FilteredValue> Ptr;
  typedef  std::shared_ptr<FilteredValue const> ConstPtr;

  //! Value: dimension N, if N==1, then the Value type is a double
  using Value = typename std::conditional<N==1, 
                    double, Eigen::Matrix<double,N,1,0,MaxN,1>>::type;

  FilteredValue();
  virtual ~FilteredValue() = default;
  FilteredValue(const FilteredValue&);
  FilteredValue& operator=(const FilteredValue&);
  FilteredValue(FilteredValue&&) = default;
  FilteredValue& operator=(FilteredValue&&) = default;

  bool activateFilter ( const Value& dead_band
                      , const Value& saturation
                      , const double natural_frequency
                      , const double sampling_time
                      , const Value& init_value );

  void deactivateFilter( );

  // ACCESSOR
  const Value& value() const;
  Value& value();

  const Value& raw() const;
  Value& raw();

  // conditional template, if n==1, it is a simple double, otherwise,
  // access to data() of the matrix
  template <int n=N, typename std::enable_if<n==1, int>::type = 0>
  const double* data() const;

  template <int n=N, typename std::enable_if<n!=1, int>::type = 0>
  const double* data() const;

  template <int n=N, typename std::enable_if<n==1, int>::type = 0>
  double* data();

  template <int n=N, typename std::enable_if<n!=1, int>::type = 0>
  double* data();

  // METHOD
  //! @brief If the filter is activtated, the values is saturated, and then filtered, otherwise, it is simply assigned
  FilteredValue<N,MaxN>& update(const Value& new_values);
  //! @brief If the filter is activtated, it returns the saturated value, otherwise, the raw value is returned
  const Value& getUpdatedValue( ) const;

  //! ACCESSOR (if iAx <0 || iAx > dim, throw exception)
  const double& value(const int iAx) const;
  double& value(const int iAx);

  const double* data(const int iAx) const;
  double* data(const int iAx);

  bool resize(int nAx); // false if N!=-1 and nAx!=N

protected:
  bool   filter_active_;
  bool   first_update_;
  double natural_frequency_;
  double sampling_time_;

  Value raw_values_;
  Value bonded_value_;
  Value values_;
  Value dead_band_;
  Value saturation_;

  eigen_control_toolbox::FirstOrderLowPass<N,MaxN> lpf_; 
};

//! The class is for 1-dim problems
using  FilteredScalar = FilteredValue<1>;
typedef std::shared_ptr<FilteredScalar      > FilteredScalarPtr;
typedef std::shared_ptr<FilteredScalar const> FilteredScalarConstPtr;


//! The class is for dynamic allocation
using  FilteredVectorXd = FilteredValue<-1>;
typedef std::shared_ptr<FilteredVectorXd      > FilteredVectorXdPtr;
typedef std::shared_ptr<FilteredVectorXd const> FilteredVectorXdConstPtr;


/**
 * 
 * 
 * 
 * 
 * Pre-allocated classes with dimension already defined
 */
#define DEFINE_FILTEREDVALUE_STATIC_DIMENSION_PTR(nAx)\
using   FilteredVector ## nAx ## d = FilteredValue<nAx>;\
typedef std::shared_ptr<FilteredVector ## nAx ## d      > FilteredVector ## nAx ## dPtr;\
typedef std::shared_ptr<FilteredVector ## nAx ## d const> FilteredVector ## nAx ## dConstPtr;\

DEFINE_FILTEREDVALUE_STATIC_DIMENSION_PTR(2) // FilteredValue2, FilteredValue1Ptr, FilteredValue2ConstPtr
DEFINE_FILTEREDVALUE_STATIC_DIMENSION_PTR(3)
DEFINE_FILTEREDVALUE_STATIC_DIMENSION_PTR(4)
DEFINE_FILTEREDVALUE_STATIC_DIMENSION_PTR(6)
DEFINE_FILTEREDVALUE_STATIC_DIMENSION_PTR(7)

#undef DEFINE_FILTEREDVALUE_STATIC_DIMENSION_PTR

} // namespace eigen_control_toolbox

#include <state_space_filters/internal/filtered_values_impl.h>

#endif  // STATE_SPACE_FILTERS__FILTERED_VALUES__H
