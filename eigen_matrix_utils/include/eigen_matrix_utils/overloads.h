#ifndef EIGEN_MATH_UTILS_OVERLOADS_H
#define EIGEN_MATH_UTILS_OVERLOADS_H

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <type_traits>

namespace eigen_utils
{

//! copy
bool copy(double& lhs, const double& rhs);
bool copy(double& lhs, const std::vector<double>& rhs);
bool copy(std::vector<double>& lhs, const double& rhs);

template<typename D>              bool copy(double& lhs, const Eigen::MatrixBase<D>& rhs);
template<typename D, typename E>  bool copy(Eigen::MatrixBase<D> & lhs,const Eigen::MatrixBase<E>& rhs);
template<typename D>              bool copy(Eigen::MatrixBase<D>& lhs, const double& rhs);
template<typename D>              bool copy(Eigen::MatrixBase<D>& lhs, const std::vector<double>& rhs);
template<typename D>              bool copy(std::vector<double>& lhs, const Eigen::MatrixBase<D>& rhs);


//! copy_to_block
bool copy_to_block(double& lhs, const double& rhs,
              Eigen::Index startRow, Eigen::Index startCol, Eigen::Index blockRows, Eigen::Index blockCols);

template<typename LHS, typename RHS>
bool copy_to_block(Eigen::MatrixBase<LHS>& lhs, const Eigen::MatrixBase<RHS>& rhs,
              Eigen::Index startRow, Eigen::Index startCol, Eigen::Index blockRows, Eigen::Index blockCols);

template<typename D>
bool copy_to_block(Eigen::MatrixBase<D>& lhs, const double& rhs,
              Eigen::Index startRow, Eigen::Index startCol, Eigen::Index blockRows, Eigen::Index blockCols);

template<typename D>
bool copy_to_block(double& lhs, const Eigen::MatrixBase<D>& rhs,
              Eigen::Index startRow, Eigen::Index startCol, Eigen::Index blockRows, Eigen::Index blockCols);

bool copy_to_block(double& lhs, const double& rhs,
              Eigen::Index startRow, Eigen::Index blockRows);

template<typename LHS, typename RHS>
bool copy_to_block(Eigen::MatrixBase<LHS>& lhs, const Eigen::MatrixBase<RHS>& rhs,
              Eigen::Index startRow, Eigen::Index blockRows);

template<typename D>
bool copy_to_block(Eigen::MatrixBase<D>& lhs, const double& rhs, Eigen::Index startRow, Eigen::Index blockRows);

template<typename D>
bool copy_to_block(double& lhs, const Eigen::MatrixBase<D>& rhs, Eigen::Index startRow, Eigen::Index blockRows);

//! copy_from_block
bool copy_from_block(double& lhs, const double& rhs,
              Eigen::Index startRow, Eigen::Index startCol, Eigen::Index blockRows, Eigen::Index blockCols);
template<typename LHS, typename RHS>
bool copy_from_block(Eigen::MatrixBase<LHS>& lhs, const Eigen::MatrixBase<RHS>& rhs,
              Eigen::Index startRow, Eigen::Index startCol, Eigen::Index blockRows, Eigen::Index blockCols);

template<typename D>
bool copy_from_block(Eigen::MatrixBase<D>& lhs, const double& rhs,
              Eigen::Index startRow, Eigen::Index startCol, Eigen::Index blockRows, Eigen::Index blockCols);
template<typename D>
bool copy_from_block(double& lhs, const Eigen::MatrixBase<D>& rhs,
              Eigen::Index startRow, Eigen::Index startCol, Eigen::Index blockRows, Eigen::Index blockCols);



//! setZero
void setZero(double& m);
template<typename D> void setZero(Eigen::MatrixBase<D>& m);

//! setIdentity
void setIdentity(double& m);
template<typename D> void setIdentity(Eigen::MatrixBase<D>& m);


//! data
double* data(double& v);
const double* data(const double& v);

template<typename D> typename D::Scalar* data(Eigen::MatrixBase<D>& m);
template<typename D> const typename D::Scalar* data(const Eigen::MatrixBase<D>& m);


//! at
double& at(double& v, const int& i, const int& j=0) ;
const double& at(const double& v, const int& i, const int& j=0);
template<typename D> double& at(Eigen::MatrixBase<D>& m, const int& i, const int& j=0);
template<typename D> const double& at(const Eigen::MatrixBase<D>& m, const int& i, const int& j=0);


//!norm
double norm(const double& m);
template<typename D> double norm(const Eigen::MatrixBase<D>& m);

//!normalized
double normalized(const double& m);

template<typename D> Eigen::Matrix<typename D::Scalar, D::RowsAtCompileTime, D::ColsAtCompileTime>
  normalized(const Eigen::MatrixBase<D>& m);

//! size
int size(const double& m);

template<typename D> int size(const Eigen::MatrixBase<D>& m);

//!rows
int rows(const double& m);
int rows(const std::vector<double>& m);
template<typename D> int rows(const Eigen::MatrixBase<D>& m);


//! cols
int cols(const double& m);
template<typename D> int cols(const Eigen::MatrixBase<D>& m);

//!rank
int rank(const double& m);

template<typename D> int rank(const Eigen::MatrixBase<D>& m);


//! resize
template<typename D,
         std::enable_if_t< (Eigen::MatrixBase<D>::RowsAtCompileTime == Eigen::Dynamic)
                        || (Eigen::MatrixBase<D>::ColsAtCompileTime == Eigen::Dynamic)
                        , int> = 0>
bool resize(Eigen::MatrixBase<D> const & m, int rows, int cols = 1);

template<typename D,
         std::enable_if_t< (Eigen::MatrixBase<D>::RowsAtCompileTime != Eigen::Dynamic)
                        && (Eigen::MatrixBase<D>::ColsAtCompileTime != Eigen::Dynamic)
                        , int> = 0>
bool resize(Eigen::MatrixBase<D> const & m, int rows, int cols = 1);
bool resize(const double& m, int rows, int cols = 1);


//!checkInputDim
template<typename D,
        std::enable_if_t< (Eigen::MatrixBase<D>::RowsAtCompileTime == Eigen::Dynamic)
                        ||(Eigen::MatrixBase<D>::ColsAtCompileTime == Eigen::Dynamic)
                        , int> = 0>
bool checkInputDim(const std::string& id, const Eigen::MatrixBase<D>& m, int rows, int cols, std::string& error);

template<typename D,
        std::enable_if_t< (Eigen::MatrixBase<D>::RowsAtCompileTime != Eigen::Dynamic)
                        && (Eigen::MatrixBase<D>::ColsAtCompileTime != Eigen::Dynamic)
                        , int> = 0>
bool checkInputDim(const std::string& id, const Eigen::MatrixBase<D>& m, int rows, int cols, std::string& error);

bool checkInputDim(const std::string& id, const double& m, int rows, int cols, std::string& error);


//!checkInputDim checkInputDimAndThrowEx
template<typename D>
void checkInputDimAndThrowEx(const std::string& id, const Eigen::MatrixBase<D>& m, int rows, int cols);
void checkInputDimAndThrowEx(const std::string& id, const double& m, int rows, int cols);


//! SVD - A least-squares solution of m*x = rhs
template<typename D, typename InputD, typename OutputD>
bool svd(const Eigen::MatrixBase<D>& m, const Eigen::MatrixBase<InputD>& rhs, Eigen::MatrixBase<OutputD>& x);
bool svd(const double& m, const double& rhs, double& x);

//! block
double& block(double& m, Eigen::Index startRow, Eigen::Index startCol, Eigen::Index blockRows, Eigen::Index blockCols);
const double& block(const double& m,
                          Eigen::Index startRow, Eigen::Index startCol, Eigen::Index blockRows, Eigen::Index blockCols);

template<typename D>
Eigen::Block<D> block(Eigen::MatrixBase<D>& m,
                          Eigen::Index startRow, Eigen::Index startCol, Eigen::Index blockRows, Eigen::Index blockCols);
 
template<typename D>
const Eigen::Block<D> block(const Eigen::MatrixBase<D>& m,
                          Eigen::Index startRow, Eigen::Index startCol, Eigen::Index blockRows, Eigen::Index blockCols);

//!solve A x = b x = b/A
bool solve(double& x, const double& A, const double& b );

//!solve A x = b x = b/A
template<typename D>
bool solve(Eigen::MatrixBase<D>& x, const Eigen::MatrixXd& A, const Eigen::VectorXd& b);

//!solve A x = b x = b/A
template<typename D>
bool solve(Eigen::MatrixBase<D>& x, const Eigen::MatrixXd& A, const double& b);

//! setConstant
void setConstant(double& m, const double& v);

template<typename D> void setConstant(Eigen::MatrixBase<D>& m, const double& v) ;

//! setRandom
void setRandom(double& m);

template<typename D> void setRandom(Eigen::MatrixBase<D>& m);

//! setDiagonal
void setDiagonal(double& m, const double& v);

void setDiagonal(double& m, const std::vector<double>& v);

template<typename D> void setDiagonal(double& m, const Eigen::MatrixBase<D>& v);

template<typename D> void setDiagonal(Eigen::MatrixBase<D>& m, const double& v);

template<typename D> void setDiagonal(Eigen::MatrixBase<D>& m, const std::vector<double>& v);

//! saturate
void saturate(double& v, const double& min, const double& max);

template<typename D> void saturate(Eigen::MatrixBase<D>& m, const double& min, const double& max);

template<typename D> void saturate(Eigen::MatrixBase<D>& m,const Eigen::MatrixBase<D>& min,const Eigen::MatrixBase<D>& max);

//! dot
double dot(const double& m1, const double& m2);

template<typename D, typename E> double dot(const Eigen::MatrixBase<D>& m1, const Eigen::MatrixBase<E>& m2);



//============
template<typename MatType>
using PseudoInverseType = Eigen::Matrix<typename MatType::Scalar, MatType::ColsAtCompileTime, MatType::RowsAtCompileTime>;

template<typename MatType>
PseudoInverseType<MatType> pseudoInverse(const MatType &a, double epsilon = std::numeric_limits<double>::epsilon());

//! DIV
double div(const double& a, const double b);

template<typename AType, typename BType> //  C = B^-1 x A  (bc x ac ) = (bc x br)  x (ar x ac )
using DivType = Eigen::Matrix<typename AType::Scalar, BType::ColsAtCompileTime, AType::ColsAtCompileTime>;

template<typename AType, typename BType>
DivType<AType, BType> div(const AType& a, const BType& b);

//============
std::string to_string(const double& m, bool transpose = true);

template<typename Derived>
std::string to_string(const Eigen::MatrixBase<Derived>& m, bool transpose = true);
  
}  // namesapce eigen_utils


#include <eigen_matrix_utils/internal/overloads_impl.h>

#endif
