#include <eigen_matrix_utils/overloads.h>
#include <eigen_matrix_utils/eigen_matrix_utils.h>


namespace eigen_utils 
{

// double standard_deviation(const Eigen::Ref< Eigen::VectorXd >& vet)
// {
//   Eigen::VectorXd mean_vet(vet.rows());
//   mean_vet.setConstant(vet.mean());
//   return std::sqrt((vet-mean_vet).dot(vet-mean_vet))/vet.rows();
// }

//double correlation(const Eigen::Ref< Eigen::VectorXd >& vet1, const Eigen::Ref< Eigen::VectorXd >& vet2)
//{
//  Eigen::VectorXd mean_vet1(vet1.rows());
//  mean_vet1.setConstant(vet1.mean());
//  Eigen::VectorXd vet1_no_mean=vet1-mean_vet1;
  
//  Eigen::VectorXd mean_vet2(vet2.rows());
//  mean_vet2.setConstant(vet2.mean());
//  Eigen::VectorXd vet2_no_mean=vet2-mean_vet2;
  
//  return ( vet1_no_mean.dot(vet2_no_mean) ) / std::sqrt( ( vet1_no_mean.dot(vet1_no_mean) ) * ( vet2_no_mean.dot(vet2_no_mean) ) );
  
//}

  

}
