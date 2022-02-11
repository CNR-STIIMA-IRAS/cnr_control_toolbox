#pragma once //qtcreator, ctidy workaround

#ifndef KINEMATICS_FILTERS__KINEMATICS_FILTERS_IMPL__H
#define KINEMATICS_FILTERS__KINEMATICS_FILTERS_IMPL__H

#include <kinematics_filters/kinematics_filters.h>

namespace cnr_control_toolbox
{


template <typename D>
using MatD = Eigen::Matrix<double,
                Eigen::MatrixBase<D>::RowsAtCompileTime, Eigen::MatrixBase<D>::ColsAtCompileTime, Eigen::ColMajor,
                  Eigen::MatrixBase<D>::MaxRowsAtCompileTime, Eigen::MatrixBase<D>::MaxColsAtCompileTime>;

// ==================================================================================
template<typename D1, typename D2, typename D3, typename D4, typename D5>
inline bool saturateSpeedFullState(Eigen::MatrixBase<D1>&  qd_target,
                            const Eigen::MatrixBase<D3>& q_actual,
                              const Eigen::MatrixBase<D2>& qd_actual,
                                const Eigen::MatrixBase<D4>& q_max,
                                  const Eigen::MatrixBase<D5>& q_min,
                                    const Eigen::MatrixBase<D4>& qd_max,
                                      const Eigen::MatrixBase<D5>& qdd_max,
                                          const double& dt,
                                            const double& max_velocity_multiplier,
                                              const bool& preserve_direction,
                                                std::stringstream* report)
{
  if( eigen_utils::size(qd_target) != eigen_utils::size(q_actual)
  ||  eigen_utils::size(qd_target) != eigen_utils::size(q_max) 
  ||  eigen_utils::size(qd_target) != eigen_utils::size(q_min))
  {
    throw std::runtime_error( (__PRETTY_FUNCTION__ + std::string(":") + "inputs dimension mismatch.").c_str());
  }
  for(size_t i=0; i<eigen_utils::rows(qd_target);i++)
  {
    if(q_max(i)<=q_min(i))
      throw std::runtime_error( (__PRETTY_FUNCTION__ + std::string(":") + " The range is wrong. 'q_max' is less than 'q_min'").c_str());
  }
  bool saturated = saturateSpeedFirstOrderState(qd_target,
                      qd_actual, qd_max, qdd_max, dt, max_velocity_multiplier, preserve_direction, report);

  if(report)
  {
    *report << "[-----][BRK   SATURATION] INPUT  qd: " << eigen_utils::to_string(qd_target) << "\n";
  }

  size_t nAx = eigen_utils::size(qd_target);
  MatD<D1> braking_distance, q_saturated_qd;
  eigen_utils::resize(braking_distance, nAx);
  eigen_utils::resize(q_saturated_qd  , nAx);
  for(size_t iAx=0; iAx<nAx;iAx++)
  {
    braking_distance(iAx)  = 0.5 * qdd_max(iAx)
                             * std::pow(std::abs(qd_target(iAx))/qdd_max(iAx) , 2.0);
  }

  q_saturated_qd = q_actual + qd_actual* dt;
  for(size_t iAx=0; iAx<nAx;iAx++)
  {
    if ((q_saturated_qd(iAx) > (q_max(iAx) - braking_distance(iAx))) && (qd_target(iAx)>0))
    {
      saturated = true;
      qd_target(iAx) = std::max(0.0, qd_target(iAx) - qdd_max(iAx) * dt);
    }
    else if((q_saturated_qd(iAx)<(q_min(iAx) + braking_distance(iAx))) && (qd_target(iAx)<0))
    {
      saturated = true;
      qd_target(iAx) = std::min(0.0, qd_target(iAx) + qdd_max(iAx) * dt);
    }
  }

  if(report)
  {
    *report << (saturated ? "[TRUE ]": "[FALSE]" )
            << "[BRK   SATURATION] OUTPUT qd: " << eigen_utils::to_string(qd_target) << "\n";
  }
  return saturated;
}


inline bool saturateSpeedFullState(double& qd_target,
                            const double& q_actual,
                              const double& qd_actual,
                                const double& q_max,
                                  const double& q_min, 
                                    const double& qd_max,
                                      const double& qdd_max, 
                                        const double& dt, 
                                          const double& max_velocity_multiplier, 
                                            const bool& preserve_direction, 
                                              std::stringstream* report)
{
  if(q_max<=q_min)
  {
    throw std::runtime_error( (__PRETTY_FUNCTION__ + std::string(":") + "The range is wrong. 'q_max' is less than 'q_min'").c_str());
  }

  bool saturated = saturateSpeedFirstOrderState(qd_target,
                      qd_actual, qd_max, qdd_max, dt, max_velocity_multiplier, preserve_direction, report);

  if(report)
  {
    *report << "[-----][BRK   SATURATION] INPUT  qd: " << eigen_utils::to_string(qd_target) << "\n";
  }
  double braking_distance;
  braking_distance  = 0.5 * qdd_max
                           * std::pow(std::abs(qd_target)/qdd_max, 2.0);

  double q_saturated_qd = q_actual + qd_actual* dt;
  if ((q_saturated_qd > (q_max - braking_distance)) && (qd_target>0))
  {
    saturated = true;
    qd_target = std::max(0.0, qd_target - qdd_max * dt);
  }
  else if((q_saturated_qd<(q_min + braking_distance)) && (qd_target<0))
  {
    saturated = true;
    qd_target = std::min(0.0, qd_target + qdd_max * dt);
  }

  if(report)
  {
    *report << (saturated ? "[TRUE ]": "[FALSE]" )
            << "[BRK   SATURATION] OUTPUT qd: " << eigen_utils::to_string(qd_target) << "\n";
  }
  return saturated;

}

// ==================================================================================


// ==================================================================================
template<typename D1, typename D2, typename D3, typename D4>
inline bool saturateSpeedFirstOrderState(Eigen::MatrixBase<D1>& qd_target,
                            const Eigen::MatrixBase<D2>& qd_actual,
                              const Eigen::MatrixBase<D3>& qd_max,
                                const Eigen::MatrixBase<D4>& qdd_max,
                                    const double& dt,
                                      const double& max_velocity_multiplier,
                                        const bool& preserve_direction,
                                          std::stringstream* report)
{
  if( eigen_utils::size(qd_target) != eigen_utils::size(qd_actual)
  ||  eigen_utils::size(qd_target) != eigen_utils::size(qdd_max))
  {
    throw std::runtime_error( (__PRETTY_FUNCTION__ + std::string(":") + "inputs dimension mismatch.").c_str());
  }

  bool saturated = saturateSpeed(qd_target, 
                      qd_max, max_velocity_multiplier, preserve_direction, report);

  if(report)
  {
    *report<<"[-----][ACC   SATURATION] INPUT  qd: "<<eigen_utils::to_string(qd_target)
              <<" qd actual: " << eigen_utils::to_string(qd_actual)<<"\n";
  }
  size_t nAx = eigen_utils::rows(qd_target);

  MatD<D1> qd_sup, qd_inf, dqd;
  eigen_utils::resize(qd_sup, nAx);
  eigen_utils::resize(qd_inf, nAx);
  eigen_utils::resize(dqd   , nAx);

  qd_sup = qd_actual + qdd_max * dt;
  qd_inf = qd_actual - qdd_max * dt;
  eigen_utils::setZero(dqd);
  for(size_t iAx=0;iAx<nAx;iAx++)
  {
    dqd(iAx) = qd_target(iAx) > qd_sup(iAx) ? (qd_sup(iAx) - qd_target(iAx))
             : qd_target(iAx) < qd_inf(iAx) ? (qd_inf(iAx) - qd_target(iAx))
             : 0.0;
  }
  saturated |= dqd.cwiseAbs().maxCoeff()>1e-5;
  if(saturated)
  {
    if(preserve_direction)
    {
      Eigen::VectorXd dqd_dir = (qd_target - qd_actual).normalized();
      if(dqd.norm() < 1e-5)
      {
        dqd_dir.setZero();
      }

      if(dqd.minCoeff() * dqd.maxCoeff() >= 0.0)
      {
        qd_target = qd_target + (dqd.dot(dqd_dir) * dqd_dir);
      }
      else
      {
        *report << "[-----][ACC   SATURATION] Target vel     : " << eigen_utils::to_string(qd_target) << "\n";
        *report << "[-----][ACC   SATURATION] Prev target vel: " << eigen_utils::to_string(qd_actual) << "\n";
        *report << "[-----][ACC   SATURATION] qd_sup         : " << eigen_utils::to_string(qd_sup) << "\n";
        *report << "[-----][ACC   SATURATION] qd_inf         : " << eigen_utils::to_string(qd_inf) << "\n";
        *report << "[-----][ACC   SATURATION] Calc correction: " << eigen_utils::to_string(dqd) << "\n";
        qd_target = qd_target + dqd;
      }
    }
  }
  else
  {
    // do nothing
  }

  if(report)
  {
    *report << (saturated ? "[TRUE ]": "[FALSE]" )
            <<"[ACC   SATURATION] OUTPUT qd: "<<eigen_utils::to_string(qd_target)<< "\n";
  }
  return saturated;
}


inline bool saturateSpeedFirstOrderState(double& qd_target,
                            const double& qd_actual,
                              const double& qd_max, 
                                const double& qdd_max,
                                  const double& dt,
                                    const double& max_velocity_multiplier, 
                                      const bool& preserve_direction, 
                                        std::stringstream* report)
{
  bool saturated = saturateSpeed(qd_target, qd_max, max_velocity_multiplier, preserve_direction, report);

  if(report)
  {
    *report<<"[-----][ACC   SATURATION] INPUT  qd: "<<eigen_utils::to_string(qd_target)<<"\n";
  }
  double qd_sup  = qd_actual + qdd_max * dt;
  double qd_inf  = qd_actual - qdd_max * dt;
  double dqd = 0;
  dqd  = qd_target > qd_sup ? (qd_sup - qd_target)
       : qd_target < qd_inf ? (qd_inf + qd_target)
       : 0.0;
  saturated |= std::fabs(dqd)>0.0;

  if(report)
  {
    *report << "Target vel     : " << eigen_utils::to_string(qd_target) << "\n";
    *report << "Prev target vel: " << eigen_utils::to_string(qd_actual) << "\n";
    *report << "qd_sup         : " << eigen_utils::to_string(qd_sup) << "\n";
    *report << "qd_inf         : " << eigen_utils::to_string(qd_inf) << "\n";
    *report << "Calc correction: " << eigen_utils::to_string(dqd) << "\n";
  }
  qd_target = qd_target + dqd;

  if(report)
  {
    *report << (saturated ? "[TRUE ]": "[FALSE]" )
            <<"[ACC   SATURATION] OUTPUT qd: "<<eigen_utils::to_string(qd_target)<< "\n";
  }
  return saturated;
}
// ==================================================================================





// ==================================================================================
template<typename D1, typename D2>
inline bool saturateSpeed(Eigen::MatrixBase<D1>& qd_target,
                            const Eigen::MatrixBase<D2>& qd_max,
                              const double& max_velocity_multiplier,
                                const bool& preserve_direction,
                                  std::stringstream* report)
{
  if( eigen_utils::size(qd_target) != eigen_utils::size(qd_max) )
  {
    throw std::runtime_error( (__PRETTY_FUNCTION__ + std::string(":") + "inputs dimension mismatch.").c_str());
  }
  
  if(report)
  {
    *report << "[-----][SPEED SATURATION] INPUT  qd: " << eigen_utils::to_string(qd_target)
              << " dq max : " << eigen_utils::to_string(qd_target)
               <<" max multiplier: " << max_velocity_multiplier<<"\n";
  }


  size_t nAx = eigen_utils::rows(qd_target);

  MatD<D1> scale;
  eigen_utils::resize(scale, nAx);
  
  for(size_t iAx=0;iAx<nAx;iAx++)
  {
    scale(iAx) = std::fabs(qd_target(iAx)) > qd_max * max_velocity_multiplier
               ? qd_max * max_velocity_multiplier/ std::fabs(qd_target(iAx) )
               : 1.0;
  }

  if(preserve_direction)
  {
    qd_target = scale.minCoeff() * qd_target;
  }
  else
  {
    qd_target = scale.asDiagonal() * qd_target;
  }

  if(report)
  {
    *report << (scale.minCoeff()<1 ? "[TRUE ]": "[FALSE]" )
            << "[SPEED SATURATION] OUTPUT qd: " << eigen_utils::to_string(qd_target) << "\n";
  }

  return scale.minCoeff()<1;
}

inline bool saturateSpeed(double& qd_target, 
                            const double& qd_max, 
                              const double& max_velocity_multiplier,
                                const bool& /*preserve_direction*/, 
                                  std::stringstream* report)
{
  if(report)
  {
    *report << "[-----][SPEED SATURATION] INPUT  qd: " << eigen_utils::to_string(qd_target)
                  <<" max multiplier: " << max_velocity_multiplier<<"\n";
  }
  double scale = std::fabs(qd_target) > qd_max * max_velocity_multiplier
               ? qd_max * max_velocity_multiplier/ std::fabs(qd_target)
               : 1.0;

  qd_target = scale * qd_target;

  if(report)
  {
    *report << (scale<1 ? "[TRUE ]": "[FALSE]" )
            << "[SPEED SATURATION] OUTPUT qd: " << eigen_utils::to_string(qd_target) << "\n";
  }

  return scale<1;
}

// ==================================================================================


// ==================================================================================
template<typename D1, typename D2, typename D3>
inline bool saturatePosition(Eigen::MatrixBase<D1>& q_target,
                              const Eigen::MatrixBase<D2>& q_max, 
                                const Eigen::MatrixBase<D3>& q_min,
                                  std::stringstream* report)
{
  if(report)
  {
    *report << "[-----][POS   SATURATION] INPUT  q: " << eigen_utils::to_string(q_target) << "\n";
    *report << "[-----][POS   SATURATION] MAX    q: " << eigen_utils::to_string(q_max) << "\n";
    *report << "[-----][POS   SATURATION] MIN    q: " << eigen_utils::to_string(q_min) << "\n";
  }
  if( eigen_utils::size(q_target) != eigen_utils::size(q_max) 
   || eigen_utils::size(q_target) != eigen_utils::size(q_min))
  {
    throw std::runtime_error( (__PRETTY_FUNCTION__ + std::string(":") + " Inputs dimension mismatch.").c_str());
  }
  for(size_t i=0; i<eigen_utils::rows(q_target);i++)
  {
    if(q_max(i)<=q_min(i))
      throw std::runtime_error( (__PRETTY_FUNCTION__ + std::string(":") + " The range is wrong. 'q_max' is less than 'q_min'").c_str());
  }


  MatD<D1> dq;
  eigen_utils::resize(dq, q_target.rows());
  size_t nAx = eigen_utils::size(q_target);
  for(size_t iAx=0;iAx<nAx;iAx++)
  {
    dq(iAx)  = q_target(iAx) > q_max(iAx) ? (q_max(iAx) - q_target(iAx))
             : q_target(iAx) < q_min(iAx) ? (q_min(iAx) - q_target(iAx))
             : 0.0;
  }

  q_target += dq;
  if(report)
  {
    *report << (dq.cwiseAbs().maxCoeff()>0.0 ? "[TRUE ]": "[FALSE]" )
            << "[POS   SATURATION] OUTPUT q: " << eigen_utils::to_string(q_target) << "\n";
  }

  return (dq.cwiseAbs().maxCoeff()>0.0);
}


inline bool saturatePosition(double& q_target, 
                              const double& q_max,
                                const double& q_min,
                                  std::stringstream* report)
{
  if(report)
  {
    *report << "[-----][POS   SATURATION] INPUT  q: " << eigen_utils::to_string(q_target) << "\n";
    *report << "[-----][POS   SATURATION] MAX    q: " << eigen_utils::to_string(q_max) << "\n";
    *report << "[-----][POS   SATURATION] MIN    q: " << eigen_utils::to_string(q_min) << "\n";
  }
  if(q_max<=q_min)
  {
    throw std::runtime_error( (__PRETTY_FUNCTION__ + std::string(":") + "The range is wrong. 'q_max' is less than 'q_min'").c_str());
  }

  double dq;
  dq   = q_target > q_max ? (q_max - q_target)
       : q_target < q_min ? (q_min + q_target)
       : 0.0;

  q_target += dq;

  if(report)
  {
    *report << (std::fabs(dq)>0.0 ? "[TRUE ]": "[FALSE]" )
            << "[POS   SATURATION] OUTPUT q: " << eigen_utils::to_string(q_target) << "\n";
  }

  return (std::fabs(dq)>0.0);
}
// ==================================================================================


}  // namespace cnr_control_toolbox

#endif  // KINEMATICS_FILTERS__KINEMATICS_FILTERS_IMPL__H
