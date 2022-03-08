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
inline bool saturateSpeed(Eigen::MatrixBase<D1>&  qd,
                            const Eigen::MatrixBase<D3>& q_prev,
                              const Eigen::MatrixBase<D2>& qd_prev,
                                const Eigen::MatrixBase<D4>& q_max,
                                  const Eigen::MatrixBase<D5>& q_min,
                                    const Eigen::MatrixBase<D4>& qd_max,
                                      const Eigen::MatrixBase<D5>& qdd_max,
                                          const double& dt,
                                            const double& max_velocity_multiplier,
                                              const bool& preserve_direction,
                                                std::stringstream* report)
{
  if( eigen_utils::size(qd) != eigen_utils::size(q_prev)
  ||  eigen_utils::size(qd) != eigen_utils::size(q_max) 
  ||  eigen_utils::size(qd) != eigen_utils::size(q_min))
  {
    throw std::runtime_error( (__PRETTY_FUNCTION__ + std::string(":") + "inputs dimension mismatch.").c_str());
  }
  for(int i=0; i<eigen_utils::rows(qd);i++)
  {
    if(q_max(i)<=q_min(i))
      throw std::runtime_error( (__PRETTY_FUNCTION__ + std::string(":") + " The range is wrong. 'q_max="+
        std::to_string(q_max(i))+"' is less than 'q_min="+std::to_string(q_min(i))+"'").c_str());
  }
  bool saturated = saturateSpeed(qd,
                      qd_prev, qd_max, qdd_max, dt, max_velocity_multiplier, preserve_direction, report);

  if(report)
  {
    *report << "[-----][BRK   SATURATION] INPUT  qd: " << eigen_utils::to_string(qd) << "\n";
  }

  int nAx = eigen_utils::size(qd);
  MatD<D1> braking_distance, q_saturated_qd;
  eigen_utils::resize(braking_distance, nAx);
  eigen_utils::resize(q_saturated_qd  , nAx);
  for(int iAx=0; iAx<nAx;iAx++)
  {
    braking_distance(iAx)  = 0.5 * qdd_max(iAx)
                             * std::pow(std::abs(qd(iAx))/qdd_max(iAx) , 2.0);
  }

  q_saturated_qd = q_prev + qd_prev* dt;
  for(int iAx=0; iAx<nAx;iAx++)
  {
    if ((q_saturated_qd(iAx) > (q_max(iAx) - braking_distance(iAx))) && (qd(iAx)>0))
    {
      saturated = true;
      qd(iAx) = std::max(0.0, qd(iAx) - qdd_max(iAx) * dt);
    }
    else if((q_saturated_qd(iAx)<(q_min(iAx) + braking_distance(iAx))) && (qd(iAx)<0))
    {
      saturated = true;
      qd(iAx) = std::min(0.0, qd(iAx) + qdd_max(iAx) * dt);
    }
  }

  if(report)
  {
    *report << (saturated ? "[TRUE ]": "[FALSE]" )
            << "[BRK   SATURATION] OUTPUT qd: " << eigen_utils::to_string(qd) << "\n";
  }
  return saturated;
}


inline bool saturateSpeed(double& qd,
                            const double& q_prev,
                              const double& qd_prev,
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

  bool saturated = saturateSpeed(qd,
                      qd_prev, qd_max, qdd_max, dt, max_velocity_multiplier, preserve_direction, report);

  if(report)
  {
    *report << "[-----][BRK   SATURATION] INPUT  qd: " << eigen_utils::to_string(qd) << "\n";
  }
  double braking_distance;
  braking_distance  = 0.5 * qdd_max
                           * std::pow(std::abs(qd)/qdd_max, 2.0);

  double q_saturated_qd = q_prev + qd_prev* dt;
  if ((q_saturated_qd > (q_max - braking_distance)) && (qd>0))
  {
    saturated = true;
    qd = std::max(0.0, qd - qdd_max * dt);
  }
  else if((q_saturated_qd<(q_min + braking_distance)) && (qd<0))
  {
    saturated = true;
    qd = std::min(0.0, qd + qdd_max * dt);
  }

  if(report)
  {
    *report << (saturated ? "[TRUE ]": "[FALSE]" )
            << "[BRK   SATURATION] OUTPUT qd: " << eigen_utils::to_string(qd) << "\n";
  }
  return saturated;

}

// ==================================================================================


// ==================================================================================
template<typename D1, typename D2, typename D3, typename D4>
inline bool saturateSpeed(Eigen::MatrixBase<D1>& qd,
                            const Eigen::MatrixBase<D2>& qd_prev,
                              const Eigen::MatrixBase<D3>& qd_max,
                                const Eigen::MatrixBase<D4>& qdd_max,
                                    const double& dt,
                                      const double& max_velocity_multiplier,
                                        const bool& preserve_direction,
                                          std::stringstream* report)
{
  if( eigen_utils::size(qd) != eigen_utils::size(qd_prev)
  ||  eigen_utils::size(qd) != eigen_utils::size(qdd_max))
  {
    throw std::runtime_error( (__PRETTY_FUNCTION__ + std::string(":") + "inputs dimension mismatch.").c_str());
  }
  if(qd_max.minCoeff() < 1e-6)
  {
    throw std::runtime_error( (__PRETTY_FUNCTION__ + std::string(":") +
      " at least one joint has a qd max of is less or equal to zero").c_str()); 
  }
  if(qdd_max.minCoeff() < 1e-6)
  {
    throw std::runtime_error( (__PRETTY_FUNCTION__ + std::string(":") +
      " at least one joint has a qdd max of is less or equal to zero").c_str());  
  }
  if(report)
  {
    *report<<"[-----][ACC   SATURATION] INPUT  qd     : "<<eigen_utils::to_string(qd)<<"\n"
              <<"[-----][ACC   SATURATION] INPUT  qd prev: " << eigen_utils::to_string(qd_prev)<<"\n"
                <<"[-----][ACC   SATURATION] INPUT  qd  max: " << eigen_utils::to_string(qd_max * max_velocity_multiplier)
                  <<" (multiplier: " << max_velocity_multiplier
                    <<", preserve dir: " << preserve_direction<<")\n"
                      <<"[-----][ACC   SATURATION] INPUT  qdd max: " << eigen_utils::to_string(qdd_max)<<"\n";
  }

  int nAx = eigen_utils::rows(qd);

  MatD<D1> qd_sup, qd_inf, dqd;
  eigen_utils::resize(qd_sup, nAx);
  eigen_utils::resize(qd_inf, nAx);
  eigen_utils::resize(dqd   , nAx);

  qd_sup = qd_prev + qdd_max * dt;
  qd_inf = qd_prev - qdd_max * dt;
  eigen_utils::setZero(dqd);
  for(int iAx=0;iAx<nAx;iAx++)
  {
    dqd(iAx) = qd(iAx) > qd_sup(iAx) ? (qd_sup(iAx) - qd(iAx))
             : qd(iAx) < qd_inf(iAx) ? (qd_inf(iAx) - qd(iAx))
             : 0.0;
  }
  
  bool saturated = dqd.cwiseAbs().maxCoeff()>1e-5;
  
  if(report)
  {
    *report << "[-----][ACC   SATURATION] CALC   qd  sup: "  << eigen_utils::to_string(qd_sup) << "\n";
    *report << "[-----][ACC   SATURATION] CALC   qd  inf: "  << eigen_utils::to_string(qd_inf) << "\n";
    *report << "[-----][ACC   SATURATION] CALC   dqd    : "  << eigen_utils::to_string(dqd) << " => Saturated? " << (saturated ? "TRUE" : "FALSE") << "\n";
  }
  if(saturated)
  {
    if(preserve_direction)
    {
      Eigen::VectorXd dqd_dir = (qd - qd_prev).normalized();
      if(dqd.norm() < 1e-5)
      {
        dqd_dir.setZero();
      }

      if(dqd.minCoeff() * dqd.maxCoeff() >= 0.0)
      {
        qd = qd + (dqd.dot(dqd_dir) * dqd_dir);
      }
      else
      {
        if(report)
        {
          *report << "[-----][ACC   SATURATION] !!!!!! IT IS IMPOSSILBE TO PRESERVE DIRECTION (min dqd: " << dqd.minCoeff() <<", max dqd: "<< dqd.maxCoeff()  << ")\n";
        }
        qd = qd + dqd;
      }
    }
    else
    {
      qd = qd + dqd;
    }
  }
  else
  {
    // do nothing
  }


  double _max_velocity_multiplier = max_velocity_multiplier;
  for(int iAx=0;iAx<eigen_utils::size(qd_max);iAx++)
  {
    _max_velocity_multiplier = std::fabs(qd_prev(iAx)) > qd_max(iAx) 
                              ? std::max( _max_velocity_multiplier, 1.01 * std::fabs(qd_prev(iAx)) / qd_max(iAx) )
                              : _max_velocity_multiplier;
  }
  if(report)
  {
    *report <<"[-----][ACC   SATURATION] CALC   qd mul : " << max_velocity_multiplier 
              << " --> " << _max_velocity_multiplier <<"\n";
  }

  saturated |= saturateSpeed(qd, qd_max, max_velocity_multiplier, preserve_direction, report);
  if(report)
  {
    *report << (saturated ? "[TRUE ]": "[FALSE]" )
              <<"[ACC   SATURATION] OUTPUT qd     : "<<eigen_utils::to_string(qd)<< "\n";
  }
  return saturated;
}


inline bool saturateSpeed(double& qd,
                            const double& qd_prev,
                              const double& qd_max, 
                                const double& qdd_max,
                                  const double& dt,
                                    const double& max_velocity_multiplier, 
                                      const bool& preserve_direction, 
                                        std::stringstream* report)
{
  if(qd_max < 1e-6)
  {
    throw std::runtime_error( (__PRETTY_FUNCTION__ + std::string(":") +
      " qd max less or equal to zero").c_str()); 
  }
  if(qdd_max < 1e-6)
  {
    throw std::runtime_error( (__PRETTY_FUNCTION__ + std::string(":") +
      " qdd max less or equal to zero").c_str()); 
  }
  if(report)
  {
    *report<<"[-----][ACC   SATURATION] INPUT  qd     : "<<eigen_utils::to_string(qd)<<"\n"
              <<"[-----][ACC   SATURATION] INPUT  qd prev: " << eigen_utils::to_string(qd_prev)<<"\n"
                <<"[-----][ACC   SATURATION] INPUT  qd  max: " << eigen_utils::to_string(qd_max * max_velocity_multiplier)
                  <<" (multiplier: " << max_velocity_multiplier
                    <<", preserve dir: " << preserve_direction<<")\n"
                      <<"[-----][ACC   SATURATION] INPUT  qdd max: " << eigen_utils::to_string(qdd_max)<<"\n";
  }
  double qd_sup  = qd_prev + qdd_max * dt;
  double qd_inf  = qd_prev - qdd_max * dt;
  double dqd = 0;
  dqd  = qd > qd_sup ? (qd_sup - qd)
       : qd < qd_inf ? (qd_inf - qd)
       : 0.0;
  bool saturated = std::fabs(dqd)>0.0;

  if(report)
  {
    *report << "[-----][ACC   SATURATION] CALC   qd  sup: "  << eigen_utils::to_string(qd_sup) << "\n";
    *report << "[-----][ACC   SATURATION] CALC   qd  inf: "  << eigen_utils::to_string(qd_inf) << "\n";
  }
  qd = qd + dqd;

  double _max_velocity_multiplier = std::fabs(qd_prev) > qd_max 
                                  ? std::max( max_velocity_multiplier, 1.01 * std::fabs(qd_prev) / qd_max )
                                  : max_velocity_multiplier;
  if(report)
  {
    *report <<"[-----][ACC   SATURATION] CALC   qd mul: " << max_velocity_multiplier 
              << " --> " << _max_velocity_multiplier <<"\n";
  }

  saturated |= saturateSpeed(qd, qd_max, _max_velocity_multiplier, preserve_direction, report);
  if(report)
  {
    *report << (saturated ? "[TRUE ]": "[FALSE]" )
              <<"[ACC   SATURATION] OUTPUT qd     : "<<eigen_utils::to_string(qd)<< "\n";
  }

  return saturated;
}
// ==================================================================================





// ==================================================================================
template<typename D1, typename D2>
inline bool saturateSpeed(Eigen::MatrixBase<D1>& qd,
                            const Eigen::MatrixBase<D2>& qd_max,
                              const double& max_velocity_multiplier,
                                const bool& preserve_direction,
                                  std::stringstream* report)
{    
  const auto _qd_max = qd_max * max_velocity_multiplier;
  if(report)
  {
    *report << "[-----][SPEED SATURATION] INPUT  qd     : " << eigen_utils::to_string(qd) << "\n"
              <<"[-----][SPEED SATURATION] INPUT  qd max : " << eigen_utils::to_string(_qd_max)
                <<" (multiplier: " << max_velocity_multiplier
                  <<", preserve dir: " << preserve_direction<<")\n";
  }

  if( eigen_utils::size(qd) != eigen_utils::size(qd_max) )
  {
    throw std::runtime_error( (__PRETTY_FUNCTION__ + std::string(":") + "inputs dimension mismatch.").c_str());
  }
  for(int iAx=0; iAx<eigen_utils::rows(qd_max);iAx++)
  {
    if(eigen_utils::at(qd_max,iAx) < 1e-6)
    {
      throw std::runtime_error( (__PRETTY_FUNCTION__ + std::string(":") +
        " qd max of the ax " + std::to_string(iAx) +" less or equal to zero").c_str()); 
    }
  }

  int nAx = eigen_utils::rows(qd);

  MatD<D1> scale;
  eigen_utils::resize(scale, nAx);
  

  for(int iAx=0;iAx<nAx;iAx++)
  {
    scale(iAx) = std::fabs(qd(iAx)) > _qd_max(iAx)
               ? _qd_max(iAx)/ std::fabs(qd(iAx) )
               : 1.0;
  }

  if(preserve_direction)
  {
    qd = scale.minCoeff() * qd;
  }
  else
  {
    qd = scale.asDiagonal() * qd;
  }

  if(report)
  {
    *report << (scale.minCoeff()<1 ? "[TRUE ]": "[FALSE]" )
            << "[SPEED SATURATION] OUTPUT qd     : " << eigen_utils::to_string(qd) << "\n";
  }

  return scale.minCoeff()<1;
}

inline bool saturateSpeed(double& qd, 
                            const double& qd_max, 
                              const double& max_velocity_multiplier,
                                const bool& preserve_direction, 
                                  std::stringstream* report)
{
  const double _qd_max =  qd_max * max_velocity_multiplier;
  if(report)
  {
    *report << "[-----][SPEED SATURATION] INPUT  qd     : " << eigen_utils::to_string(qd) << "\n"
              <<"[-----][SPEED SATURATION] INPUT  qd max : " << eigen_utils::to_string(_qd_max)
                <<" (multiplier: " << max_velocity_multiplier
                  <<", preserve dir: " << preserve_direction<<")\n";
  }
  if(qd_max < 1e-6)
  {
    throw std::runtime_error( (__PRETTY_FUNCTION__ + std::string(":") +
      " qd max less or equal to zero").c_str()); 
  }


  double scale   = std::fabs(qd) > _qd_max ? _qd_max / std::fabs(qd)
                 : 1.0;

  qd = scale * qd;

  if(report)
  {
    *report << (scale<1 ? "[TRUE ]": "[FALSE]" )
            << "[SPEED SATURATION] OUTPUT qd     : " << eigen_utils::to_string(qd) << "\n";
  }

  return scale<1;
}

// ==================================================================================


// ==================================================================================
template<typename D1, typename D2, typename D3>
inline bool saturatePosition(Eigen::MatrixBase<D1>& q,
                              const Eigen::MatrixBase<D2>& q_max, 
                                const Eigen::MatrixBase<D3>& q_min,
                                  std::stringstream* report)
{
  if(report)
  {
    *report << "[-----][POS   SATURATION] INPUT  q: " << eigen_utils::to_string(q) << "\n";
    *report << "[-----][POS   SATURATION] MAX    q: " << eigen_utils::to_string(q_max) << "\n";
    *report << "[-----][POS   SATURATION] MIN    q: " << eigen_utils::to_string(q_min) << "\n";
  }
  if( eigen_utils::size(q) != eigen_utils::size(q_max) 
   || eigen_utils::size(q) != eigen_utils::size(q_min))
  {
    throw std::runtime_error( (__PRETTY_FUNCTION__ + std::string(":") + " Inputs dimension mismatch.").c_str());
  }
  for(int i=0; i<eigen_utils::rows(q);i++)
  {
    if(q_max(i)<=q_min(i))
      throw std::runtime_error( (__PRETTY_FUNCTION__ + std::string(":") + " The range is wrong. 'q_max' is less than 'q_min'").c_str());
  }


  MatD<D1> dq;
  eigen_utils::resize(dq, q.rows());
  int nAx = eigen_utils::size(q);
  for(int iAx=0;iAx<nAx;iAx++)
  {
    dq(iAx)  = q(iAx) > q_max(iAx) ? (q_max(iAx) - q(iAx))
             : q(iAx) < q_min(iAx) ? (q_min(iAx) - q(iAx))
             : 0.0;
  }

  q += dq;
  if(report)
  {
    *report << (dq.cwiseAbs().maxCoeff()>0.0 ? "[TRUE ]": "[FALSE]" )
            << "[POS   SATURATION] OUTPUT q: " << eigen_utils::to_string(q) << "\n";
  }

  return (dq.cwiseAbs().maxCoeff()>0.0);
}


inline bool saturatePosition(double& q, 
                              const double& q_max,
                                const double& q_min,
                                  std::stringstream* report)
{
  if(report)
  {
    *report << "[-----][POS   SATURATION] INPUT  q: " << eigen_utils::to_string(q) << "\n";
    *report << "[-----][POS   SATURATION] MAX    q: " << eigen_utils::to_string(q_max) << "\n";
    *report << "[-----][POS   SATURATION] MIN    q: " << eigen_utils::to_string(q_min) << "\n";
  }
  if(q_max<=q_min)
  {
    throw std::runtime_error( (__PRETTY_FUNCTION__ + std::string(":") + "The range is wrong. 'q_max' is less than 'q_min'").c_str());
  }

  double dq;
  dq   = q > q_max ? (q_max - q)
       : q < q_min ? (q_min + q)
       : 0.0;

  q += dq;

  if(report)
  {
    *report << (std::fabs(dq)>0.0 ? "[TRUE ]": "[FALSE]" )
            << "[POS   SATURATION] OUTPUT q: " << eigen_utils::to_string(q) << "\n";
  }

  return (std::fabs(dq)>0.0);
}
// ==================================================================================


}  // namespace cnr_control_toolbox

#endif  // KINEMATICS_FILTERS__KINEMATICS_FILTERS_IMPL__H
