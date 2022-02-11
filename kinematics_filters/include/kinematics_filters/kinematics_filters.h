#ifndef KINEMATICS_FILTERS__KINEMATICS_FILTERS__H
#define KINEMATICS_FILTERS__KINEMATICS_FILTERS__H

#include <Eigen/Dense>
#include <eigen_matrix_utils/eigen_matrix_utils.h>
#include <eigen_matrix_utils/overloads.h>

namespace cnr_control_toolbox
{

/**
 * @brief  * @tparam D1 The template has 3 arguments to manage also different kind of inputs (a colum of a matrix, a row of a matrix, a vector etc).
 *            The user usually has not to explicit the template params, since they are aumatically recognized.
 * @tparam D2 see D2
 * @tparam D3 see D1
 * @tparam D4 see D1
 * @tparam D5 see D1
 * @tparam D6 see D1
 * @tparam D7 see D1
 
 * @param qd_target 
 * @param qd_actual 
 * @param q_actual 
 * @param qd_max 
 * @param qdd_max 
 * @param dt 
 * @param max_velocity_multiplier 
 * @param preserve_direction 
 * @param report 
 * @return true 
 * @return false 
 */
template<typename D1,typename D2,typename D3,typename D4,typename D5,typename D6,typename D7>
bool saturateSpeedFullState(Eigen::MatrixBase<D1>&         qd_target,
                    const Eigen::MatrixBase<D3>&   q_actual,  
                      const Eigen::MatrixBase<D2>&   qd_actual,
                        const Eigen::MatrixBase<D4>&   q_max,
                          const Eigen::MatrixBase<D5>&   q_min,
                            const Eigen::MatrixBase<D6>&   qd_max,
                              const Eigen::MatrixBase<D7>&   qdd_max,
                                const double&                  dt,
                                  const double&                  max_velocity_multiplier,
                                    const bool&                    preserve_direction,
                                      std::stringstream*             report);


//! SPECIAL CASE 1DOF
bool saturateSpeedFullState(double&           qd_target,
                    const double&      q_actual,
                      const double&      qd_actual,
                        const double&      q_max,
                          const double&      q_min,
                            const double&      qd_max,
                              const double&      qdd_max,
                                const double&      dt,
                                  const double&      max_velocity_multiplier,
                                    const bool&        preserve_direction,
                                      std::stringstream* report);


/**
 * @brief 
 * 
 * @tparam D1 
 * @tparam D2 
 * @tparam D3 
 * @tparam D4 
 * @param qd_target 
 * @param qd_actual 
 * @param qd_max 
 * @param qdd_max 
 * @param dt 
 * @param max_velocity_multiplier 
 * @param preserve_direction 
 * @param report 
 * @return true 
 * @return false 
 */
template<typename D1,typename D2,typename D3,typename D4>
bool saturateSpeedFirstOrderState(Eigen::MatrixBase<D1>&       qd_target,
                    const Eigen::MatrixBase<D2>& qd_actual,
                      const Eigen::MatrixBase<D3>& qd_max,
                        const Eigen::MatrixBase<D4>& qdd_max,
                          const double&                dt,
                            const double&                max_velocity_multiplier,
                              const bool&                  preserve_direction,
                                std::stringstream*           report);

/// Special case 1 dof
bool saturateSpeedFirstOrderState(double&            qd_target,
                    const double&      qd_actual,
                      const double&      qd_max,
                        const double&      qdd_max,
                          const double&      dt,
                            const double&      max_velocity_multiplier,
                              const bool&        preserve_direction,
                                std::stringstream* report);


/**
 * @brief 
 * 
 * @tparam D1 
 * @tparam D2 
 * @param qd_target 
 * @param qd_max 
 * @param max_velocity_multiplier 
 * @param preserve_direction 
 * @param report 
 * @return true 
 * @return false 
 */
template<typename D1, typename D2>
bool saturateSpeed(Eigen::MatrixBase<D1>&       qd_target,
                    const Eigen::MatrixBase<D2>& qd_max,
                      const double&                max_velocity_multiplier,
                        const bool&                  preserve_direction,
                          std::stringstream*           report);

/// Special case 1 DoF
bool saturateSpeed(double&            qd_target,
                    const double&      qd_max,
                      const double&      max_velocity_multiplier,
                        const bool&        preserve_direction,
                          std::stringstream* report);

/**
 * @brief 
 * 
 * @tparam D1 
 * @tparam D2 
 * @tparam D3 
 * @param q_target 
 * @param q_max 
 * @param q_min 
 * @param report 
 * @return true 
 * @return false 
 */
template<typename D1, typename D2, typename D3>
bool saturatePosition(Eigen::MatrixBase<D1>&        q_target,
                        const Eigen::MatrixBase<D2>&  q_max,
                          const Eigen::MatrixBase<D3>&  q_min,
                            std::stringstream*            report);

/// Special Case 1 DoF
bool saturatePosition(double&             q_target,
                        const double&       q_max, 
                          const double&       q_min, 
                            std::stringstream*  report);

}  // namespace cnr_control_toolbox

/// Implementations
#include <kinematics_filters/internal/kinematics_filters_impl.h>

#endif  // KINEMATICS_FILTERS__KINEMATICS_FILTERS__H
