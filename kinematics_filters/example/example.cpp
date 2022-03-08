#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <Eigen/Core>

#include <kinematics_filters/kinematics_filters.h>

const Eigen::Matrix<double,6,6> D6 = Eigen::Matrix<double,6,1>::Ones().asDiagonal();
const Eigen::Matrix<double,6,1> I6 = Eigen::Matrix<double,6,1>::Ones();
const Eigen::Matrix<double,6,1> Z6 = Eigen::Matrix<double,6,1>::Zero();


int main(int argc, char* argv[])
{
    Eigen::VectorXd q_max = 10 * I6  + 2 * Eigen::Matrix<double,6,1>::Random() ;
    Eigen::VectorXd q_min = -q_max;

    Eigen::VectorXd qd_max = 1.2 * I6;
    Eigen::VectorXd qdd_max = 250 * qd_max;

// Target inside
    Eigen::Matrix<double,6,6> R6q = Eigen::Matrix<double,6,1>::Random().asDiagonal();
    Eigen::Matrix<double,6,6> R6q_prev = Eigen::Matrix<double,6,1>::Random().asDiagonal();

    Eigen::VectorXd q_target = q_min + 0.5 * (D6 + R6q ) * (q_max - q_min);
    Eigen::VectorXd q_actual = q_min + 0.1 * (D6 + R6q_prev) * (q_max - q_min);

    std::stringstream report;
    if(cnr_control_toolbox::saturatePosition(q_target, q_max, q_min, &report))
    {
        std::cout << report.str() << std::endl;
    }

    double dt = 0.001;
    Eigen::VectorXd qd_prev = Z6;
    Eigen::VectorXd q_prev = q_actual;
    bool saturated = false;
    size_t cnt = 0;
    std::ofstream ofile("test_kinematics_filter.plt", std::ofstream::out);
    ofile << "t," << "qt1,qt2,qt3,qt4,qt5,qt6," 
        << "q1,q2,q3,q4,q5,q6,"  
        << "qdt1,qdt2,qdt3,qdt4,qdt5,qdt6,"
        << "qd1,qd2,qd3,qd4,qd5,qd6"<< std::endl;
    do
    {
        Eigen::VectorXd qd = (q_target-q_prev) / dt;
        report << cnt++ << "-------" << std::endl;
        saturated |= cnr_control_toolbox::saturateSpeed(qd, q_prev, qd_prev, q_max, q_min, qd_max, qdd_max, dt, 1.0, true, &report);

        Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "", "");
        ofile  << cnt * dt<<","<< q_target.transpose().format(CommaInitFmt)<<", "
                << q_prev.transpose().format(CommaInitFmt) << ","
                << qd.transpose().format(CommaInitFmt)<<", "
                << qd_prev.transpose().format(CommaInitFmt) << std::endl;

        q_prev = q_prev + qd;
        qd_prev = qd;
        auto dist = (q_target-q_prev);
        std::cout.precision(6);
        Eigen::IOFormat fmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
        std::cout << std::fixed 
            << "qd: " << qd_prev.transpose().format(fmt) << " dist: " << dist.transpose().format(fmt) << " norm: " << dist.norm() <<std::endl;


        if(dist.norm()<1e-3)
        {
            break;
        }
    } while(1);

    if(saturated)
    {
        std::cout << report.str() << std::endl;
    }

    return 1;
}