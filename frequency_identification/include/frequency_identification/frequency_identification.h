/*
Copyright (c) 2020, JRL-CARI CNR-STIIMA/UNIBS
Manuel Beschi manuel.beschi@unibs.it
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once
#include <ros/ros.h>
#include <Eigen/Dense>
#include <cnr_logger/cnr_logger.h>
#include <complex>
#include <thread>
#include <mutex>
#include <random>



namespace  identification
{
enum state {Idle, GeneratingInput, Running, Complete} ;
class MultiSineEstimator
{
public:
  MultiSineEstimator(const ros::NodeHandle& nh,
                     const cnr_logger::TraceLoggerPtr& logger=NULL);

  ~MultiSineEstimator();
  bool loadParam();
  state execute(const double& dt, const double &y, double& x, double& dx, double& ddx);
  void initTest(const double &dt);

  void printFreqResp() const;
  double getExperimentTime() const;
  state getState();
  void saveFreqResp();
protected:
  ros::NodeHandle m_nh;
  std::random_device m_rd;
  std::mt19937 m_rng;
  std::normal_distribution<double> m_gen;
  double m_carrier_frequency;
  double m_carrier_period;
  double m_carrier_amplitude;
  double m_carrier_periods;
  double m_test_time;
  double m_rampup_time=5.0;
  double m_time=0.0;

  double m_max_pos;
  double m_max_vel;
  double m_max_acc;
  int    m_min_harmonic;
  int    m_max_harmonic;
  int    m_harmonics_number;
  state m_state;

  std::thread m_gen_thread;

  std::map<double,std::complex<double>> m_spetrum_command;
  std::map<double,std::complex<double>> m_spetrum_output;
  Eigen::VectorXcd m_freq_resp;
  Eigen::VectorXd m_omega;
  cnr_logger::TraceLoggerPtr m_logger;

  void generateCommandSignal(const double &dt);
  void computeFreqResp();
  std::complex<double> computeFourierCoefficient(const double& freq);
  void getCommand(const double& t, double& x, double& dx, double& ddx);
  void setOutput(const double& y, const double &t, const double &dt);

  void generatingSignalThread(const double &dt);
};

typedef std::shared_ptr<MultiSineEstimator> MultiSineEstimatorPtr;
}  // end namespace identification
