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


#include <frequency_identification/frequency_identification.h>

namespace  identification
{

MultiSineEstimator::MultiSineEstimator(const ros::NodeHandle& nh,
                                       const cnr_logger::TraceLoggerPtr& logger):
  m_nh(nh),
  m_rng(m_rd())
{
  if (logger)
  {
    m_logger=logger;
  }
  else
  {
    std::string path=m_nh.getNamespace();
    std::string logger_id;
    if(!m_nh.getParam("file_name", logger_id))
      logger_id="multisine_estimation";
    m_logger=std::make_shared<cnr_logger::TraceLogger>();
    m_logger->init(logger_id,path);
  }

  std::vector<double> fr_real;
  std::vector<double> fr_imag;
  std::vector<double> fr_omega;

  if(!m_nh.getParam("frequency_response/angular_frequency", fr_omega))
  {
    fr_omega.clear();
  }
  if(!m_nh.getParam("frequency_response/real", fr_real))
  {
    fr_real.clear();
  }
  if(!m_nh.getParam("frequency_response/imag", fr_imag))
  {
    fr_imag.clear();
  }
  if (fr_omega.size()!=fr_real.size() || fr_omega.size() != fr_real.size() )
  {
    CNR_WARN(m_logger,"frequency_response sizes do not match");
    fr_real.clear();
    fr_imag.clear();
    fr_omega.clear();
  }

  m_freq_resp.resize(fr_omega.size());
  m_omega.resize(fr_omega.size());
  for (size_t idx=0;idx<fr_omega.size();idx++)
  {
    m_omega(idx)=fr_omega.at(idx);
    m_freq_resp(idx)=std::complex<double>(fr_real.at(idx),fr_imag.at(idx));
  }


  if (not loadParam())
    throw std::invalid_argument("unable to initialize multisine estimator");
}

MultiSineEstimator::~MultiSineEstimator()
{
  if (m_gen_thread.joinable())
    m_gen_thread.join();
}

bool MultiSineEstimator::loadParam()
{

  if(!m_nh.getParam("carrier/angular_frequency", m_carrier_frequency))
  {
    CNR_RETURN_FALSE(m_logger,"The param '"+m_nh.getNamespace()+"carrier/angular_frequency' does not exist");
  }
  m_carrier_period = 2.0*M_PI/m_carrier_frequency;
  if(!m_nh.getParam("carrier/amplitude", m_carrier_amplitude))
  {
    CNR_RETURN_FALSE(m_logger,"The param '"+m_nh.getNamespace()+"/carrier/amplitude' does not exist");
  }
  if(!m_nh.getParam("carrier/periods", m_carrier_periods))
  {
    CNR_RETURN_FALSE(m_logger,"The param '"+m_nh.getNamespace()+"/carrier/periods' does not exist");
  }
  m_carrier_periods=std::ceil(m_carrier_periods);
  if (m_carrier_periods<1.0)
  {
    CNR_RETURN_FALSE(m_logger,"The param '"+m_nh.getNamespace()+"/carrier/periods' should be greater than 1.0");
  }
  m_test_time=m_carrier_periods*2.0*M_PI/m_carrier_frequency;

  if(!m_nh.getParam("rampup_time", m_rampup_time))
  {
    CNR_RETURN_FALSE(m_logger,"The param '"+m_nh.getNamespace()+"/rampup_time' does not exist");
  }

  double min_angular_frequency;
  if(!m_nh.getParam("input/min_angular_frequency", min_angular_frequency))
  {
    CNR_RETURN_FALSE(m_logger,"The param '"+m_nh.getNamespace()+"input/min_angular_frequency' does not exist");
  }
  m_min_harmonic=std::round(min_angular_frequency/m_carrier_frequency);

  double max_angular_frequency;
  if(!m_nh.getParam("input/max_angular_frequency", max_angular_frequency))
  {
    CNR_RETURN_FALSE(m_logger,"The param '"+m_nh.getNamespace()+"input/max_angular_frequency' does not exist");
  }
  m_max_harmonic=std::round(max_angular_frequency/m_carrier_frequency);

  if(!m_nh.getParam("input/harmonics_number", m_harmonics_number))
  {
    CNR_RETURN_FALSE(m_logger,"The param '"+m_nh.getNamespace()+"input/harmonics_number' does not exist");
  }
  if (m_harmonics_number>(m_max_harmonic-m_min_harmonic))
    m_harmonics_number=(m_max_harmonic-m_min_harmonic);

  if(!m_nh.getParam("input/max_pos", m_max_pos))
  {
    CNR_RETURN_FALSE(m_logger,"The param '"+m_nh.getNamespace()+"input/max_pos' does not exist");
  }
  if(!m_nh.getParam("input/max_vel", m_max_vel))
  {
    CNR_RETURN_FALSE(m_logger,"The param '"+m_nh.getNamespace()+"input/max_vel' does not exist");
  }
  if(!m_nh.getParam("input/max_acc", m_max_acc))
  {
    CNR_RETURN_FALSE(m_logger,"The param '"+m_nh.getNamespace()+"input/max_acc' does not exist");
  }


  if(!m_nh.getParam("rampup_time", m_rampup_time))
  {
    CNR_RETURN_FALSE(m_logger,"The param '"+m_nh.getNamespace()+"/warmup_time' does not exist");
  }


  CNR_RETURN_BOOL(m_logger,true);
}

void MultiSineEstimator::getCommand(const double& t, double& x, double& dx, double& ddx)
{
  x   = 0.0;
  dx  = 0.0;
  ddx = 0.0;

  std::complex<double> i(0.0,1.0);
  for (const std::pair<double,std::complex<double>>& p: m_spetrum_command)
  {
    x   += (                          (p.second*std::exp(i*p.first * t) + std::conj(p.second)*std::exp(-i*p.first * t)) ).real();
    dx  += ( i*p.first              * (p.second*std::exp(i*p.first * t) - std::conj(p.second)*std::exp(-i*p.first * t)) ).real();
    ddx += ( -std::pow(p.first,2.0) * (p.second*std::exp(i*p.first * t) + std::conj(p.second)*std::exp(-i*p.first * t)) ).real();
  }

  double ratio=1.0;
  if (t<m_rampup_time)
  {
    double tmp=t/m_rampup_time;
    ratio=3*std::pow(tmp,2.0)-2*std::pow(tmp,3.0);
  }
  else if (t>(m_carrier_periods*m_carrier_period+2.0*m_rampup_time))
  {
    ratio=0.0;
  }
  else if (t>(m_carrier_periods*m_carrier_period+m_rampup_time))
  {
    double tmp=((m_carrier_periods*m_carrier_period+2*m_rampup_time)-t)/m_rampup_time;
    ratio=3*std::pow(tmp,2.0)-2*std::pow(tmp,3.0);
  }

  x   *= ratio;
  dx  *= ratio;
  ddx *= ratio;

}

void MultiSineEstimator::initTest(const double& dt)
{
  if (m_state==state::GeneratingInput)
  {
    ROS_FATAL("time =%f, state = %d",m_time,m_state);
    CNR_WARN(m_logger,"signal generation is in executiong, waiting for the result");
    return;
  }
  if (m_gen_thread.joinable())
    m_gen_thread.join();
  m_state=state::GeneratingInput;
  m_time=0.0;
  m_gen_thread=std::thread(&MultiSineEstimator::generatingSignalThread,this,dt);
}

void MultiSineEstimator::generatingSignalThread(const double &dt)
{
  ros::WallTime t0=ros::WallTime::now();
  generateCommandSignal(dt);
  m_spetrum_output.clear();
  for (const std::pair<double,std::complex<double>>& p: m_spetrum_command)
  {
    std::complex<double> c(0.0,0.0);
    std::pair<double,std::complex<double>> freq_pair(p.first,c);
    m_spetrum_output.insert(freq_pair);
  }
  ros::WallTime t1=ros::WallTime::now();
  CNR_INFO(m_logger,"signal generated in "<<(t1-t0).toSec()<<" seconds");
  m_state=state::Running;
  return;
}

void MultiSineEstimator::setOutput(const double& y, const double& t, const double& dt)
{
  if ( (t>m_rampup_time) && (t<(m_rampup_time+m_carrier_periods*m_carrier_period)) )
  {
    std::complex<double> i(0.0,1.0);
    for (const std::pair<double,std::complex<double>>& p: m_spetrum_command)
    {
      std::complex<double> c=m_spetrum_output.at(p.first);
      c+=dt*y*std::exp(-i*p.first*t)/m_carrier_period/m_carrier_periods;
      m_spetrum_output.at(p.first)=c;
    }
  }
}

double MultiSineEstimator::getExperimentTime() const
{
  return (m_carrier_periods*m_carrier_period+2.0*m_rampup_time);
}

void MultiSineEstimator::printFreqResp() const
{
  ROS_INFO_STREAM("omega = " << m_omega.transpose() << "\nfrequency response = " << m_freq_resp.transpose());
}

state MultiSineEstimator::execute(const double& dt, const double& y, double& x, double& dx, double& ddx)
{
  // transitions
  if (m_state==state::Running && m_time>=getExperimentTime())
  {
    m_state=state::Complete;
  }
  else if (m_state==state::Complete)
  {
    m_state=state::Idle;
    saveFreqResp();
  }

  // states
  x=0.0;
  dx=0.0;
  ddx=0.0;
  if (m_state==state::Running)
  {
    setOutput(y,m_time,dt);
    getCommand(m_time,x,dx,ddx);
    m_time+=dt;
  }
  else if (m_state==state::Complete)
  {
    computeFreqResp();
  }
  return m_state;
}

void MultiSineEstimator::generateCommandSignal(const double& dt)
{
  m_gen=std::normal_distribution<double>(0.0,1.0);
  std::vector<int> tmp(m_max_harmonic-m_min_harmonic+1);
  std::iota(tmp.begin(),tmp.end(),m_min_harmonic);


  std::shuffle(tmp.begin(), tmp.end(), m_rng);
  m_spetrum_command.clear();
  for (int idx=0;idx<m_harmonics_number;idx++)
  {
    double freq=tmp.at(idx)*m_carrier_frequency;
    std::complex<double> c(m_gen(m_rng),m_gen(m_rng));
    std::pair<double,std::complex<double>> freq_pair(freq,c/std::pow(freq,2.0));
    m_spetrum_command.insert(freq_pair);
  }

  double max_x   = 0.0;
  double max_dx  = 0.0;
  double max_ddx = 0.0;

  for (double t=0;t<getExperimentTime();t+=dt)
  {
    double x,dx,ddx;
    getCommand(t,x,dx,ddx);
    x   = std::abs(  x);
    dx  = std::abs( dx);
    ddx = std::abs(ddx);

    max_x   =   x>  max_x?   x:   max_x;
    max_dx  =  dx> max_dx?  dx:  max_dx;
    max_ddx = ddx>max_ddx? ddx: max_ddx;
  }
  double scale=1.0;
  scale = m_max_pos/max_x   < scale ? m_max_pos/max_x   : scale;
  scale = m_max_vel/max_dx  < scale ? m_max_vel/max_dx  : scale;
  scale = m_max_acc/max_ddx < scale ? m_max_acc/max_ddx : scale;

  for (int idx=0;idx<m_harmonics_number;idx++)
  {
    double freq=tmp.at(idx)*m_carrier_frequency;
    std::complex<double> c=m_spetrum_command.at(freq);
    m_spetrum_command.at(freq) = c*scale;
  }
}

void MultiSineEstimator::computeFreqResp()
{
  for (const std::pair<double,std::complex<double>>& p: m_spetrum_command)
  {
    std::complex<double> c_in=p.second;
    std::complex<double> c_out=m_spetrum_output.at(p.first);
    std::complex<double> fr = c_out/c_in;

    m_freq_resp.conservativeResize(m_freq_resp.size()+1);
    m_freq_resp(m_freq_resp.size()-1)=fr;

    m_omega.conservativeResize(m_omega.size()+1);
    m_omega(m_omega.size()-1)=p.first;
  }
}

void MultiSineEstimator::saveFreqResp()
{
  std::vector<double> w;
  std::vector<double> freq_resp_real;
  std::vector<double> freq_resp_imag;
  for (int idx=0;idx<m_omega.size();idx++)
  {
    w.push_back(m_omega(idx));
    freq_resp_real.push_back(m_freq_resp(idx).real());
    freq_resp_imag.push_back(m_freq_resp(idx).imag());
  }
  m_nh.setParam("frequency_response/real",freq_resp_real);
  m_nh.setParam("frequency_response/imag",freq_resp_imag);
  m_nh.setParam("frequency_response/angular_frequency",w);
}

}  // end namespace identification


