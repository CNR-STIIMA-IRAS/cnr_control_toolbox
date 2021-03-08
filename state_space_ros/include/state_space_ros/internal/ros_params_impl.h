#pragma once

#ifndef STATE_SPACE_ROS__ROS_PARAMS_IMPL_H
#define STATE_SPACE_ROS__ROS_PARAMS_IMPL_H

#include <state_space_ros/ros_params.h>

//! @brief The implementation of all the classes and functions for the management of a Discreate Space System
namespace eigen_control_toolbox
{

inline std::string to_string(const std::string& what, const bool& ret, const std::string& msg)
{
  return what + (what.size()>0?"\n":"") +
    ( ret ? (msg.size()==0? "" : "[?]" + msg) : "[!]" + msg);
}

inline std::string dim(const Eigen::MatrixXd& m)
{
  return "(" + std::to_string(eu::rows(m)) +"x"+std::to_string(eu::cols(m)) + ")";
}

inline bool importMatricesFromParam(const ros::NodeHandle&  nh,
                                    const std::string&      name,
                                    Eigen::MatrixXd&        A,
                                    Eigen::MatrixXd&        B,
                                    Eigen::MatrixXd&        C,
                                    Eigen::MatrixXd&        D,
                                    std::string&            what)
{
  what="";
  std::string type = "state-space";
  if(!rosparam_utilities::getParam(nh,name+"/type",type, what, &type))
  {
    return false;
  }

  if (!type.compare("unity"))
  {
    A.resize(1,1);
    B.resize(1,1);
    C.resize(1,1);
    D.resize(1,1);
    A.setZero();
    B.setZero();
    C.setZero();
    D(0,0)=1;
    return true;
  }

  if(!rosparam_utilities::getParam(nh, name+"/A", A, what))
  {
    return false;
  }
  if(!rosparam_utilities::getParam(nh, name+"/B", B, what))
  {
    return false;
  }
  if(!rosparam_utilities::getParam(nh, name+"/C", C, what))
  {
    return false;
  }
  if(!rosparam_utilities::getParam(nh, name+"/D", D, what))
  {
    return false;
  }
  return true;
}


template<int S, int I, int O, int MS, int MI, int MO>
inline int setMatricesFromParam(DiscreteStateSpace<S,I,O,MS,MI,MO>& out,
                            const ros::NodeHandle& nh,
                              const std::string& name,
                                std::string& what)
{
  what="";
  Eigen::MatrixXd A;
  Eigen::MatrixXd B;
  Eigen::MatrixXd C;
  Eigen::MatrixXd D;
  DiscreteStateSpaceArgs<S,I,O,MS,MI,MO> args;

  std::string msg;
  bool ok = importMatricesFromParam(nh, name, A,B,C,D, msg);
  what = to_string(what, ok,msg);
  if(!ok)
  {
     return -1;
  }

  int num_s = eu::rows(A);
  int num_i = eu::cols(B);
  int num_o = eu::rows(C);
  std::vector<std::pair<bool, std::string>> operations =
  { {num_s==eu::cols(A), "A is not a squared " + dim(A) + " (num states: " + std::to_string(num_s) +")"},
    {num_s==eu::rows(B), "B rows differ from A rows [B:"+dim(B)+", A:"+dim(A)+"]"},
    {num_s==eu::cols(C), "C cols differ from A rows [C:"+dim(C)+", A:"+dim(A)+"]"},
    {num_i==eu::cols(D), "D cols differ from B cols [D:"+dim(D)+", A:"+dim(B)+"]"},
    {num_o==eu::rows(D), "D rows differ from C rows [D:"+dim(D)+", C:"+dim(C)+"]"},
    {eu::resize(args.A, A.rows(), A.cols()), "A cannot be resized as the param A. Wrong static allocation?"},
    {eu::resize(args.B, B.rows(), B.cols()), "B cannot be resized as the param B. Wrong static allocation?"},
    {eu::resize(args.C, C.rows(), C.cols()), "C cannot be resized as the param C. Wrong static allocation?"},
    {eu::resize(args.D, D.rows(), D.cols()), "D cannot be resized as the param D. Wrong static allocation?"},
    {eu::copy  (args.A, A), "Weird error in storing the param A. Wrong static allocation?"},
    {eu::copy  (args.B, B), "Weird error in storing the param B. Wrong static allocation?"},
    {eu::copy  (args.C, C), "Weird error in storing the param C. Wrong static allocation?"},
    {eu::copy  (args.D, D), "Weird error in storing the param D. Wrong static allocation?"},  };

  for(auto const & c: operations)
  {
    what += c.first ? "" : (what.size()>0?"\n":"") + c.second;
    ok &= c.first;
  }
  if(!ok)
  {
    return -1;
  }

  return out.setMatrices(args,what);
}


template< int K,int N,int MK,int MN>
inline int setMatricesFromParam(IntegralStateSpace<K,N,MK,MN>& out,
                                  const ros::NodeHandle& nh,
                                    const std::string& name,
                                      std::string& what)
{
  what="";

  std::string type = "integral-state-space";
  if(!rosparam_utilities::getParam(nh,name+"/type",type, what, &type))
  {
    return false;
  }

  if (!type.compare("integral-state-space"))
  {
    what += "Error, the type of the matrices is not 'integral-state-space'";
    return -1;
  }

  IntegralStateSpaceArgs args;

  args.order = 1;
  if(!rosparam_utilities::getParam(nh,name+"/order",args.order, what, &args.order))
  {
    return -1;
  }

  args.degrees_of_freedom = 1;
  if(!rosparam_utilities::getParam(nh,name+"/dof",args.degrees_of_freedom, what, &args.degrees_of_freedom))
  {
    return -1;
  }

  return out.setMatrices(args,what);
}


template< int K,int N,int MK,int MN>
inline int setMatricesFromParam(IntegralDiscreteStateSpace<K,N,MK,MN>& out,
                                  const ros::NodeHandle& nh,
                                    const std::string& name,
                                      std::string& what)
{
  what="";
  std::string type = "integral-discrete-state-space";
  if(!rosparam_utilities::getParam(nh,name+"/type",type, what, &type))
  {
    return -1;
  }

  if (!type.compare("integral-discrete-state-space"))
  {
    what += "Error, the type of the matrices is not 'integral-discrete-state-space'";
    return -1;
  }

  IntegralDiscreteStateSpaceArgs args;

  args.order = 1;
  if(!rosparam_utilities::getParam(nh,name+"/order",args.order, what, &args.order))
  {
    return -1;
  }

  args.degrees_of_freedom = 1;
  if(!rosparam_utilities::getParam(nh,name+"/dof",args.degrees_of_freedom, what, &args.degrees_of_freedom))
  {
    return -1;
  }

  args.dt = 0;
  if(!rosparam_utilities::getParam(nh,name+"/dt",args.dt, what, &args.dt))
  {
    return -1;
  }

  return out.setMatrices(args,what);
}

template<int N, int MN>
inline int setMatricesFromParam(Controller<N,MN>& out,
    const ros::NodeHandle& nh,const std::string& name,std::string& what)
{
  int ret = 1;
  std::map<std::string,std::vector<std::string>> _types =
  { {"P"  , {"proportional_controller", "p_controller", "P_controller", "PROPORTIONAL",  "proportional", "p", "P"}},
    {"PI" , {"integral_proportional_controller", "pi_controller",
              "PI_controller", "PROPORTIONAL_INTEGRAL",  "proportional_integral", "PI", "P"}},
    {"SS" , {"state-space"}}};
  double sample_period = 0.001;
  std::string type = "state-space";

  what="";
  std::string msg;
  bool ok = rosparam_utilities::getParam(nh,name+"/type",type, msg, &type);
  what = to_string(what, ok,msg);
  if(!ok)
  {
    return -1;
  }

  int n = out.xDim();
  if(N==-1)
  {
    nh.param(name+"/order", n, 1);
  }

  auto pt = std::find_if(_types["P" ].begin(),_types["P"]. end(), [type](const auto& v){return v == type;});
  auto it = std::find_if(_types["PI"].begin(),_types["PI"].end(), [type](const auto& v){return v == type;});
  auto st = std::find_if(_types["SS"].begin(),_types["SS"].end(), [type](const auto& v){return v == type;});
  if((pt!=_types["P"].end())||(it!=_types["PI"].end()))
  {
    typename Controller<N,MN>::MatrixN Kp, Ki;
    double kp=0;

    eu::resize(Kp,n,n);
    eu::resize(Ki,n,n);
    eu::setZero(Kp);
    eu::setZero(Ki);

    if(!rosparam_utilities::getParam(nh, name+"/proportional_gain", kp, what))
    {
      return -1;
    }
    eu::setDiagonal(Kp,kp);
    if(it!=_types["PI"].end())
    {
      double ki=0;
      if(!rosparam_utilities::getParam(nh, name+"/integral_gain", ki, what))
      {
        return -1;
      }

      if(!rosparam_utilities::getParam(nh, name+"/sample_period", sample_period, what))
      {
        return -1;
      }

      eu::setDiagonal(Ki,ki);
    }

    return out.setPI(Kp,Ki,sample_period, what);
  }
  else if(st!=_types["SS"].end())
  {
    DiscreteStateSpace<N,N,N,MN,MN,MN>& _tmp = dynamic_cast< DiscreteStateSpace<N,N,N,MN,MN,MN>& >(out);

    ret = setMatricesFromParam(_tmp,nh,name,msg);
    what = to_string(what, ret>0, msg);
    if(!ret)
    {
      return -1;
    }

    typename Controller<N,MN>::MatrixN aw_gain;  //antiwindup_gain

    eu::resize(aw_gain,n,n);
    eu::setZero(aw_gain);

    std::vector<double> aw_states(out.xDim(),0); //antiwindup_gain
    ok = rosparam_utilities::getParam(nh, name+"/antiwindup_gain", aw_gain, msg, &aw_gain);
    what = to_string(what, ok,msg);
    if(!ok)
    {
      return -1;
    }

    ok = rosparam_utilities::getParam(nh, name+"/antiwindup_states", aw_states, msg, &aw_states);
    what = to_string(what, ok,msg);
    if(!ok)
    {
      return -1;
    }

    ok = int(aw_states.size())!=out.xDim();
    what = to_string(what, ok, " antiwindup_states size is wrong '" + name + ".");
    if(!ok)
    {
      return -1;
    }

    typename Controller<N,MN>::MatrixN Baw;
    Baw = out.B() * aw_gain;

    for(int iord=0;iord<out.xDim();iord++)
    {
      if (!aw_states.at(iord))
      {
        for(int j=0;j<eu::cols(Baw);j++)
        {
          eu::at(Baw, iord, j) = 0.0;
        }
      }
    }
    ret = out.setAntiWindupMatrix(Baw, msg);
    what = to_string(what, ret>0,msg);
    return ret;
  }
  else if(type == "none")
  {
    ControllerStateSpaceArgs<N,MN> args;
    eu::resize(args.A  , 1, 1);
    eu::resize(args.B  , 1, 1);
    eu::resize(args.C  , 1, 1);
    eu::resize(args.D  , 1, 1);
    eu::resize(args.Baw, 1, 1);

    eu::setZero(args.A  );
    eu::setZero(args.B  );
    eu::setZero(args.C  );
    eu::setZero(args.D  );
    eu::setZero(args.Baw);
    ret = out.setMatrices(args, msg);
    what = to_string(what, ret>0,msg);
    return ret;
  }

  what = "Type '" + type + "' not recognized. The implemented controller types are: ";
  for(auto const & t : _types)
  {
    what +="\n\t- ";
    for(auto const & v : t.second)
      what +="'" + v + "',";
    what +=" (these are equivalent options!)";
  }
  return -1;
}

}

#endif // ROS_PARAMS_H
