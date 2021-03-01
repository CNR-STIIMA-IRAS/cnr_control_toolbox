#pragma once

#ifndef STATE_SPACE_ROS__ROS_PARAMS_IMPL_H
#define STATE_SPACE_ROS__ROS_PARAMS_IMPL_H

#include <state_space_ros/ros_params.h>

//! @brief The implementation of all the classes and functions for the management of a Discreate Space System
namespace eigen_control_toolbox
{

inline bool importMatricesFromParam(const ros::NodeHandle&  nh,
                                    const std::string&      name,
                                    Eigen::MatrixXd&        A,
                                    Eigen::MatrixXd&        B,
                                    Eigen::MatrixXd&        C,
                                    Eigen::MatrixXd&        D,
                                    std::string&            what)
{
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
  Eigen::MatrixXd A;
  Eigen::MatrixXd B;
  Eigen::MatrixXd C;
  Eigen::MatrixXd D;

  if(!importMatricesFromParam(nh, name, A,B,D,D, what))
  {
     return false;
  }

  int num_s = eu::rows(A);
  if(num_s!=eu::rows(A))
  {
    what += "Matrix A is not a sqare-matrix";
    return false;
  }
  if(num_s!=eu::rows(B))
  {
    what += "The number of rows of Matrix B is different from the number of rows of Matrix A.";
    return false;
  }
  if(num_s!=eu::cols(C))
  {
    what += "The number of cols of Matrix C is different from the number of rows of Matrix A.";
    return false;
  }

  int num_i = eu::cols(B);
  if(num_i!=eu::cols(D))
  {
    what += "The number of cols of Matrix D is different from the number of cols of Matrix B.";
    return false;
  }

  int num_o = eu::rows(C);
  if(num_i!=eu::rows(D))
  {
    what += "The number of rows of Matrix D is different from the number of cols of Matrix C.";
    return false;
  }

  DiscreteStateSpaceArgs<S,I,O,MS,MI,MO> args;
  for(int i=0;i<num_s;i++)
  {
    for(int j=0;j<num_s;j++)
    {
      eu::at(args.A,i,j) = A(i,j);
    }
  }

  for(int i=0;i<num_s;i++)
  {
    for(int j=0;j<num_i;j++)
    {
      eu::at(args.B,i,j) = B(i,j);
    }
  }

  for(int i=0;i<num_o;i++)
  {
    for(int j=0;j<num_s;j++)
    {
      eu::at(args.C,i,j) = C(i,j);
    }
  }

  for(int i=0;i<num_o;i++)
  {
    for(int j=0;j<num_s;j++)
    {
      eu::at(args.D,i,j) = D(i,j);
    }
  }

  return out.setMatrices(args,what);
}


template< int K,int N,int MK,int MN>
inline int setMatricesFromParam(IntegralStateSpace<K,N,MK,MN>& out,
                                  const ros::NodeHandle& nh,
                                    const std::string& name,
                                      std::string& what)
{
  double order;
  double dof;

  std::string type = "integral-state-space";
  if(!rosparam_utilities::getParam(nh,name+"/type",type, what, &type))
  {
    return false;
  }

  if (!type.compare("integral-state-space"))
  {
    what += "Error, the type of the matrices is not 'integral-state-space'";
    return false;
  }

  IntegralStateSpaceArgs args;

  args.order = 1;
  if(!rosparam_utilities::getParam(nh,name+"/order",args.order, what, &args.order))
  {
    return false;
  }

  args.degrees_of_freedom = 1;
  if(!rosparam_utilities::getParam(nh,name+"/dof",args.degrees_of_freedom, what, &args.degrees_of_freedom))
  {
    return false;
  }

  return out.setMatrices(args,what);
}


template< int K,int N,int MK,int MN>
inline int setMatricesFromParam(IntegralDiscreteStateSpace<K,N,MK,MN>& out,
                                  const ros::NodeHandle& nh,
                                    const std::string& name,
                                      std::string& what)
{
  std::string type = "integral-discrete-state-space";
  if(!rosparam_utilities::getParam(nh,name+"/type",type, what, &type))
  {
    return false;
  }

  if (!type.compare("integral-discrete-state-space"))
  {
    what += "Error, the type of the matrices is not 'integral-discrete-state-space'";
    return false;
  }

  IntegralDiscreteStateSpaceArgs args;

  args.order = 1;
  if(!rosparam_utilities::getParam(nh,name+"/order",args.order, what, &args.order))
  {
    return false;
  }

  args.degrees_of_freedom = 1;
  if(!rosparam_utilities::getParam(nh,name+"/dof",args.degrees_of_freedom, what, &args.degrees_of_freedom))
  {
    return false;
  }

  args.dt = 0;
  if(!rosparam_utilities::getParam(nh,name+"/dt",args.dt, what, &args.dt))
  {
    return false;
  }

  return out.setMatrices(args,what);
}

template<int N, int MN>
inline int setMatricesFromParam(Controller<N,MN>& out,
    const ros::NodeHandle& nh,const std::string& name,std::string& what)
{
  std::map<std::string,std::vector<std::string>> _types =
  { {"P"  , {"proportional_controller", "p_controller", "P_controller", "PROPORTIONAL",  "proportional", "p", "P"}},
    {"PI" , {"integral_proportional_controller", "pi_controller",
              "PI_controller", "PROPORTIONAL_INTEGRAL",  "proportional_integral", "PI", "P"}},
    {"SS" , {"state-space"}}};
  double sample_period = 0.001;
  std::string type = "proportional_controller";

  if(!rosparam_utilities::getParam(nh,name+"/type",type, what, &type))
  {
    return false;
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

    if(!setMatricesFromParam(_tmp,nh,name,what))
    {
      return -1;
    }

    typename Controller<N,MN>::MatrixN aw_gain;  //antiwindup_gain

    eu::resize(aw_gain,n,n);
    eu::setZero(aw_gain);

    std::vector<double> aw_states(out.xDim(),0); //antiwindup_gain
    if(!rosparam_utilities::getParam(nh, name+"/antiwindup_gain", aw_gain, what, &aw_gain))
    {
      return -1;
    }
    if (!rosparam_utilities::getParam(nh, name+"/antiwindup_states", aw_states, what, &aw_states))
    {
      return -1;
    }
    if (int(aw_states.size())!=out.xDim())
    {
      what += " antiwindup_states size is wrong '" + name + ".";
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
    return out.setAntiWindupMatrix(Baw, what);
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
    return out.setMatrices(args, what);
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
