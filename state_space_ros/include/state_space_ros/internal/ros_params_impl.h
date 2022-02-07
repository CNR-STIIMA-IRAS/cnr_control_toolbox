#pragma once

#ifndef STATE_SPACE_ROS__ROS_PARAMS_IMPL_H
#define STATE_SPACE_ROS__ROS_PARAMS_IMPL_H

#include <rosparam_utilities/rosparam_utilities.h>
#include <eigen_matrix_utils/overloads.h>
#include <state_space_ros/ros_params.h>

namespace ru = rosparam_utilities;
namespace eu = eigen_utils;

//! @brief The implementation of all the classes and functions for the management of a Discreate Space System
namespace eigen_control_toolbox
{

//!
inline std::string append_string(const std::string& what, const bool& ret, const std::string& msg)
{
  return what + (what.size()>0?"\n":"") +
    ( ret ? (msg.size()==0? "" : "" + msg) : "[!]" + msg);
}

//!
inline std::string dim(const Eigen::MatrixXd& m)
{
  return "(" + std::to_string(eu::rows(m)) +"x"+std::to_string(eu::cols(m)) + ")";
}

//!
inline bool importMatricesFromParam(const ros::NodeHandle& nh,
                                    const std::string& name,
                                    Eigen::MatrixXd &A, Eigen::MatrixXd &B, Eigen::MatrixXd &C, Eigen::MatrixXd &D,
                                    std::string& what)
{
  what="";
  std::string msg;
  std::string ns = (name.find("/") == 0) ? name : nh.getNamespace() + "/" + name;
  std::string type = "state-space";

  bool ok = ru::get(ns+"/type",type, msg, &type);
  what += msg.size()==0? "" : (what.size()>0? "\n" :"") + std::string(ok? "[?]" : "[!]" ) + msg;
  if(!ok)
  {
    return false;
  }

  if (!type.compare("unity"))
  {
    int dim;
    if(!ru::get(ns+"/order", dim, msg))
    {
      if(!ru::get(ns+"/dof", dim, msg))
      {
        what += (what.size()>0? "\n[!]" :"[!]") + msg;
        return false;
      }
    }

    A.resize(dim,dim);
    B.resize(dim,dim);
    C.resize(dim,dim);
    D.resize(dim,dim);
    A.setZero();
    B.setZero();
    C.setZero();
    D.setIdentity();
    return true;
  }

  if(!ru::get(ns + "/A", A, what))
  {
    return false;
  }
  if(!ru::get(ns+"/B", B, what))
  {
    return false;
  }
  if(!ru::get(ns+"/C", C, what))
  {
    return false;
  }
  if(!ru::get(ns+"/D", D, what))
  {
    return false;
  }
  if(A.rows()!=B.rows())
  {
    what = "The "+ nh.getNamespace()+"/"+ name+"/A has a numbers of rows that is different from B";
    return false;
  }
  if(A.cols()!=C.cols())
  {
    what = "The "+ nh.getNamespace()+"/"+ name+"/A has a numbers of cols that is different from C";
    return false;
  }
  if(C.rows()!=D.rows())
  {
    what = "The "+ nh.getNamespace()+"/"+ name+"/C has a numbers of cols that is different from D";
    return false;
  }
  if(D.cols()!=B.cols())
  {
    what = "The "+ nh.getNamespace()+"/"+ name+"/B has a numbers of cols that is different from D";
    return false;
  }

  return true;
}

//!
template<int S, int I, int O, int MS, int MI, int MO>
inline bool getDiscreteStateSpaceArgs(DiscreteStateSpaceArgs<S,I,O,MS,MI,MO>& args,
                                      const ros::NodeHandle& nh,
                                      const std::string& name,
                                      std::string& what)
{
  bool ret = true;
  try
  {

    what="";
    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
    Eigen::MatrixXd C;
    Eigen::MatrixXd D;

    std::string msg;
    ret = importMatricesFromParam(nh, name, A,B,C,D, msg);
    what = append_string(what, ret,msg);
    if(!ret)
    {
       return false;
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
      ret &= c.first;
    }
  }
  catch(std::exception& e)
  {
    what = append_string(what, false, "Caught an exception: " + std::string(e.what()));
    return false;
  }
  return ret;
}

//!
template<int S, int I, int O, int MS, int MI, int MO>
inline bool setMatricesFromParam(DiscreteStateSpace<S,I,O,MS,MI,MO>& out,
                                const ros::NodeHandle& nh,
                                const std::string& name,
                                std::string& what)
{
  bool ret = true;
  try
  {
    DiscreteStateSpaceArgs<S,I,O,MS,MI,MO> args;
    if(!getDiscreteStateSpaceArgs(args, nh, name, what))
    {
      return false;
    }

    std::string msg;
    ret = out.setMatrices(args,msg);
    what = append_string(what, ret, msg);
  }
  catch(std::exception& e)
  {
    what = append_string(what, ret, "Caught an exception: " +std::string(e.what()));
    return false;
  }
  return ret;
}

//!
template<int K,int N,int MK,int MN>
inline bool setMatricesFromParam(IntegralStateSpace<K,N,MK,MN>& out,
                                 const ros::NodeHandle& nh,
                                 const std::string& name,
                                 std::string& what)
{
  what="";

  std::string type = "integral-state-space";
  if(!ru::getParam(nh,name+"/type",type, what, &type))
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
  if(!ru::getParam(nh,name+"/order",args.order, what, &args.order))
  {
    return false;
  }

  args.degrees_of_freedom = 1;
  if(!ru::getParam(nh,name+"/dof",args.degrees_of_freedom, what, &args.degrees_of_freedom))
  {
    return false;
  }

  return out.setMatrices(args,what);
}

//!
template<int K,int N,int MK,int MN>
inline bool setMatricesFromParam(IntegralDiscreteStateSpace<K,N,MK,MN>& out,
                                const ros::NodeHandle& nh,
                                const std::string& name,
                                std::string& what)
{
  bool ret = true;
  try
  {
    what="";
    std::string type = "integral-discrete-state-space";
    if(!ru::getParam(nh,name+"/type",type, what, &type))
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
    if(!ru::getParam(nh,name+"/order",args.order, what, &args.order))
    {
      return false;
    }

    args.degrees_of_freedom = 1;
    if(!ru::getParam(nh,name+"/dof",args.degrees_of_freedom, what, &args.degrees_of_freedom))
    {
      return false;
    }

    args.dt = 0;
    if(!ru::getParam(nh,name+"/dt",args.dt, what, &args.dt))
    {
      return false;
    }

    ret =out.setMatrices(args,what);
  }
  catch(std::exception& e)
  {
    std::cerr << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": Caught an exception: " << e.what() << std::endl;
    return false;
  }
  return ret;
}

//!
template<int N, int MN>
inline bool setMatricesFromParam(Controller<N,MN>& out,
    const ros::NodeHandle& nh,const std::string& name,std::string& what)
{
  bool ret = true;
  try
  {
    std::map<std::string,std::vector<std::string>> _types =
    { {"P"  , {"proportional_controller", "p_controller", "P_controller", "PROPORTIONAL",  "proportional", "p", "P"}},
      {"PI" , {"integral_proportional_controller", "pi_controller",
                "PI_controller", "PROPORTIONAL_INTEGRAL",  "proportional_integral", "PI", "P"}},
      {"SS" , {"state-space"}}};
    double sample_period = 0.001;
    std::string type = "state-space";

    what="Param '"+name+"'";
    std::string msg;
    ret = ru::getParam(nh,name+"/type",type, msg, &type);
    what = append_string(what, ret,msg);
    if(!ret)
    {
      return false;
    }

    auto pt = std::find_if(_types["P" ].begin(),_types["P"]. end(), [type](const auto& v){return v == type;});
    auto it = std::find_if(_types["PI"].begin(),_types["PI"].end(), [type](const auto& v){return v == type;});
    auto st = std::find_if(_types["SS"].begin(),_types["SS"].end(), [type](const auto& v){return v == type;});

    //==
    if((pt!=_types["P"].end())||(it!=_types["PI"].end()))
    //==
    {
      typename Controller<N,MN>::MatrixN Kp, Ki;
      int n_from_param = -1;
      if(!ru::getParam(nh, name+"/order", n_from_param, what))
      {
        if(!ru::getParam(nh, name+"/dof", n_from_param, what, &n_from_param))
        {
          return false;
        }
      }
      int n_from_template = N;
      int n_from_class = out.xDim();

      int n = n_from_param;
      if(n_from_template!=-1)
      {
        if(n_from_template!=n_from_class)
        {
          what = "Error in class construction. The class is a static-allocated templated with dimension "
                  + std::to_string(n_from_class)+", while the function template is with dimension "
                    + std::to_string(n_from_template);
          return false;
        }
        if((n_from_template!=n_from_param)&&(n_from_param!=-1)) // th default value
        {
          what = "Error in class construction. The class is a static-allocated templated with dimension "
                  + std::to_string(n_from_class) + ", while the param specifies a dimension of "
                    + std::to_string(n_from_param);
          return false;
        }
        n = n_from_template; // class properites overridden
      }
      else // from param
      {
        if(n_from_param!=-1)
        {
          n = n_from_param;
        }
        else if(n_from_class >0)
        {
          n = n_from_param;
        }
        else
        {
          what = "The system dimension is superimposed to 1, since none parameter is present in the rosparam server, "
                    "and the controller has never been initialized before.";
          n = 1;
        }
      }

      eu::resize(Kp,n,n);
      eu::resize(Ki,n,n);
      eu::setZero(Kp);
      eu::setZero(Ki);

      std::vector<double> kp, ki;
      if(!ru::getParam(nh, name+"/proportional_gain", kp, what))
      {
        double kp_=0.0;
        if(!ru::getParam(nh, name+"/proportional_gain", kp_, what))
        {
          return false;
        }
        kp.resize(n,kp_);
      }
      if(static_cast<int>(kp.size())!=n)
      {
        if(static_cast<int>(kp.size())==1)
        {
          double kp_ = kp.front();
          kp.resize(n,kp_);
        }
        else
        {
          what += "The param dimension of the 'proportional_gain' " + std::to_string(kp.size())
                    + " mismatches with the foreseen dimension" + std::to_string(n);
          return false;
        }
      }
      eu::setDiagonal(Kp,kp);

      if(it!=_types["PI"].end())
      {
        if(!ru::getParam(nh, name+"/integral_gain", ki, what))
        {
          double ki_=0;
          if(!ru::getParam(nh, name+"/integral_gain", ki_, what))
          {
            return false;
          }
          ki.resize(n,ki_);
        }
        if(!ru::getParam(nh, name+"/sample_period", sample_period, what))
        {
          return false;
        }
        if(static_cast<int>(ki.size())!=n)
        {
          if(static_cast<int>(ki.size())==1)
          {
            double ki_ = ki.front();
            ki.resize(n,ki_);
          }
          else
          {
            what += "The param dimension of the 'integral_gain' " + std::to_string(ki.size())
                      + " mismatches with the foreseen dimension" + std::to_string(n);
            return false;
          }
        }
        eu::setDiagonal(Ki,ki);
      }

      ret = out.setPI(Kp,Ki,sample_period, what);
    }
    //==
    else if(st!=_types["SS"].end())
    //==
    {
      ControllerStateSpaceArgs<N,MN> args;
      std::string msg;
      ret = getDiscreteStateSpaceArgs(args, nh, name, msg);
      what = append_string(what, ret, msg);
      if(!ret)
      {
        return false;
      }

      typename Controller<N,MN>::MatrixN aw_gain;  //antiwindup_gain
      std::vector<double> aw_states(eu::rows(args.A), 0);

      std::vector<std::pair<bool, std::string>> checks =
      { {ru::getParam(nh, name+"/antiwindup_gain", aw_gain, msg, &aw_gain), msg},
        {ru::getParam(nh, name+"/antiwindup_states", aw_states, msg, &aw_states), msg} };

      for(auto const & c : checks)
      {
        ret &= c.first;
        what = append_string(what, c.first, c.second);
      }

      if(static_cast<int>(aw_states.size())!=eu::rows(args.A))
      {
        ret &= false;
        what = append_string(what, false,  "antiwindup_states size ("+std::to_string(static_cast<int>(aw_states.size()))+") "
                                        "is different from the order of the system " + dim(args.A));
      }

      if(!eu::resize(args.Baw, eu::rows(args.A),eu::rows(args.B)))
      {
        ret &= false;
        what = append_string(what, false,  "Error in resizing the Baw matrix " +dim(args.A));
      }

      if(!ret)
      {
        return false;
      }

      args.Baw = out.B() * aw_gain;
      for(int iord=0;iord<out.xDim();iord++)
      {
        if (!aw_states.at(iord))
        {
          for(int j=0;j<eu::cols(args.Baw);j++)
          {
            eu::at(args.Baw, iord, j) = 0.0;
          }
        }
      }

      ret = out.setMatrices(args,msg);
      what = append_string(what, ret, msg);
    }
    // ======
    else
    // ======
    {
      what = "Type '" + type + "' not recognized. The implemented controller types are: ";
      for(auto const & t : _types)
      {
        what +="\n\t- ";
        for(auto const & v : t.second)
          what +="'" + v + "',";
        what +=" (these are equivalent options!)";
      }
      ret = false;
    }
  }
  catch(std::exception& e)
  {
    what = append_string(what, ret, "Caught an exception: " + std::string(e.what()));
    return false;
  }
  return ret;
}

}

#endif // ROS_PARAMS_H
