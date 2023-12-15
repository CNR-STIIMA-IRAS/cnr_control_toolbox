#include <sensor_msgs/JointState.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <ros/ros.h>

sensor_msgs::JointState fb;
bool received=false;

void cb(const sensor_msgs::JointStateConstPtr& msg)
{
  fb=*msg;
  received=true;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "multisine_test");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;


  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::Publisher  jt_pub=nh.advertise<sensor_msgs::JointState>("joint_target",1);
  ros::Subscriber js_sub=nh.subscribe<sensor_msgs::JointState>("joint_states",1,cb);

  ros::Publisher freq_pub=nh.advertise<std_msgs::Float64>("frequency",1);
  ros::Publisher freqstring_pub=nh.advertise<std_msgs::String>("frequency_string",1);
  ros::ServiceClient start_srv=nh.serviceClient<std_srvs::Empty>("/start_log");
  ros::ServiceClient stop_srv=nh.serviceClient<std_srvs::Empty>("/stop_log");

  std::string joint_name;
  if (!pnh.getParam("joint_name",joint_name))
  {
    ROS_ERROR("no joint_name specified");
    return 0;
  }

  double dt=0.001;
  ros::Rate lp(1.0/dt);
  double t=0.0;

  sensor_msgs::JointState jt_msg;
  jt_msg.position.resize(1);
  jt_msg.velocity.resize(1);
  jt_msg.effort.resize(1);
  jt_msg.name.resize(1);
  jt_msg.name.at(0)=joint_name;


  while (ros::ok()) {
    if (received)
      break;
    lp.sleep();
  }

  size_t idx=0;
  for (idx=0;idx<fb.name.size();idx++)
  {
    if (!fb.name.at(idx).compare(joint_name))
      break;
  }
  if (idx>=fb.name.size())
  {
    ROS_ERROR_STREAM(joint_name << " is not part of the joint state msg = "<<jt_msg);
    return 0;
  }

  double init_pos=fb.position.at(idx);
  ROS_INFO("inital position = %f",init_pos);

  double alpha=0;
  double omega0=500;
  double omega_end=0.1;
  double t_max=360;
  double ampl=40;

  if (!pnh.getParam("first_natural_frequency",omega0))
  {
    ROS_ERROR("no first_natural_frequency specified");
    omega0=500;
  }
  if (!pnh.getParam("last_natural_frequency",omega_end))
  {
    ROS_ERROR("no last_natural_frequency specified");
    omega_end=0.01;
  }
  if (!pnh.getParam("test_duration",t_max))
  {
    ROS_ERROR("no first_natural_frequency specified");
    t_max=360;
  }
  if (!pnh.getParam("amplitude",ampl))
  {
    ROS_ERROR("no amplitude specified");
    ampl=40;
  }

  ROS_INFO("test of %f seconds, amplitude=%f, frequency range=[%f,%f]",t_max,ampl,omega0,omega_end);
  //
  // x=[0,1]
  // omega=omega0* (omega_end/omega)^x
  // x=0 -> omega=omega0
  // x=1 -> omega=omega_end
  //

  std_srvs::Empty srv;
  start_srv.call(srv);
  ros::WallDuration(1).sleep();

  ros::Time t0=ros::Time::now();
  while (ros::ok())
  {
    double tnew=(ros::Time::now()-t0).toSec();
    double dt=tnew-t;
    t+=dt;
    double fraction=t/t_max;
    double omega=omega0*std::pow(omega_end/omega0,fraction);
    alpha+=omega*dt;
    double x=ampl*sin(alpha);

    jt_msg.position.at(0)=init_pos;
    jt_msg.velocity.at(0)=0.0;
    jt_msg.effort.at(0)=x;

    jt_msg.header.stamp=ros::Time::now();
    jt_pub.publish(jt_msg);

    std_msgs::String freq_string;
    freq_string.data="frequency: "+std::to_string( omega);
    freqstring_pub.publish(freq_string);

    std_msgs::Float64 freq_msg;
    freq_msg.data=omega;
    freq_pub.publish(freq_msg);
    if (t>t_max)
      break;
    lp.sleep();
  }

  jt_msg.position.at(0)=init_pos;
  jt_msg.velocity.at(0)=0.0;
  jt_msg.effort.at(0)=0.0;

  jt_msg.header.stamp=ros::Time::now();

  ros::WallDuration(1).sleep();
  stop_srv.call(srv);

  return 0;
}
