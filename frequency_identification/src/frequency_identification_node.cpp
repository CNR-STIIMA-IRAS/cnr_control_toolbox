#include <frequency_identification/frequency_identification.h>
#include <sensor_msgs/JointState.h>

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

  std::string joint_name;
  if (!pnh.getParam("joint_name",joint_name))
  {
    ROS_ERROR("no joint_name specified");
    joint_name="";
  }

  identification::MultiSineEstimator mse(pnh);

  double dt=0.001;
  ros::Rate lp(1.0/dt);
  double t=0.0;

  sensor_msgs::JointState jt_msg;
  jt_msg.position.resize(1);
  jt_msg.velocity.resize(1);
  jt_msg.effort.resize(1);
  jt_msg.name.resize(1);
  jt_msg.name.at(0)=joint_name;
  mse.initTest(dt);

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
  while (ros::ok())
  {
    double x,dx,ddx;
    identification::state state= mse.execute(dt,
                                             fb.position.at(idx+1),
                                             x,
                                             dx,
                                             ddx
                                             );

    jt_msg.position.at(0)=init_pos;
    jt_msg.velocity.at(0)=0.0;
    jt_msg.effort.at(0)=x;

    jt_msg.header.stamp=ros::Time::now();
    jt_pub.publish(jt_msg);
    t+=dt;
    if (state==identification::state::Idle)
    {
      mse.initTest(dt);
      t=0.0;
    }
    lp.sleep();
  }
}
