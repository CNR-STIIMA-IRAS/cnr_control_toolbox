#include <frequency_identification/frequency_identification.h>
#include <sensor_msgs/JointState.h>
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "multisine_test");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;


  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::Publisher js_pub=pnh.advertise<sensor_msgs::JointState>("js",1);

  identification::MultiSineEstimator mse(pnh);

  double dt=0.001;
  ros::Rate lp(1.0/dt);
  double t=0.0;

  sensor_msgs::JointState js_msg;
  js_msg.position.resize(1);
  js_msg.velocity.resize(1);
  js_msg.effort.resize(1);
  js_msg.name.resize(1);
  mse.initTest(dt);
  while (ros::ok())
  {
    identification::state state= mse.execute(dt,
                                             0.5*js_msg.position.at(0),
                                             js_msg.position.at(0),
                                             js_msg.velocity.at(0),
                                             js_msg.effort.at(0)
                                             );
    js_msg.header.stamp=ros::Time::now();
    js_pub.publish(js_msg);
    t+=dt;
    if (state==identification::state::Idle)
    {
      mse.initTest(dt);
      t=0.0;
    }
    lp.sleep();
  }
}
