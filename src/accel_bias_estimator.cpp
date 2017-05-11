#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Quaternion.h>
#include "sensor_msgs/Imu.h"
#include "drone_KF/kalmanfilter.h"
#include <Eigen/Geometry>
#include <math.h>
#include <tf/transform_broadcaster.h>

#include <iostream>

using namespace std;
using namespace Eigen;


class IMU{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ros::NodeHandle nh_;
  ros::Subscriber imu_sub;
  ros::Publisher pub_measurment;
  ros::Publisher pub_estimation;
  void imuCb(const sensor_msgs::Imu::ConstPtr& msg);
  float Ang(float deg);
  float deg_pre;
  float deg_abs;
  float dt;

  kalmanfilter kf;

  MatrixXd A = MatrixXd::Zero(9,9);// システム
  MatrixXd b = MatrixXd::Zero(9,1);
  MatrixXd bu = MatrixXd::Zero(9,1);
  MatrixXd c = MatrixXd::Zero(3,9);
  MatrixXd P = MatrixXd::Identity(9,9);
  int gamma;
  float Q,R;
  float u; //入力
  
  // 状態空間モデルも用いた時系列データの生成
  MatrixXd xhat = MatrixXd::Zero(9,1);
  MatrixXd y = MatrixXd::Zero(3,1);

  float v; // システム雑音
  MatrixXd w = MatrixXd::Zero(1,1); // 観測雑音

  //Vector3f rpy; //roll,pitch,yaw
  double roll, pitch, yaw;
  void conv_rpy(double& x, double& y, double& z);
  IMU();
	
};

IMU::IMU():
  dt(0.02),// sec
  gamma(1),
  u(1),
  Q(5),
  R(5),
  nh_("~")
{
  
  A << 1, 0, 0, dt, 0, 0, -dt*dt, 0, 0,
	0, 1, 0, 0, dt, 0, 0, -dt*dt, 0,
	0, 0, 1, 0, 0, dt, 0, 0, -dt*dt,
	0, 0, 0, 1, 0, 0, -dt, 0, 0,
	0, 0, 0, 0, 1, 0, 0, -dt, 0,
	0, 0, 0, 0, 0, 1, 0, 0, -dt,
	0, 0, 0, 0, 0, 0, 1, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 1, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 1;
  
  //bu << ,
  // 	0;
  b << 1,1,1,1,1,1,1,1,1;
  c << 1, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 1, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 1, 0, 0, 0, 0, 0, 0;
  P = gamma * P;

  pub_measurment = nh_.advertise<geometry_msgs::Vector3>("measure",50);
  pub_estimation = nh_.advertise<geometry_msgs::Vector3>("estimate",50);
}

void IMU::imuCb(const sensor_msgs::Imu::ConstPtr& msg)
{
  geometry_msgs::Quaternion ori(msg->orientation);
  geometry_msgs::Vector3 li_ac(msg->linear_acceleration);
  tf::Vector3 li_ac_tf = tf::Vector3(li_ac.x,li_ac.y,li_ac.z);

  bu << li_ac.x*dt*dt,li_ac.y*dt*dt,(li_ac.z-9.806)*dt*dt,
	li_ac.x*dt,li_ac.y*dt,(li_ac.z-9.806)*dt,
	0,0,0;


  /*kalman filter*/
  y << 0,0,0;
  //kf.kf(A,b,bu,c,Q,R,0,y,xhat,P);
  kf.kf(A,b,bu,c.transpose(),Q,R,u,y,xhat,P);
  
  cout << "bias:" << xhat(6,0) <<","<< xhat(7,0) <<","<< xhat(8,0) << endl;

  geometry_msgs::Vector3 mst_msg;
  geometry_msgs::Vector3 est_msg;
  mst_msg.x = y(0,0);mst_msg.y = y(1,0);mst_msg.z = y(2,0);
  est_msg.x = xhat(6,0);
  est_msg.y = xhat(7,0);
  est_msg.z = xhat(8,0);
  pub_measurment.publish(mst_msg);
  pub_estimation.publish(est_msg);
}

int main(int argc, char** argv)
{

  ros::init(argc, argv,"kalman_accel_bias");
  IMU imu;
  ros::NodeHandle nh;
  imu.imu_sub = nh.subscribe("ardrone/imu", 10, &IMU::imuCb, &imu);
  ROS_INFO_STREAM("Bias estimator started!");
  ros::spin();

  return 0;
}
