#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Quaternion.h>
#include "sensor_msgs/Imu.h"
#include "drone_KF/kalmanfilter.h"
#include <Eigen/Geometry>
#include <math.h>

#include <iostream>

using namespace std;
using namespace Eigen;


class IMU{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ros::NodeHandle pnh_;
  ros::Subscriber imu_sub;
  ros::Publisher pub_measurment;
  ros::Publisher pub_estimation;
  void imuCb(const sensor_msgs::Imu::ConstPtr& msg);
  float Ang(float deg);
  float deg_pre;
  float deg_abs;
  float dt;

  kalmanfilter kf;

  MatrixXd A = MatrixXd::Zero(2,2);// システム
  MatrixXd b = MatrixXd::Zero(2,1);
  MatrixXd bu = MatrixXd::Zero(2,1);
  MatrixXd c = MatrixXd::Zero(2,1);
  MatrixXd P = MatrixXd::Identity(2,2);
  int gamma;
  float Q,R;
  float u; //入力
  
  // 状態空間モデルも用いた時系列データの生成
  MatrixXd xhat = MatrixXd::Zero(2,1);
  MatrixXd y = MatrixXd::Zero(1,1);

  float v; // システム雑音
  MatrixXd w = MatrixXd::Zero(1,1); // 観測雑音
  
  IMU();
	
};

IMU::IMU():
  deg_pre(360),
  dt(0.01),// sec
  gamma(1),
  Q(1),
  R(5),
  pnh_("~")
{
  
  A << 1, -dt,
	0, 1;
  
  bu << dt,
	0;
  b << 1,
	1;
  c << 1,
	0;
  P = gamma * P;

  pub_measurment = pnh_.advertise<geometry_msgs::Vector3>("measure",100);
  pub_estimation = pnh_.advertise<geometry_msgs::Vector3>("estimate",100);
}

void IMU::imuCb(const sensor_msgs::Imu::ConstPtr& msg)
{
  geometry_msgs::Quaternion ori(msg->orientation);
  geometry_msgs::Vector3 ang_vel(msg->angular_velocity);
  geometry_msgs::Vector3 lin_acl(msg->linear_acceleration);

  Quaternionf q1(ori.w,ori.x,ori.y,ori.z);
  float roll = q1.toRotationMatrix().eulerAngles(0, 1, 2)[0]; //roll,pitch,roll
  float roll_vel = ang_vel.x*180/M_PI;
  roll = roll*180/M_PI;
  //ROS_INFO("I heard roll: [%f]", roll);
  
  deg_abs = Ang(roll);
  //cout << deg_abs << endl;


  /*kalman filter*/
  y << deg_abs;
  //kf.kf(A,b,bu,c,Q,R,0,y,xhat,P);
  kf.kf(A,b,bu,c,Q,R,roll_vel,y,xhat,P);
  
  //cout << y << " " << xhat(0,0) << " " << xhat(1,0) << endl;

  geometry_msgs::Vector3 mst_msg;
  geometry_msgs::Vector3 est_msg;
  mst_msg.x = y(0,0);
  est_msg.x = xhat(0,0);
  pub_measurment.publish(mst_msg);
  pub_estimation.publish(est_msg);
  
}

float IMU::Ang(float deg){

  if(deg > 90){//0 -> 180
	return deg - 180;
  }else if(deg <= 90){//180 -> 0
	return deg;
  }else
  return 0;
}

int main(int argc, char** argv)
{

  ros::init(argc, argv,"imu_kalman");
  IMU imu;
  ros::NodeHandle nh;
  imu.imu_sub = nh.subscribe("ardrone/imu", 10, &IMU::imuCb, &imu);
  ROS_INFO_STREAM("kalmanfilter started!");
  ros::spin();

  return 0;
}
