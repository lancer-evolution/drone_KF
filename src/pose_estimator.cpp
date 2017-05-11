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
  ros::Publisher pub_angle;
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
  deg_pre(360),
  dt(0.01),// sec
  gamma(1),
  u(0),
  Q(5),
  R(5),
  nh_("~")
{
  
  A << 1, 0, 0, dt, 0, 0, 0.5*dt*dt, 0, 0,
	0, 1, 0, 0, dt, 0, 0, 0.5*dt*dt, 0,
	0, 0, 1, 0, 0, dt, 0, 0, 0.5*dt*dt,
	0, 0, 0, 1, 0, 0, dt, 0, 0,
	0, 0, 0, 0, 1, 0, 0, dt, 0,
	0, 0, 0, 0, 0, 1, 0, 0, dt,
	0, 0, 0, 0, 0, 0, 1, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 1, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 1;
  
  // bu << dt,
  // 	0;
  b << 1,1,1,1,1,1,1,1,1;
  c << 0, 0, 0, 0, 0, 0, 1, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 1, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 1;
  P = gamma * P;

  pub_angle = nh_.advertise<geometry_msgs::Vector3>("angle_deg",50);
  pub_measurment = nh_.advertise<geometry_msgs::Vector3>("measure",50);
  pub_estimation = nh_.advertise<geometry_msgs::Vector3>("estimate",50);
}

void IMU::imuCb(const sensor_msgs::Imu::ConstPtr& msg)
{
  geometry_msgs::Quaternion ori(msg->orientation);
  geometry_msgs::Vector3 li_ac(msg->linear_acceleration);
  tf::Vector3 li_ac_tf = tf::Vector3(li_ac.x,li_ac.y,li_ac.z);
  //Quaternionf q1(ori.w,ori.x,ori.y,ori.z);
  //rpy = q1.toRotationMatrix().eulerAngles(0, 1, 2); //roll,pitch,yaw(rad)
  //deg_abs = Ang(rpy[2]*180/(M_PI));
  
  
  tf::Quaternion q;
  tf::quaternionMsgToTF(ori, q);
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
  //cout << rpy[2]*180/(M_PI) << " , " << yaw*180/(M_PI) << " , "  << endl;

  if( std::isnan(roll) || std::isnan(pitch) || std::isnan(yaw)){
	roll = 0;
	pitch = 0;
	yaw = 0;
  }
  roll = -roll;
  pitch = -pitch;
  yaw = -yaw;
  
  // 座標変換
  //cout <<"roll:" << roll << " "<< pitch << " "<< yaw << endl;
  //cout << "acceleration:" << li_ac.x << " " << li_ac.y << " " << endl;
  
  MatrixXd vec = MatrixXd::Zero(3,1);
  MatrixXd rotation = MatrixXd::Zero(3,3);
  vec << li_ac.x,li_ac.y,li_ac.z-9.807;
  rotation << cos(roll)*cos(pitch),cos(roll)*sin(pitch)*sin(yaw)-sin(roll)*cos(yaw),cos(roll)*sin(pitch)*cos(yaw)+sin(roll)*sin(yaw),
	sin(yaw)*cos(pitch),sin(roll)*sin(pitch)*sin(yaw)+cos(roll)*cos(yaw),sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw),
	-sin(roll),cos(pitch)*sin(yaw),cos(pitch)*cos(yaw);

  vec = rotation*vec; // 世界座標へ
  // geometry_msgs::Vector3 world_ac;
  // world_ac.x = vec(0,0);
  // world_ac.y = vec(1,0);
  // world_ac.z = vec(2,0);
  //cout << "trans:" << world_ac << endl;
  
  /*kalman filter*/
  y << vec(0,0),vec(1,0),vec(2,0);
  //kf.kf(A,b,bu,c,Q,R,0,y,xhat,P);
  kf.kf(A,b,bu,c.transpose(),Q,R,u,y,xhat,P);
  
  cout << "y:" << y << endl
	   << "xhat:" << xhat(6,0) <<","<< xhat(7,0) <<","<< xhat(8,0) << endl;

  geometry_msgs::Vector3 mst_msg;
  geometry_msgs::Vector3 est_msg;
  geometry_msgs::Vector3 ang_msg;
  mst_msg.x = y(0,0);mst_msg.y = y(1,0);mst_msg.z = y(2,0);
  est_msg.x = xhat(6,0);
  est_msg.y = xhat(7,0);
  est_msg.z = xhat(8,0);
  ang_msg.x = roll*180/(M_PI);
  ang_msg.y = pitch*180/(M_PI);
  ang_msg.z = yaw*180/(M_PI);
  pub_measurment.publish(mst_msg);
  pub_estimation.publish(est_msg);
  pub_angle.publish(ang_msg);
}

float IMU::Ang(float deg){

  if(deg > 90){//0 -> 180
	return deg - 180;
  }else if(deg <= 90){//180 -> 0
	return deg;
  }else
  return 0;
}

void IMU::conv_rpy(double& x, double& y, double& z){
  MatrixXd conv = MatrixXd::Zero(3,3);


}

int main(int argc, char** argv)
{

  ros::init(argc, argv,"kalman_pose");
  IMU imu;
  ros::NodeHandle nh;
  imu.imu_sub = nh.subscribe("ardrone/imu", 10, &IMU::imuCb, &imu);
  ROS_INFO_STREAM("kalmanfilter started!");
  ros::spin();

  return 0;
}