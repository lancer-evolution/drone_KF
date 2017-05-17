#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Quaternion.h>
#include "sensor_msgs/Imu.h"
#include "drone_KF/kalmanfilter.h"
#include "kalman_ang.h"
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
  ros::Publisher pub_est;
  ros::Publisher pub_mst;

  void imuCb(const sensor_msgs::Imu::ConstPtr& msg);
  void getAng();
  bool Ang_init;
  float deg_pre;
  float deg_abs;
  float dt;

  kalmanfilter kf_pose;
  kalmanfilter kf_ang;
  kf_angle kf_r;
  kf_angle kf_p;
  kf_angle kf_y;

  MatrixXd A = MatrixXd::Zero(3,3);// システム
  MatrixXd b = MatrixXd::Zero(3,1);
  MatrixXd bu = MatrixXd::Zero(3,1);
  MatrixXd c = MatrixXd::Zero(3,3);
  MatrixXd P = MatrixXd::Identity(3,3);
  int gamma;
  float Q,R;
  float u; //入力
  
  // 状態空間モデルも用いた時系列データの生成
  MatrixXd xhat = MatrixXd::Zero(3,1);
  MatrixXd y = MatrixXd::Zero(3,1);

  float v; // システム雑音
  MatrixXd w = MatrixXd::Zero(1,1); // 観測雑音

  //Vector3f rpy; //roll,pitch,yaw
  double roll, pitch, yaw;
  Vector3d rpy_get;//rectified data
  Vector3d rpy_raw;//imu raw data
  Vector3d rpy_pre;//imu pre data
  Vector3d rpy_est;//imu pre data
  Vector3d rpy_init;
  void conv_rpy(double& x, double& y, double& z);

  //param
  double accel_bias_x,accel_bias_y,accel_bias_z;
  double ang_bias_z;
  IMU();
	
};

IMU::IMU():
  deg_pre(360),
  dt(0.02),// sec
  gamma(2),
  u(0),
  Q(0.1),
  R(1),
  nh_("~")
{
  Ang_init = 1;
  rpy_get = Vector3d::Zero();
  
  A << 1, 0, 0,
	0, 1, 0, 
	0, 0, 1;

  b << 1,1,1;
  c << 1, 0, 0,
     0, 1, 0,
     0, 0, 1;
  P = gamma * P;

  pub_est = nh_.advertise<geometry_msgs::Vector3>("est",50);
  pub_mst = nh_.advertise<geometry_msgs::Vector3>("mst",50);
  nh_.param<double>("accel_bias_x", accel_bias_x, 0.25);//0.66
  nh_.param<double>("accel_bias_y", accel_bias_y, 0.37);
  nh_.param<double>("accel_bias_z", accel_bias_z, 0.12);
  nh_.param<double>("ang_bias_z", ang_bias_z, 0.0033);//0.0013
}

void IMU::imuCb(const sensor_msgs::Imu::ConstPtr& msg)
{
  geometry_msgs::Quaternion ori(msg->orientation);
  geometry_msgs::Vector3 ang_vel(msg->angular_velocity);
  geometry_msgs::Vector3 li_ac(msg->linear_acceleration);
  // 線形加速度のバイアス補正
  li_ac.x = li_ac.x - accel_bias_x;
  li_ac.y = li_ac.y - accel_bias_y;
  li_ac.z = li_ac.z - accel_bias_z;
  //tf::Vector3 li_ac_tf = tf::Vector3(li_ac.x,li_ac.y,li_ac.z);
  //Quaternionf q1(ori.w,ori.x,ori.y,ori.z);
  //rpy = q1.toRotationMatrix().eulerAngles(0, 1, 2); //roll,pitch,yaw(rad)
  //deg_abs = Ang(rpy[2]*180/(M_PI));
  float sq;
  cout << sqrt(li_ac.x*li_ac.x+li_ac.y*li_ac.y+li_ac.z*li_ac.z) << endl;

  if(Ang_init == 1){
	rpy_pre = rpy_raw;
	rpy_init = rpy_raw;
	Ang_init = 0;
  }else{
	// エラーチェック
	if( sqrt(rpy_raw(0)) < 1e-9f ){
	  rpy_raw = rpy_pre;
	}
	
	MatrixXd vec = MatrixXd::Zero(3,1);
	vec << li_ac.x,li_ac.y,li_ac.z;
  
	/*kalman filter*/
	y << vec(0,0),vec(1,0),vec(2,0);
	//kf.kf(A,b,bu,c,Q,R,0,y,xhat,P);
	kf_pose.kf(A,b,bu,c.transpose(),Q,R,u,y,xhat,P);
  
	cout << "x  :" << xhat(0,0) <<","<< xhat(1,0) <<","<< xhat(2,0) << endl
		 << "y  :" << y(0,0) << "," << y(1,0) << "," << y(2,0) << endl;

	geometry_msgs::Vector3 est_msg;
	geometry_msgs::Vector3 mst_msg;
	est_msg.x = xhat(0,0);
	est_msg.y = xhat(1,0);
	est_msg.z = xhat(2,0);
	mst_msg.x = y(0,0);
	mst_msg.y = y(1,0);
	mst_msg.z = y(2,0);
    pub_est.publish(est_msg);
	pub_mst.publish(mst_msg);
  }
}

  void IMU::getAng(){
	rpy_get(0) += (rpy_raw(0) - rpy_pre(0));
	rpy_get(1) += (rpy_raw(1) - rpy_pre(1));
	
	if(((rpy_pre(2) >= -180) && (rpy_pre(2) < -90)) && ((rpy_raw(2) <= 180) && (rpy_raw(2) > 90))){// 反対
	  rpy_get(2) += (- (180 + rpy_pre(2)) + (rpy_raw(2) - 180));
	  //cout << "aaaa" << rpy_pre(2) << " " << rpy_raw(2) << endl;
	}else if (((rpy_pre(2) <= 180) && (rpy_pre(2) > 90)) && ((rpy_raw(2) >= -180) && (rpy_raw(2) < -90))){
	  rpy_get(2) += ((180 - rpy_pre(2)) + (180 + rpy_raw(2)));
	  //cout << "bbbb"  << rpy_pre(2) << " " << rpy_raw(2) << endl;
	}else{
	  rpy_get(2) += (rpy_raw(2) - rpy_pre(2));
	  //cout << "c" << endl;
	}
	rpy_get(2) -= ang_bias_z;
	
	// 値の保持
	rpy_pre = rpy_raw;
  }

  void IMU::conv_rpy(double& x, double& y, double& z){
	MatrixXd conv = MatrixXd::Zero(3,3);


  }

  int main(int argc, char** argv)
  {

	ros::init(argc, argv,"kalman_acc_test");
	IMU imu;
	ros::NodeHandle nh;
	imu.imu_sub = nh.subscribe("ardrone/imu", 10, &IMU::imuCb, &imu);
	ROS_INFO_STREAM("kalmanfilter started!");
	ros::spin();

	return 0;
  }
