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
  ros::Publisher pub_pose;
  ros::Publisher pub_ang_raw;
  ros::Publisher pub_ang_mst;
  ros::Publisher pub_ang_est;
  ros::Publisher pub_ang_vel_mst;
  ros::Publisher pub_acc_lcl;
  ros::Publisher pub_acc_wrd;
  ros::Publisher pub_acc_wrd_kf;
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
  gamma(1),
  u(0),
  Q(0.1),
  R(10),
  nh_("~")
{
  Ang_init = 1;
  rpy_get = Vector3d::Zero();
  
  A << 1, 0, 0, dt, 0, 0, 0.5*dt*dt, 0, 0,
	0, 1, 0, 0, dt, 0, 0, 0.5*dt*dt, 0,
	0, 0, 1, 0, 0, dt, 0, 0, 0.5*dt*dt,
	0, 0, 0, 1, 0, 0, dt, 0, 0,
	0, 0, 0, 0, 1, 0, 0, dt, 0,
	0, 0, 0, 0, 0, 1, 0, 0, dt,
	0, 0, 0, 0, 0, 0, 1, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 1, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 1;

  b << 1,1,1,1,1,1,1,1,1;
  c << 0, 0, 0, 0, 0, 0, 1, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 1, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 1;
  P = gamma * P;

  
  pub_ang_raw = nh_.advertise<geometry_msgs::Vector3>("ang_raw",50);
  pub_ang_mst = nh_.advertise<geometry_msgs::Vector3>("ang_mst",50);
  pub_ang_est = nh_.advertise<geometry_msgs::Vector3>("ang_est",50);
  pub_ang_vel_mst = nh_.advertise<geometry_msgs::Vector3>("ang_vel_mst",50);
  pub_acc_lcl = nh_.advertise<geometry_msgs::Vector3>("lin_acc_lcl",50);
  pub_acc_wrd = nh_.advertise<geometry_msgs::Vector3>("lin_acc_wrd",50);
  pub_acc_wrd_kf = nh_.advertise<geometry_msgs::Vector3>("lin_acc_wrd_kf",50);
  pub_pose = nh_.advertise<geometry_msgs::Vector3>("pose_est",50);
  nh_.param<double>("accel_bias_x", accel_bias_x, 0.64);//0.66
  nh_.param<double>("accel_bias_y", accel_bias_y, 0.67);
  nh_.param<double>("accel_bias_z", accel_bias_z, 0.17);
  nh_.param<double>("ang_bias_z", ang_bias_z, 0.0009);//0.0013
}

void IMU::imuCb(const sensor_msgs::Imu::ConstPtr& msg)
{
  geometry_msgs::Quaternion ori(msg->orientation);
  geometry_msgs::Vector3 ang_vel(msg->angular_velocity);
  geometry_msgs::Vector3 li_ac(msg->linear_acceleration);
  //float sq;
  //cout << sqrt(li_ac.x*li_ac.x+li_ac.y*li_ac.y+li_ac.z*li_ac.z) << endl;
  // 角速度の単位変換(deg)
  ang_vel.x = ang_vel.x*180/(M_PI);
  ang_vel.y = ang_vel.y*180/(M_PI);
  ang_vel.z = ang_vel.z*180/(M_PI);
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
  
  tf::Quaternion q;
  tf::quaternionMsgToTF(ori, q);
  tf::Matrix3x3 m(q);
  m.getRPY(rpy_raw(0), rpy_raw(1),rpy_raw(2));
  rpy_raw = rpy_raw*180/(M_PI);
  

  if(Ang_init == 1){
	rpy_pre = rpy_raw;
	rpy_init = rpy_raw;
	Ang_init = 0;
  }else{
	// エラーチェック
	if( sqrt(rpy_raw(0)) < 1e-9f ){
	  rpy_raw = rpy_pre;
	}
	
	IMU::getAng(); // rpy_rawを積算角度に変換してrpy_getに保存(deg)

    // KFによる角度推定
	//rpy_est(0) = kf_r.kalman(rpy_get(0), ang_vel.x);
	//rpy_est(1) = kf_p.kalman(rpy_get(1), ang_vel.y);
	//rpy_est(2) = kf_y.kalman(rpy_get(2), ang_vel.z);
	rpy_est(0) = rpy_get(0);
	rpy_est(1) = rpy_get(1);
	rpy_est(2) = rpy_get(2); 
	
	// ラジアン変換
	roll = rpy_est(0)*M_PI/180;
	pitch = rpy_est(1)*M_PI/180;
	yaw = rpy_est(2)*M_PI/180;
	
	/*ここからーーーーーーーーー*/
	// 座標変換
	//cout <<"roll:" << roll << " "<< pitch << " "<< yaw << endl;
	//cout << "acceleration:" << li_ac.x << " " << li_ac.y << " " << endl;
	MatrixXd vec = MatrixXd::Zero(3,1);
	MatrixXd rotation = MatrixXd::Zero(3,3);
	vec << li_ac.x,li_ac.y,li_ac.z;
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
	if(abs(vec(0,0))<1)
	   vec(0,0)=0;
	if(abs(vec(1,0))<1)
	  vec(1,0)=0;
	if(abs(vec(2,0)-9.806)<1)
	  vec(2,0)=9.806;
	
	y << vec(0,0),vec(1,0),vec(2,0)-9.806;
	//kf.kf(A,b,bu,c,Q,R,0,y,xhat,P);
	kf_pose.kf(A,b,bu,c.transpose(),Q,R,u,y,xhat,P);
  
	cout << "ang:" << rpy_est(0) << "," << rpy_est(1) << "," << rpy_est(2) << endl
		 << "x  :" << xhat(0,0) <<","<< xhat(1,0) <<","<< xhat(2,0) << endl
		 << "x' :" << xhat(3,0) <<","<< xhat(4,0) <<","<< xhat(5,0) << endl
		 << "x'':" << xhat(6,0) <<","<< xhat(7,0) <<","<< xhat(8,0) << endl
		 << "y  :" << y(0,0) << "," << y(1,0) << "," << y(2,0) << endl;

	geometry_msgs::Vector3 ang_raw_msg;
	geometry_msgs::Vector3 ang_mst_msg;
	geometry_msgs::Vector3 ang_est_msg;
	geometry_msgs::Vector3 ang_vel_mst_msg;
	geometry_msgs::Vector3 pose_msg;
	geometry_msgs::Vector3 lin_acc_msg;
	geometry_msgs::Vector3 lin_acc_wrd_msg;
	geometry_msgs::Vector3 lin_acc_wrd_kf_msg;
	ang_raw_msg.x = rpy_raw(0);
	ang_raw_msg.y = rpy_raw(1);
	ang_raw_msg.z = rpy_raw(2);
	ang_mst_msg.x = rpy_get(0);
	ang_mst_msg.y = rpy_get(1);
	ang_mst_msg.z = rpy_get(2);
	ang_est_msg.x = rpy_est(0);
	ang_est_msg.y = rpy_est(1);
	ang_est_msg.z = rpy_est(2);
	ang_vel_mst_msg.x = ang_vel.x;
	ang_vel_mst_msg.y = ang_vel.y;
	ang_vel_mst_msg.z = ang_vel.z;
	//mst_msg.x = li_ac.x;mst_msg.y = li_ac.y;mst_msg.z = li_ac.z;
	pose_msg.x = xhat(0,0);
	pose_msg.y = xhat(1,0);
	pose_msg.z = xhat(2,0);
	lin_acc_msg.x = li_ac.x;
	lin_acc_msg.y = li_ac.y;
	lin_acc_msg.z = li_ac.z;
	lin_acc_wrd_msg.x = y(0,0);
	lin_acc_wrd_msg.y = y(1,0);
	lin_acc_wrd_msg.z = y(2,0);
	lin_acc_wrd_kf_msg.x = xhat(6,0);
	lin_acc_wrd_kf_msg.y = xhat(7,0);
	lin_acc_wrd_kf_msg.z = xhat(8,0);
    pub_pose.publish(pose_msg);
	pub_ang_raw.publish(ang_raw_msg);
	pub_ang_mst.publish(ang_mst_msg);
	pub_ang_est.publish(ang_est_msg);
	pub_ang_vel_mst.publish(ang_vel_mst_msg);
	pub_acc_lcl.publish(lin_acc_msg);
	pub_acc_wrd.publish(lin_acc_wrd_msg);
	pub_acc_wrd_kf.publish(lin_acc_wrd_kf_msg);
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

	ros::init(argc, argv,"kalman_pose");
	IMU imu;
	ros::NodeHandle nh;
	imu.imu_sub = nh.subscribe("ardrone/imu", 10, &IMU::imuCb, &imu);
	ROS_INFO_STREAM("kalmanfilter started!");
	ros::spin();

	return 0;
  }
