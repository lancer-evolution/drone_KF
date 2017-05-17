#ifndef _KALMAN_ANG_H_
#define _KALMAN_ANG_H_

// カルマンフィルタによる角度推定
// 入力：角速度、観測：角度
// 状態変数:角度、バイアス

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


class kf_angle{
 public:
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
  
  kf_angle(){
	gamma = 1;
	Q = 1;
	R = 100;
	dt = 0.02;
	
	A << 1, -dt,
	  0, 1;
	
	bu << dt,
	  0;
	b << 1,
	  0;
	c << 1,
	  0;
	P = gamma * P;
	
  }
  
  double kalman(double ori, double ori_v){
	/*kalman filter*/
	y << ori;
	//kf.kf(A,b,bu,c,Q,R,0,y,xhat,P);
	kf.kf(A,b,bu,c,Q,R,ori_v,y,xhat,P);
	//cout << y << " " << xhat(0,0) << " " << xhat(1,0) << endl;

	cout << "vel:" << ori_v << endl
	  << "bias:" << xhat(1,0) << endl;
	return xhat(0,0);
  }
};


#endif/*KALMAN_ANG_H*/
