#ifndef _KALMANFILTER_H_
#define _KALMANFILTER_H_

#include <iostream>
#include <Eigen/Core>

using namespace Eigen;
using namespace std;

class kalmanfilter{
 public:
  void test();
  template<typename T> void temp(T a){
	cout << a << endl;
  }
  kalmanfilter();
  
  template<typename A_,typename B_, typename Bu_, typename C_, typename Q_, typename R_, typename u_, typename y_, typename xhat_, typename P_>
inline void kf(A_ A,B_ B, Bu_ Bu, C_ C, Q_ Q, R_ R, u_ u, y_ y, xhat_& xhat, P_& P){
  xhat_ xhatm, xhat_new, G;
  P_ Pm, S;

  /*cout << "A" << A << endl
	   << "B" << B << endl
	   << "Bu" << Bu << endl
	<< "C" << C << endl
	<< "Q" << Q << endl
	<< "R" << R << endl
	<< "u" << u << endl
	<< "y" << y << endl
	<< "xhat" << xhat << endl
	  << "P" << P << endl;
  */
  //事前推定値
  MatrixXd eye1 = MatrixXd::Identity(A.rows(),A.cols());
  MatrixXd eye2 = MatrixXd::Identity(y.rows(),y.rows());
  xhatm = (A*xhat).array() + (Bu*u).array(); // 状態
  Pm = A*P*A.transpose() + Q*eye1; // 誤差共分散
  //cout << "xhatm" << xhatm << endl;
  //cout << "Pm" << Pm.row(7) << endl;
  //cout << "BQB" << B*Q << endl;
  
  // カルマンゲイン行列
  //cout << "qwe" << ((C.transpose()*Pm*C)) << endl;
  S = (C.transpose()*Pm*C + R*eye2).inverse();
  //cout << "S:" << S << endl;
  G = Pm*C*S;
  //cout << "G" << G << endl;

 
  // 事後推定値
  //xhat_new = xhatm.col(0) + G*(y - C.transpose()*xhatm); // 状態
  xhat = xhatm + G*(y - C.transpose()*xhatm); // 状態
  //cout << "カルマンゲイン:" << G << endl;
  //cout << "観測誤差:" << y - C.transpose()*xhatm << endl;
  P = (eye1 - G*C.transpose())*Pm;
  //cout << "誤差共分散:" << P << endl;
  }

};

 // end namespace

#endif	/* KALMANFILTER_H_ */

