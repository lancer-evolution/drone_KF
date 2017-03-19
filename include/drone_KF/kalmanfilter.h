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
  P_ Pm, P_new;

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
  xhatm = (A*xhat).array() + (Bu*u).array(); // 状態
  Pm = A*P*A.transpose() + B*Q*B.transpose(); // 誤差共分散
  /*cout << "xhatm" << xhatm << endl
	<< "Pm" << Pm << endl;
  */
  // カルマンゲイン行列
  G = (Pm*C).array() /((C.transpose()*Pm*C).array() + R)(0,0);
  //cout << "G" << G << endl;

 
  // 事後推定値
  MatrixXd eye = MatrixXd::Identity(A.rows(),A.cols());
  //xhat_new = xhatm.col(0) + G*(y - C.transpose()*xhatm); // 状態
  xhat = xhatm + G*(y - C.transpose()*xhatm); // 状態
  //P_new = (eye - G*C.transpose())*Pm;
  P = (eye - G*C.transpose())*Pm;

  }

};

 // end namespace

#endif	/* KALMANFILTER_H_ */

