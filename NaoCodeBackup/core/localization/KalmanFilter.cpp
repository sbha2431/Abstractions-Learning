#include <localization/KalmanFilter.h>
#include <Eigen/LU>
#include <Eigen/Core>
#include <stdexcept>
#include <assert.h>
#include <iostream>


template <int m,int n>
KalmanFilter<m,n>::KalmanFilter(
      double dt,
      Eigen::Matrix<double,m,n>& H,
      Eigen::Matrix<double,m,m>& R, 
      Eigen::Matrix<double,n,n>& Phi,
      Eigen::Matrix<double,n,n>& Q,
      Eigen::Matrix<double,n,n>& P
  ) : 
	Phi(Phi), H(H), Q(Q), R(R), P0(P),
	     dt(dt), initialized(false)
	{I.setIdentity();
	  }

template <int m,int n>
KalmanFilter<m,n>::KalmanFilter() {}

template <int m,int n>
void KalmanFilter<m,n>::init(double t0, Eigen::Matrix<double,n,1>& x0) {
  x_hat = x0;
  P = P0;
  this->t0 = t0;
  t = t0;
  initialized = true;
}

template <int m,int n>
void KalmanFilter<m,n>::init() {
  x_hat.setZero();
  P = P0;
  t0 = 0;
  t = t0;
  initialized = true;
}

template <int m, int n>
void KalmanFilter<m,n>::timeUpdate() {

  if(!initialized)
    throw std::runtime_error("Filter is not initialized!");
  // First we do prediction update
  x_hat_new = Phi * x_hat;
  P = Phi*P*Phi.transpose() + Q;
  t += dt;
  x_hat = x_hat_new;
}

template <int m, int n>
void KalmanFilter<m,n>::measurementUpdate(Eigen::Matrix<double,m,1>& z) {

  if(!initialized)
    throw std::runtime_error("Filter is not initialized!");
  // We do prediction update

  K = P*H.transpose()*(H*P*H.transpose() + R).inverse();
  x_hat_new += K * (z - H*x_hat);
  P = (I - K*H)*P;
  x_hat = x_hat_new;
}

template <int m, int n>
Eigen::Matrix<double,n,1> KalmanFilter<m,n>::propogateState(int T,Eigen::Matrix<double,n,1> x_curr) {
  if(!initialized)
    throw std::runtime_error("Filter is not initialized!");
  // First we do prediction update
  Eigen::Matrix<double,n,n> Phi_prop = Phi;
  for(int i=2;i<=T;i++){Phi_prop = Phi_prop*Phi;}
  assert(Phi_prop.cols() ==x_curr.rows());
  return Phi_prop * x_curr;
}

template <int m, int n>
Eigen::Matrix<double,n,n> KalmanFilter<m,n>::propogateCovariance(int T,Eigen::Matrix<double,n,n> P_curr) {
  if(!initialized)
    throw std::runtime_error("Filter is not initialized!");
  // First we do prediction update
  Eigen::Matrix<double,n,n> P_prop = P_curr;
  for(int i=1;i<=T;i++){P_prop = Phi*P_prop*Phi.transpose() + Q;}
  return P_prop;
}

template <int m, int n>
void KalmanFilter<m,n>::update(Eigen::Matrix<double,m,1>& z, double dt, Eigen::Matrix<double,n,n> Phi) {

  this->Phi = Phi;
  this->dt = dt;
  timeUpdate();
  measurementUpdate(z);
}