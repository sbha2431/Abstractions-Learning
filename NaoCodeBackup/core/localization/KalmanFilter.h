#include <Eigen/LU>
#include <Eigen/Core>
#pragma once


template <int m,int n>
class KalmanFilter{

private:

  // System dimensions

  Eigen::Matrix<double,m,n> H;
  Eigen::Matrix<double,m,m> R; 
  Eigen::Matrix<double,n,n> Phi;
  Eigen::Matrix<double,n,n> Q;
  Eigen::Matrix<double,n,n> P;
  Eigen::Matrix<double,n,n> P0;
  Eigen::Matrix<double,n,m> K;

  // Initial and current time
  double t0, t;

  // Discrete time step
  double dt;

  // Is the filter initialized?
  bool initialized;

  // n-size identity
  Eigen::Matrix<double,n,n> I;

  // Estimated states
  // Eigen::VectorXd x_hat, x_hat_new;

  Eigen::Matrix<double,n,1> x_hat;
  Eigen::Matrix<double,n,1> x_hat_new;
  // template <typename Type,int n> using Vector = Eigen::Matrix<Type, n, 1>;
  // Vector <double,n> x_hat;
  // Vector <double,n> x_hat_new;
  // template<typename T,int n> static Eigen::Vector<T,n> x_hat_new;

public:

  KalmanFilter(
    // static int m, n,
    // const Eigen::Matrix<double,n,n>& Phi,
    // const Eigen::Matrix<double,n,n>& H,
    // const Eigen::Matrix<double,n,n>& Q,
    // const Eigen::Matrix<double,n,n>& R,
    // const Eigen::Matrix<double,n,n>& P,
    // const Eigen::Matrix<double,n,n>& K0
      double dt,
      Eigen::Matrix<double,m,n>& H,
      Eigen::Matrix<double,m,m>& R,
      Eigen::Matrix<double,n,n>& Phi,
      Eigen::Matrix<double,n,n>& Q,
      Eigen::Matrix<double,n,n>& P
  );


  /**
  * Create a blank estimator.
  */
  KalmanFilter();

  /**
  * Initialize the filter with initial states as zero.
  */
  void init();

  /**
  * Initialize the filter with a guess for initial states.
  */
  void init(double t0, Eigen::Matrix<double,n,1>& x0);

  /**
  * Update the estimated state based on measured values. The
  * time step is assumed to remain constant.
  */
  void timeUpdate();
  void measurementUpdate(Eigen::Matrix<double,m,1>& z);

  /**
  * Update the estimated state based on measured values,
  * using the given time step and dynamics matrix.
  */
  void update(Eigen::Matrix<double,m,1>& z, double dt, Eigen::Matrix<double,n,n> Phi);

  Eigen::Matrix<double,n,1> propogateState(int T,Eigen::Matrix<double,n,1> x_curr); 

  Eigen::Matrix<double,n,n> propogateCovariance(int T,Eigen::Matrix<double,n,n> P_curr);

  /**
  * Return the current state and time.
  */
  Eigen::Matrix<double,n,1> state() { return x_hat; };
  Eigen::Matrix<double,n,n> covariance() { return P; };
  float Pnorm() { return P.norm(); };
  double time() { return t; };

};
