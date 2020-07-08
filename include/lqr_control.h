#ifndef LQR_CONTROL
#define LQR_CONTROL

#include <eigen3/Eigen/Dense>

using namespace Eigen;

class LQRControl
{

private:

  double M;//car mass
  double m;//pole mass
  double J;//rotational inertia of rod
  double l;//distance to the center of rod mass
  double g;//gravity


  //system matrix
  Matrix<double,4,4> A;
  Matrix<double,4,1> B;
  Matrix<double,2,4> C;
  Matrix<double,2,1> D;

  //weight
  Matrix<double,4,4> Q;
  Matrix<double,1,1> R;

  //Init
  Matrix<double,4,1> x;
  Matrix<double,1,1> u;

  //LQR Gain
  Matrix<double,1,4> K;

public:
  LQRControl();
  LQRControl(double carMass, double poleMass, double rotInertia, double rodDistance);
  ~LQRControl();

  /**
   * @brief care: Continuous-time Algebraic Riccati Equation
   * @param A
   * @param B
   * @param Q
   * @param R
   * @return
   */
  MatrixXd care(const MatrixXd &A, const MatrixXd &B, const MatrixXd &Q, const MatrixXd &R);
  MatrixXd calcGainK();


private:

};

#endif
