#include <iostream>
#include "lqr_control.h"

using namespace std;
using namespace Eigen;

LQRControl::LQRControl(){



}


LQRControl::LQRControl(double carMass, double poleMass, double rotInertia, double rodDistance)
{
  M = carMass;
  m = poleMass;
  J = rotInertia;
  l = rodDistance;
  g = 9.8;

  double alpha = (M + m) * (J + m * pow(l, 2.0)) - pow(m, 2.0) * pow(l, 2.0);

  //Matrix
  A << 0, 1, 0, 0,
       0, 0, -(pow(m, 2.0) * pow(l, 2.0) * g) / alpha, 0,
       0, 0, 0, 1,
       0, 0, ((M + m) * m * g * l) / alpha, 0;

  B << 0, (J + m * pow(l, 2.0)) / alpha, 0, -(m * l) / alpha;

  C << 1, 0, 0, 0,
       0, 0, 1, 0;

  D << 0, 0;


  Q << 10, 0, 0, 0,
       0, 10, 0, 0,
       0, 0, 10, 0,
       0, 0, 0, 10;

  R << 1;

}

MatrixXd  LQRControl::care(const MatrixXd &A, const MatrixXd &B, const MatrixXd &Q, const MatrixXd &R)
{

}
