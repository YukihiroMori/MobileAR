//
// 3-D reconstruction from 2 views
//

#include <iostream>

#include "TwoViewReconstruction.h"

using namespace Eigen;

// unnamed namespace for local function
namespace {
// sign function
int
sgn(double x)
{
   if (fabs(x) < TwoViewReconstruction::Almost0)
      return 0;
   else if (x < 0.0)
      return -1;
   else
      return 1;
}
   
}
// end of unnamed namespace

// 3-D reconstruction from two views
bool
TwoViewReconstruction::reconstruction(Vector2d pos0[],
                                      Vector2d pos1[],
                                      int Num,
                                      const Vector9d& theta,
                                      const Matrix3d& R,
                                      const Vector3d& t,
                                      double f,
                                      double fp,
                                      Vector3d X[],
                                      int Max_Iter,
                                      double Conv_EPS,
                                      double f0
)
{
   Matrix34d    P, Pp, T, Tp;
   DiagonalMatrix<double,3>     A(f, f, f0);
   DiagonalMatrix<double,3>     Ap(fp, fp, f0);
   Vector2d     np0[Num];
   Vector2d     np1[Num];
   double       J;

   // Projection (camera) matrices
   T.block<3,3>(0,0) = I3;
   T.block<3,1>(0,3) = ZeroVec3;
   Tp.block<3,3>(0,0) = R.transpose();
   Tp.block<3,1>(0,3) = -R.transpose() * t;
   P = A * T;
   Pp = Ap * Tp;

   // Optimally correct image points
   Triangulation::optimal_correction(pos0, pos1, Num, theta, np0, np1,
                                     Max_Iter, Conv_EPS, f0);

   // Reconstruct 3-D points from corrected image positions
   Triangulation::least_squares(np0, np1, Num, P, Pp, X, f0);

   // check sign
   J = 0.0;
   for (int al = 0; al < Num; al++)
      J += sgn(X[al](2));

   // change sign
   if (J < 0)
   {
      for (int al = 0; al < Num; al++)
         X[al] = -X[al];
   }

   return true;
}

// 3-D reconstruction from two views
bool
TwoViewReconstruction::reconstruction(Vector2d pos0[],
                                      Vector2d pos1[],
                                      int Num,
                                      const Matrix3d& F,
                                      const Matrix3d& R,
                                      const Vector3d& t,
                                      double f,
                                      double fp,
                                      Vector3d X[],
                                      int Max_Iter,
                                      double Conv_EPS,
                                      double f0)
{
   Vector9d theta = convert_vec9(F);

   return reconstruction(pos0, pos1, Num, theta, R, t, f, fp, X, Max_Iter, Conv_EPS, f0);
}
