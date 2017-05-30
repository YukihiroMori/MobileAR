//
// Least squares for homography computation
// 

#include "HomographyComputation.h"

using namespace Eigen;

// Least-squares method
bool
HomographyComputation::least_squares(
   Vector2d pos0[], Vector2d pos1[],
   int Num,
   Matrix3d& H,
   double f0)
{
   Vector9d h;
   Matrix9d M;
   Vector9d xi1, xi2, xi3;

   // Compute moment matrix M
   M = ZeroMat9;
   for (int al = 0; al < Num; al++)
   {
      xi1 = set_xi1(pos0[al], pos1[al], f0);
      xi2 = set_xi2(pos0[al], pos1[al], f0);
      xi3 = set_xi3(pos0[al], pos1[al], f0);

      M += xi1*xi1.transpose()
	 + xi2*xi2.transpose()
	 + xi3*xi3.transpose();
   }
   M /= (double)Num;

   // Solving eigen value problem
   SelfAdjointEigenSolver<Matrix9d> ES(M);

   // let h be the eigenvector corresponding
   // to the smallest eigenvalue.
   h = ES.eigenvectors().col(0);

   H = convert_mat3(h);

   return true;
}
