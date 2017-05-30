//
// Renormalization for ellipse fitting
//

#include "EllipseFitting.h"

using namespace Eigen;

// iterative reweight method (proc. 2-2)
bool
EllipseFitting::renormalization(
   Vector2d pos[],  // Point Positions:           INPUT
   int Num,         // Number of Data:            INPUT
   Vector6d &theta, // Ellipse Parameter:          OUTPUT
   int Max_Iter,    // Max. Iteration:            INPUT w. default
   double Conv_EPS, // Threshold for convergence: INPUT w. default
   double f0        // Default focal length:      INPUT w. default
                               )
{
   Vector6d     Xi[Num];
   Matrix6d     V0[Num];
   double       W[Num];
   Vector6d     theta0, Dp, Dm;
   Matrix6d     M, N;
   double       diffp, diffm;
   int          count;

   // Construct Generalized Eigen Solver
   GeneralizedSelfAdjointEigenSolver<Matrix6d> GES;

   // Initialization
   theta0 = ZeroVec6;
   for (int al = 0; al < Num; al++)
   {
      W[al] = 1.0;
      Xi[al] = convert_vec6(pos[al], f0);
      V0[al] = make_cov_mat(pos[al], f0);
   }

   // Update by iteration
   count = 0;
   do
   {
      // Compute Moment Matrix
      M = N = ZeroMat6;
      for (int al = 0; al < Num; al++)
      {
         M += W[al] * Xi[al] * Xi[al].transpose();
         N += W[al] * V0[al];
      }

      M /= (double)Num;
      N /= (double)Num;

      // Compute eigen vector corresponding to the maximum(*) eigen value
      // (*) Matrices M and N are swapped in the following function.
      GES.compute(N, M);
      if (GES.info() != Success) return false;
      theta = GES.eigenvectors().col(5);
      theta.normalize();

      // Calc the differences
      Dp = theta0 + theta;
      Dm = theta0 - theta;
      diffp = Dp.norm();
      diffm = Dm.norm();

      // Check the convergence
      if (diffp < Conv_EPS || diffm < Conv_EPS) break;

      // Update Weights
      theta0 = theta;
      for (int al = 0; al < Num; al++)
         W[al] = 1.0 / theta.dot(V0[al] * theta);
   }
   while (++count < Max_Iter);

   // Maximum Iteration
   if (count >= Max_Iter)  return false;

   return true;
}
