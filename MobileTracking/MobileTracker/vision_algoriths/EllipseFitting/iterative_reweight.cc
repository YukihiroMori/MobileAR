//
// Iterative reweight method for ellipse fitting
//

#include "EllipseFitting.h"

using namespace Eigen;

// iterative reweight method (proc. 2-1)
bool
EllipseFitting::iterative_reweight(
   Vector2d pos[],   // Point Coordinates:         INPUT
   int Num,          // Number of Data:            INPUT
   Vector6d &theta,  // Ellipse Parameter:          OUTPUT
   int Max_Iter,     // Max. Iteration :           INPUT w. default
   double Conv_EPS,  // Threshold for convergence: INPUT w. default
   double f0         // Default focal length:      INPUT w. default
                                  )
{
   Vector6d     Xi[Num];
   Matrix6d     V0[Num];
   Vector6d     theta0;
   Matrix6d     M;
   double       W[Num];
   double       diffp, diffm;
   int          count;

   // Construct Eigen Solver
   SelfAdjointEigenSolver<Matrix6d> ES;

   // Initialization
   theta0 = ZeroVec6;
   for (int al = 0; al < Num; al++)
   {
      Xi[al] = convert_vec6(pos[al], f0);
      V0[al] = make_cov_mat(pos[al], f0);
      W[al] = 1.0;
   }

   // Update by iteration
   count = 0;
   do
   {
      // Compute Moment Matrix
      M = ZeroMat6;
      for (int al = 0; al < Num; al++)
         M += W[al] * Xi[al] * Xi[al].transpose();
      M /= (double)Num;

      // Compute eigen vector corresponding to the smallest eigen value
      ES.compute(M);
      if (ES.info() != Success) return false;
      theta = ES.eigenvectors().col(0);

      // Calc the differences
      diffp = (theta0 + theta).norm();
      diffm = (theta0 - theta).norm();

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
