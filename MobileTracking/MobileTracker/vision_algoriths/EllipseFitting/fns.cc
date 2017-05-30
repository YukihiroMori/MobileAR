//
// FNS (Fundamental Numerical Scheme) method for ellipse fitting
//

#include "EllipseFitting.h"

using namespace Eigen;

// FNS core routine
bool
EllipseFitting::fns_core(
   Vector6d Xi[],
   Matrix6d V0[],
   double W[],
   int Num,
   Vector6d &theta,
   Matrix6d &M,
   int Max_Iter,
   double Conv_EPS
                        )
{
   Vector6d     theta0;
   Matrix6d     L, X;
   double       diffp, diffm;
   int          count;

   // Construct Eigen Solver
   SelfAdjointEigenSolver<Matrix6d> ES;

   // initialization
   theta = theta0 = ZeroVec6;

   // Update by iteration
   count = 0;
   do
   {
      // Compute Moment Matrices
      M = L = ZeroMat6;
      for (int al = 0; al < Num; al++)
      {
         M += W[al] * Xi[al] * Xi[al].transpose();
         L += sqr(W[al]* theta.dot(Xi[al])) * V0[al];
      }

      // Compute Matrix X
      X = (M - L)/(double)Num;

      // Compute eigen vector corresponding to the smallest eigen value
      // of Matrix X
      ES.compute(X);
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
   

// FNS method
bool
EllipseFitting::fns(
   Vector2d pos[],  // Point Positions:           INPUT
   int Num,         // Number of Data:            INPUT
   Vector6d &theta, // Ellipse Parameter:         OUTPUT
   int Max_Iter,    // Max. Iteration:            INPUT w. default
   double Conv_EPS, // Threshold for convergence: INPUT w. default
   double f0        // Default focal length:      INPUT w. default
                   )
{
   Vector6d     Xi[Num];
   Matrix6d     V0[Num];
   double       W[Num];
   Matrix6d     M;
   
   // Initialization
   for (int al = 0; al < Num; al++)
   {
      Xi[al] = convert_vec6(pos[al], f0);
      V0[al] = make_cov_mat(pos[al], f0);
      W[al] = 1.0;
   }

   // call fns core routine
   if (!fns_core(Xi, V0, W, Num, theta, M, Max_Iter, Conv_EPS))
      return false;

   return true;
}
