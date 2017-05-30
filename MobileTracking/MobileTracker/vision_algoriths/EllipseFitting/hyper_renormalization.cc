//
// Hyper renormalization for ellipse fitting
//

#include "EllipseFitting.h"

using namespace Eigen;

// Hyper renormalization core function
bool
EllipseFitting::hyper_renormalization_core(
   Vector6d     Xi[],
   Matrix6d     V0[],
   double       W[],
   int          Num,
   Vector6d     &theta,
   int          Max_Iter,
   double       Conv_EPS
                                          )
{
   Vector6d     theta0;
   Matrix6d     M, N, N1, N2, M5i;
   double       diffp, diffm;
   Matrix6d     T;
   Vector6d     M5Xi;
   RowVector6d  evec6;
   int          count;

   // Initialize vector
   evec6 << 1, 0, 1, 0, 0, 0;

   // Construct Generalized Eigen Solver
   GeneralizedSelfAdjointEigenSolver<Matrix6d> GES;

   // Initialization
   theta0 = ZeroVec6;

   // Update by iteration
   count = 0;
   do
   {
      // Compute Matrices M and N1
      M = N1 = N2 = ZeroMat6;
      for (int al = 0; al < Num; al++)
      {
         M += W[al] * Xi[al] * Xi[al].transpose();
         T = Xi[al] * evec6;
         N1 += W[al] * (V0[al] + Sym2(T));
      }

      M /= (double)Num;

      // Compute Generalized inverse with rank 5
      M5i = trancated_generalized_inverse(M, 5);

      // Compute Matrix N2
      for (int al = 0; al < Num; al++)
      {
         M5Xi = M5i * Xi[al];
         T = V0[al]*M5Xi*Xi[al].transpose();
         N2 += sqr(W[al]) * (Xi[al].dot(M5Xi) * V0[al] + Sym2(T));
      }

      // Compute Matrix N
      N = N1 / (double)Num - N2 / (double)(Num*Num);

      // Compute eigen vector corresponding to the absolutely maximum(*)
      // eigen value(*) Matrices M and N are swapped in the following
      // function.
      GES.compute(N, M);
      if (GES.info() != Success) return false;
      if (fabs(GES.eigenvalues()(5)) > fabs(GES.eigenvalues()(0)))
         theta = GES.eigenvectors().col(5); // largest with plus sign
      else
         theta = GES.eigenvectors().col(0); // largest with minus sign

      // normalization
      theta.normalize();

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

// Hyper renormalization (proc. 2-3)
bool
EllipseFitting::hyper_renormalization(
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

   // Initialization
   for (int al = 0; al < Num; al++)
   {
      Xi[al] = convert_vec6(pos[al], f0);
      V0[al] = make_cov_mat(pos[al], f0);
      W[al] = 1.0;
   }

   if (!hyper_renormalization_core(Xi, V0, W, Num, theta, Max_Iter, Conv_EPS))
      return false;

   return true;
}
