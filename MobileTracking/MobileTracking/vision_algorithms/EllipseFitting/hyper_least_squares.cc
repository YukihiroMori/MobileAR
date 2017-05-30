//
// Hyper least squares for ellipse fitting
//

#include "EllipseFitting.h"

using namespace Eigen;

// hyper least squares
bool
EllipseFitting::hyper_least_squares(
   Vector2d pos[],  // Point Positions:           INPUT
   int Num,         // Number of Data:            INPUT
   Vector6d &theta, // Ellipse Parameter:          OUTPUT
   double f0        // Default focal length:      INPUT w. default
                                   )
{
   Vector6d     Xi[Num];
   Matrix6d     V0[Num];
   Matrix6d     M, M5i, N, N1, N2;
   Matrix6d     T;
   Vector6d     M5Xi;
   RowVector6d  evec6;

   // Initialize vector
   evec6 << 1, 0, 1, 0, 0, 0;

   // Construct Generalized Eigen Solver
   GeneralizedSelfAdjointEigenSolver<Matrix6d> GES;

   // Compute Matrices M and N1
   M = N1 = N2 = ZeroMat6;
   for (int al = 0; al < Num; al++)
   {
      Xi[al] = convert_vec6(pos[al], f0);
      V0[al] = make_cov_mat(pos[al], f0);
      M += Xi[al] * Xi[al].transpose();
      T = Xi[al] * evec6;
      N1 += V0[al] + Sym2(T);
   }
   M /= (double)Num;

   // Compute Generalized inverse with rank 5
   M5i = trancated_generalized_inverse(M, 5);

   // Compute Matrix N2
   for (int al = 0; al < Num; al++)
   {
         M5Xi = M5i * Xi[al];
         T = V0[al] * M5Xi * Xi[al].transpose();
         N2 += Xi[al].dot(M5Xi) * V0[al] + Sym2(T);
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

   return true;
}
