//
// Taubin method for Fundamental Matrix Computation
//

#include "FMatrixComputation.hpp"

using namespace Eigen;

// Taubin method (core)
bool
FMatrixComputation::taubin_core(
   std::vector<Vector9d> Xi,
   std::vector<Matrix9d> V0,
   int      Num,
   Vector9d& theta
)
{
   Matrix9d     M, N;

   // Initialization
   M = N = ZeroMat9;

   // Calc Matrices
   for (int al = 0; al < Num; al++)
   {
      M += Xi[al] * Xi[al].transpose();
      N += V0[al];
   }
   M /= (double)Num;
   N /= (double)Num;

   // Compute eigen vector corresponding to the maximum(*) eigen value
   GeneralizedSelfAdjointEigenSolver<Matrix9d> GES(N, M);
   if (GES.info() != Success) return false;
   theta = GES.eigenvectors().col(8);

   // need normalization
   theta.normalize();

   return true;
}

// Taubin method
bool
FMatrixComputation::taubin(
   std::vector<Vector2d> pos0,
   std::vector<Vector2d> pos1,
   int      Num,
   Vector9d& theta,
   double    f0
)
{
   std::vector<Vector9d> Xi(Num);
   std::vector<Matrix9d> V0(Num);
   Matrix9d     M, N;

   // set Xi and V0
   for (int al = 0; al < Num; al++)
   {
      Xi[al] = convert_vec9(pos0[al], pos1[al], f0);
      V0[al] = make_cov_mat(pos0[al], pos1[al], f0);
   }

   return taubin_core(Xi, V0, Num, theta);
}
