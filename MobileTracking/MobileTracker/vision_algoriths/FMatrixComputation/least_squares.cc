//
// Least Squares for Fundamental Matrix Computation
//

#include "FMatrixComputation.h"

using namespace Eigen;

// least squares
bool
FMatrixComputation::least_squares(
   Vector2d pos0[],
   Vector2d pos1[],
   int      Num,
   Vector9d& theta,
   double    f0)
{
   Vector9d     Xi;
   Matrix9d     M;

   // Initialization
   M = ZeroMat9;

   // Calc Moment Matrix
   for (int al = 0; al < Num; al++)
   {
      Xi = convert_vec9(pos0[al], pos1[al], f0);
      M += Xi * Xi.transpose();
   }
   M /= (double)Num;

   // Compute eigen vector corresponding to the smallest eigen value
   SelfAdjointEigenSolver<Matrix9d> ES(M);
   if (ES.info() != Success) return false;
   theta = ES.eigenvectors().col(0);

   return true;
}
