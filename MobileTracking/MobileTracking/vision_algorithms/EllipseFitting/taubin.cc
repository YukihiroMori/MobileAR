//
// Taubin method for ellipse fitting
//

#include "EllipseFitting.h"

using namespace Eigen;

// Taubin method
bool
EllipseFitting::taubin(
   Vector2d pos[],   // Point Positions:      INPUT
   int Num,          // Number of Data:       INPUT
   Vector6d &theta,  // Ellipse Parameter:     OUTPUT
   double f0         // Default focal length: INPUT w. default
                      )
{
   Vector6d Xi;
   Matrix6d V0;
   Matrix6d M, N;

   // Compute Moment Matrix
   M = N = ZeroMat6;
   for (int al = 0; al < Num; al++)
   {
      Xi = convert_vec6(pos[al], f0);
      V0 = make_cov_mat(pos[al], f0);
      M += Xi * Xi.transpose();
      N += V0;
   }

   M /= (double)Num;
   N /= (double)Num;

   // Compute eigen vector corresponding to the maximum(*) eigen value
   GeneralizedSelfAdjointEigenSolver<Matrix6d> GES(N, M);
   if (GES.info() != Success) return false;
   theta = GES.eigenvectors().col(5);

   // normalization
   theta.normalize();

   return true;
}
