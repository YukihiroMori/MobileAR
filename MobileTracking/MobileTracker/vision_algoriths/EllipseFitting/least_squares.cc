//
// Least squares for ellipse fitting
//

#include "EllipseFitting.h"

using namespace Eigen;

// least squares
bool
EllipseFitting::least_squares(
   Vector2d pos[],    // Point Coordinates:    INPUT
   int Num,           // Number of Data:       INPUT
   Vector6d &theta,   // Ellipse Parameter:     OUTPUT
   double f0          // Default focal length: INPUT w. default
                             )
{
   Vector6d     Xi;
   Vector6d     theta0;
   Matrix6d     M;

   // Compute Moment Matrix
   M = ZeroMat6;
   for (int al = 0; al < Num; al++)
   {
      // convert position to 6-vector
      Xi = convert_vec6(pos[al], f0);
      // accumulate M
      M += Xi * Xi.transpose();
   }
   M /= (double)Num;

   // Compute eigen vector corresponding to the smallest eigen value
   SelfAdjointEigenSolver<Matrix6d> ES(M);
   if (ES.info() != Success) return false;
   theta = ES.eigenvectors().col(0);

   return true;
}
