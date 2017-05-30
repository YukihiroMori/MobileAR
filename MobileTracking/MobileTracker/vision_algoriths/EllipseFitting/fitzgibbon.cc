//
// Fitzgibbon's method for ellipse fitting
//

#include "EllipseFitting.h"

using namespace Eigen;

// Fitsgibbon's method
bool
EllipseFitting::fitzgibbon(
   Vector2d pos[],      // Point Positions:             INPUT
   int Num,             // Number of data:              INPUT
   Vector6d &theta,     // Ellipse Parameter:            OUTPUT
   double f0            // Default focal length:        INPUT w. default
                          )
{
   Vector6d     Xi[Num];
   Matrix6d     M, N;

   // Initialization
   M = ZeroMat6;
   for (int al = 0; al < Num; al++)
   {
      Xi[al] = convert_vec6(pos[al], f0);
      M += Xi[al] * Xi[al].transpose();
   }
   M /= (double)Num;

   // set matrix N
   N = ZeroMat6;
   N(2,0) = N(0,2) = 1.0;
   N(1,1) = -2.0;

   // Construct Generalized Eigen Solver
   // maximized generalized eigen value
   GeneralizedSelfAdjointEigenSolver<Matrix6d> GES(N,M);

   if (GES.info() != Success) return false;
   theta = GES.eigenvectors().col(5);

   // normalization
   theta.normalize();

   return true;
}
