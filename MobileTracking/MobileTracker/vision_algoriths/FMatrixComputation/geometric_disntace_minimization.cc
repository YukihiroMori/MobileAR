//
// Geometric distance minimization for Fundamental Matrix correction
//

#include "FMatrixComputation.h"

using namespace Eigen;

// Geometric distance minimization
bool
FMatrixComputation::geometric_distance_minimization(
   Vector2d p0[],     // Data:                      INPUT
   Vector2d p1[],     // Data:                      INPUT
   int      Num,      // Number of Data:            INPUT
   const Vector9d &theta0,   // Fundamental Matrix:  INPUT
   Vector9d &theta,  // corrected Fundamental Matrix: OUTPUT
   int      Max_Iter, // Max. Iteration:            INPUT w. default
   double   Conv_EPS, // Threshold for convergence: INPUT w. default
   double   f0        // Default focal length:      INPUT w. default
)
{
   Vector2d     p0h[Num], p1h[Num];       // hat x, y, x', and y'
   Vector2d     p0t[Num], p1t[Num];       // tilde x, y, x', and y'
   Vector9d     Xst[Num];                 // Xi*
   Matrix9d     V0[Num];
   Matrix23d    thM0, thM1;
   Vector3d     x0h, x1h;
   double       J0, Jst;
   int          count;

   // Initialization
   for (int al = 0; al < Num; al++)
   {
      p0h[al] = p0[al];
      p1h[al] = p1[al];
      p0t[al] = p1t[al] = ZeroVec2;
   }

   // set large number to J0 
   J0 = Large_Number;
   
   // Update by iteration
   theta = theta0;
   count = 0;
   do
   {
      // Set vectors and covariance matrices
      for (int al = 0; al < Num; al++)
      {
         Xst[al] = convert_vec9_st(p0h[al], p0t[al], p1h[al], p1t[al], f0);
         V0[al] = make_cov_mat(p0h[al], p1h[al], f0);
      }

      // Solve minimization
      if (!efns_core(theta, Xst, V0, Num, Max_Iter, Conv_EPS))
         return false;

      // Update
      Jst = 0.0;
      // set Matrix
      thM0 << theta(0), theta(1), theta(2), theta(3), theta(4), theta(5);
      thM1 << theta(0), theta(3), theta(6), theta(1), theta(4), theta(7);
      for (int al = 0; al < Num; al++)
      {
         double c;

         // Update tilde x and y
         c = theta.dot(Xst[al]) / theta.dot(V0[al]*theta);
         x0h << p0h[al](0), p0h[al](1), f0;
         x1h << p1h[al](0), p1h[al](1), f0;
         p0t[al] = c * thM0 * x1h;
         p1t[al] = c * thM1 * x0h;

         // Update hat x and y
         p0h[al] = p0[al] - p0t[al];
         p1h[al] = p1[al] - p1t[al];

         // accumulate J*
         Jst += p0t[al].squaredNorm() + p1t[al].squaredNorm();
      }
      Jst /= (double)Num;

      // check convergence 
      if (fabs(Jst - J0) < Conv_EPS) break;
      
      J0 = Jst;
   }
   while (++count < Max_Iter);

   // Maximum Iteration
   if (count >= Max_Iter)  return false;

   return true;
}
