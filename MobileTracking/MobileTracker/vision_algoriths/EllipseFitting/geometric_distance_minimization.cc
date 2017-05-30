//
// Geometric distance minimization for ellipse fitting
//

#include "EllipseFitting.h"

using namespace Eigen;

typedef Matrix<double,2,3> Matrix23d;

// Geometric distance minimization
bool
EllipseFitting::geometric_distance_minimization(
   Vector2d p0[],     // Data:                      INPUT
   int      Num,      // Number of Data:            INPUT
   Vector6d &theta,   // Ellipse Parameter:         OUTPUT
   int      Max_Iter, // Max. Iteration:            INPUT w. default
   double   Conv_EPS, // Threshold for convergence: INPUT w. default
   double   f0        // Default focal length:      INPUT w. default
)
{
   Vector2d     ph[Num];       // hat x and y
   Vector2d     pt[Num];       // tilde x and y
   Vector6d     Xst[Num];      // Xi*
   Matrix6d     V0[Num];
   double       W[Num];
   Matrix6d     M;
   Matrix23d    thMat;
   Vector3d     xhat;     
   double       J0, Jst;
   int          count;

   // Initialization
   for (int al = 0; al < Num; al++)
   {
      ph[al] = p0[al];
      pt[al] << 0.0, 0.0;
      W[al] = 1.0;
   }

   // set large number to J0 
   J0 = Large_Number;
   
   // Update by iteration
   count = 0;
   do
   {
      // Set vectors and covariance matrices
      for (int al = 0; al < Num; al++)
      {
         Xst[al] = convert_vec6_st(ph[al], pt[al], f0);
         V0[al] = make_cov_mat(ph[al], f0);
      }

      // Solve minimization
      if (!fns_core(Xst, V0, W, Num, theta, M, Max_Iter, Conv_EPS))
         return false;
      
      // Update
      Jst = 0.0;
      for (int al = 0; al < Num; al++)
      {
         double c;

         // Update tilde x and y
         thMat << theta[0], theta[1], theta[3], theta[1], theta[2], theta[4];
         c = 2.0*theta.dot(Xst[al]) / theta.dot(V0[al]*theta);
         xhat << ph[al](0), ph[al](1), f0;
         pt[al] = c * thMat * xhat;

         // Update hat x and y
         ph[al] = p0[al] - pt[al];

         // accumulate J*
         Jst += pt[al].squaredNorm();
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
