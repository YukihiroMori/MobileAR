//
// Hyperaccurate correction for ellipse fitting
//

#include "EllipseFitting.h"

using namespace Eigen;

// Hyperaccurate correction 
bool
EllipseFitting::hyperaccurate_correction(
   Vector2d pos[],  // Data Vectors:              INPUT
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
   Matrix6d     M, M5i;
   Vector6d     t1, t2;
   Vector6d     d_th;
   double       sg2;
   Vector6d     evec6;

   // set vector
   evec6 << 1, 0, 1, 0, 0, 0;
   
   // Initialization
   for (int al = 0; al < Num; al++)
   {
      Xi[al] = convert_vec6(pos[al], f0);
      V0[al] = make_cov_mat(pos[al], f0);
      W[al] = 1.0;
   }
   
   // Compute theta by FNS
   if (!fns_core(Xi, V0, W, Num, theta, M, Max_Iter, Conv_EPS))
      return false;

   // Calc. sigma^2
   sg2 = theta.dot(M*theta) / (1.0 - 5.0/Num);

   M5i = trancated_generalized_inverse(M, 5);

   t1 = t2 = ZeroVec6;
   for (int al = 0; al < Num; al++)
   {
      t1 += W[al] * theta.dot(evec6) * Xi[al];
      t2 += sqr(W[al]) * Xi[al].dot(M5i*V0[al]*theta) * Xi[al];
   }
   d_th = sg2*M5i*(- t1/Num + t2/sqr((double)Num));

   theta -= d_th;
   theta.normalize();
      
   return true;
}
