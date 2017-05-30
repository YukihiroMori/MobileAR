//
// Hyperaccurate correction for Homography Computaion
//

#include "HomographyComputation.h"

using namespace Eigen;

// Hyperaccurate correction 
bool
HomographyComputation::hyperaccurate_correction(
   Vector2d pos0[], // Data Vectors:              INPUT
   Vector2d pos1[], // Data Vectors:              INPUT
   int Num,         // Number of Data:            INPUT
   Matrix3d& H,     // Homography:                OUTPUT
   int Max_Iter,    // Max. Iteration:            INPUT w. default
   double Conv_EPS, // Threshold for convergence: INPUT w. default
   double f0        // Default focal length:      INPUT w. default
)
{
   Vector9d theta, d_th, tt;
   Matrix94d Ta[Num][3];
   Vector9d xi[Num][3];
   Matrix3d Wa[Num];
   Matrix9d M, M8i, V0mn;
   double sg2;

   // Initialization
   for (int i = 0; i < Num; i++)
   {
      // set xi
      xi[i][0] = set_xi1(pos0[i], pos1[i], f0);
      xi[i][1] = set_xi2(pos0[i], pos1[i], f0);
      xi[i][2] = set_xi3(pos0[i], pos1[i], f0);

      // set Ta
      Ta[i][0] = Ta[i][1] = Ta[i][2] = ZeroMat94;
      set_Ta1(Ta[i][0], pos0[i], pos1[i], f0);
      set_Ta2(Ta[i][1], pos0[i], pos1[i], f0);
      set_Ta3(Ta[i][2], pos0[i], pos1[i], f0);

      // set weight
      Wa[i] = I3;
   }

   // call fns core routine
   if (!fns_core(xi, Ta, Wa, Num, theta, M, Max_Iter, Conv_EPS))
      return false;

   // Calc. sigma^2
   sg2 = theta.dot(M*theta) / (2.0*(1.0 - 4.0/Num));

   M8i = trancated_generalized_inverse(M, 8);

   tt = ZeroVec9;
   for (int al = 0; al < Num; al++)
   {
      for (int k = 0; k < 3; k++)
         for (int l = 0; l < 3; l++)
            for (int m = 0; m < 3; m++)
               for (int n = 0; n < 3; n++)
               {
                  V0mn = Ta[al][m] * Ta[al][n].transpose();
                  tt += Wa[al](k,m) * Wa[al](l,n)
                     * xi[al][l].dot(M8i*V0mn*theta) * xi[al][k];
               }
   }
   d_th = sg2 * M8i * tt / sqr((double)Num);

   theta -= d_th;
   theta.normalize();

   H = convert_mat3(theta);
   return true;
}
