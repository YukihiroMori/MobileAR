//
// Taubin method for homography computation
//

#include <iostream>
#include <cstdlib>
#include <Eigen/Dense>

#include "HomographyComputation.h"

// Renormalization
bool
HomographyComputation::taubin(
   Vector2d pos0[],
   Vector2d pos1[],
   int Num,
   Matrix3d& H,
   double f0
)
{
   GeneralizedSelfAdjointEigenSolver<Matrix9d> ES;
   Vector9d theta, theta0, dtheta;
   Matrix9d M, N, V0kl;
   Matrix94d Ta[Num][3];
   Vector9d xi[Num][3];
   Matrix3d T;
   bool flag;
   int count;

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
   }

   // Clear M and N
   M = N = ZeroMat9;

   // Compute M and N
   for (int i = 0; i < Num; i++)
   {
      for (int k = 0; k < 3; k++)
      {      
         M += xi[i][k]*xi[i][k].transpose();
         N += Ta[i][k] * Ta[i][k].transpose();
      }
   }
   M /= (double)Num;
   N /= (double)Num;

   // Solving generalized eigen value problem
   ES.compute(N, M);

   // let theta be the eigenvector corresponding
   // to the largest eigenvalue.

   // Warning message for too small largest eigenvalue.
   if (ES.eigenvalues()[8] < 1.0)
   {
      std::cerr << "*** Warning ***\n"
                << "The largest eigen value ("
                << ES.eigenvalues()[8]
                << ") is too small!"
                << std::endl;
      H = ZeroMat3;
      return false;
   }

   // set theta
   theta = ES.eigenvectors().col(8);
   theta /= theta.norm();

   // convert mat3
   H = convert_mat3(theta);

   return flag;
}
