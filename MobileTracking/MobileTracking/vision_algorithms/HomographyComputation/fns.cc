//
// FNS for homography computation
//

#include <iostream>
#include <cstdlib>
#include <Eigen/Dense>

#include "HomographyComputation.h"

using namespace Eigen;

// FNS core routine
bool
HomographyComputation::fns_core(
   Vector9d xi[][3],
   Matrix94d Ta[][3],
   Matrix3d Wa[],
   int Num,
   Vector9d& theta,
   Matrix9d& M,
   int Max_Iter,
   double Conv_EPS
)
{
   SelfAdjointEigenSolver<Matrix9d> ES;
   Vector9d theta0, dtheta;
   Matrix9d X, L, V0kl;
   Matrix3d T;
   double va[3];
   int count;
   bool flag;

   // theta0 = zero vector
   theta0 = theta = ZeroVec9;

   // set counter
   count = 0;
   flag = false;

   // loop
   do
   {
      // Clear M and L
      M = L = ZeroMat9;

      // Compute M and L
      for (int i = 0; i < Num; i++)
      {
         // set va
         va[0] = va[1] = va[2] = 0.0;
         for (int l = 0; l < 3; l++)
         {
            va[0] += Wa[i](0,l) * theta.dot(xi[i][l]);
            va[1] += Wa[i](1,l) * theta.dot(xi[i][l]);
            va[2] += Wa[i](2,l) * theta.dot(xi[i][l]);
         }

         for (int k = 0; k < 3; k++)
            for (int l = 0; l < 3; l++)
            {
               V0kl = Ta[i][k] * Ta[i][l].transpose();

               M += Wa[i](k,l) * xi[i][k] * xi[i][l].transpose();
               L += va[k] * va[l] * V0kl;
            }
      }
      M /= (double)Num;
      L /= (double)Num;
      X = M - L;

      // Solving eigen value problem
      ES.compute(X);

      // let h be the eigenvector corresponding
      // to the smallest eigenvalue.
      theta = ES.eigenvectors().col(0);
      theta /= theta.norm();
      dtheta = theta - theta0;

      // check convergence
      if (dtheta.norm() < Conv_EPS)
      {
	 flag = true;
	 break;
      }

      // update Wa
      for (int i = 0; i < Num; i++)
      {
         for (int k = 0; k < 3; k++)
            for (int l = 0; l < 3; l++)
            {
               V0kl = Ta[i][k] * Ta[i][l].transpose();
               T(k,l) = theta.dot(V0kl*theta);
            }

	 Wa[i] = trancated_generalized_inverse(T,2);
      }

      // update theta
      theta0 = theta;
   }
   while (++count < Max_Iter);

   if (count >= Max_Iter)
      return false;
   
   return true;
}


// FNS
bool
HomographyComputation::fns(
   Vector2d pos0[],
   Vector2d pos1[],
   int Num,
   Matrix3d& H,
   int Max_Iter,
   double Conv_EPS,
   double f0
)
{
   Vector9d theta;
   Matrix94d Ta[Num][3];
   Vector9d xi[Num][3];
   Matrix3d Wa[Num];
   Matrix9d M;

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

   // convert mat3
   H = convert_mat3(theta);
   
   return true;
}
