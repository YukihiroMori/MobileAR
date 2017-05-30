//
// Iterative reweight for homography computation
//

#include <iostream>
#include <Eigen/Dense>

#include "HomographyComputation.h"

// Iteration with weight update
bool
HomographyComputation::iterative_reweight(
   Vector2d pos0[],
   Vector2d pos1[],
   int Num,
   Matrix3d& H,
   int Max_Iter,
   double Conv_EPS,
   double f0
)
{
   SelfAdjointEigenSolver<Matrix9d> ES;
   Vector9d theta, dtheta, theta0;
   Matrix9d M;
   Vector9d xi[Num][3];
   Matrix94d Ta[Num][3];
   Matrix3d Wa[Num];
   Matrix3d T;
   Matrix9d V0kl;
   int count;
   bool flag;
   
   // make xi vectors
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

      // initialize weight
      Wa[i] = I3;
   }

   // set counter
   count = 0;
   flag = false;

   // loop
   do
   {
      // Clear M
      M = ZeroMat9;

      // Compute M and N
      for (int i = 0; i < Num; i++)
      {
	 for (int k = 0; k < 3; k++)
	    for (int l = 0; l < 3; l++)
	       M += Wa[i](k,l)*xi[i][k]*xi[i][l].transpose();
      }
      M /= (double)Num;

      // Solving eigen value problem
      ES.compute(M);

      // let theta be the eigenvector corresponding
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

   // convert mat3
   H = convert_mat3(theta);
   
   return flag;
}
