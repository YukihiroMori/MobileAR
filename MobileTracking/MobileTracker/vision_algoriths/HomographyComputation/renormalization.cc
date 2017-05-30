//
// Renormalization for homography computation
//

#include <iostream>
#include <cstdlib>
#include <Eigen/Dense>

#include "HomographyComputation.h"

// Renormalization
bool
HomographyComputation::renormalization(
   Vector2d pos0[],
   Vector2d pos1[],
   int Num,
   Matrix3d& H,
   int Max_Iter,
   double Conv_EPS,
   double f0
)
{
   GeneralizedSelfAdjointEigenSolver<Matrix9d> ES;
   Vector9d theta, theta0, dtheta;
   Matrix3d Wa[Num];
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

      // set weight
      Wa[i] = I3;
   }

   // theta0 = zero vector
   theta0 = ZeroVec9;

   // set counter
   count = 0;
   flag = false;

   // loop
   do
   {
      // Clear M and N
      M = N = ZeroMat9;

      // Compute M and N
      for (int i = 0; i < Num; i++)
      {
	 for (int k = 0; k < 3; k++)
	    for (int l = 0; l < 3; l++)
	    {
	       V0kl = Ta[i][k] * Ta[i][l].transpose();

	       M += Wa[i](k,l)*xi[i][k]*xi[i][l].transpose();
	       N += Wa[i](k,l)*V0kl;
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
   
   // convert mat3
   H = convert_mat3(theta);

   return flag;
}
