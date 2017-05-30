//
// Hyper least squares for homography computation
//

#include <iostream>
#include <cstdlib>
#include <Eigen/Dense>

#include "HomographyComputation.h"

// Least Squares with Super Accuracy
bool
HomographyComputation::hyper_least_squares(
   Vector2d pos0[],
   Vector2d pos1[],
   int Num,
   Matrix3d& H,
   double f0
)       
{
   GeneralizedSelfAdjointEigenSolver<Matrix9d> ES;
   Vector9d h, theta, theta0;
   Matrix3d Wa;
   Matrix9d M, N, N1, N2, Mi8, V0ln, V0km, symT;
   Matrix94d Ta[Num][3];
   Vector9d xi[Num][3];
   const DiagMatrix3d Pk(1.0, 1.0, 0.0);
   Matrix3d T, T1, T2;
   double x, y, xp, yp, f02;
   double xMx;

   // initialization
   f02 = f0 * f0;
   Wa = I3;

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

   // theta0 = zero vector
   theta0 = ZeroVec9;

   // Compute M
   M = ZeroMat9;
   for (int i = 0; i < Num; i++)
   {
      for (int k = 0; k < 3; k++)
	 for (int l = 0; l < 3; l++)
	    M += Wa(k,l)*xi[i][k]*xi[i][l].transpose();
   }
   M /= (double)Num;

   // compute generalized inverse with rank 8
   Mi8 = trancated_generalized_inverse(M, 8);

   // Compute N
   N1 = N2 = ZeroMat9;
   for (int i = 0; i < Num; i++)
   {
      for (int k = 0; k < 3; k++)
	 for (int l = 0; l < 3; l++)
	 {
	    N1 += Wa(k,l)*Ta[i][k]*Ta[i][l].transpose();

	    for (int m = 0; m < 3; m++)
	       for (int n = 0; n < 3; n++)
	       {
		  V0ln = Ta[i][l]*Ta[i][n].transpose();
		  V0km = Ta[i][k]*Ta[i][m].transpose();
		  xMx = xi[i][k].dot(Mi8*xi[i][m]);
		  symT = sym_mat9(V0km*Mi8*xi[i][l]*xi[i][n].transpose());
		  
		  N2 += Wa(k,l)*Wa(m,n)*
		     (xMx*V0ln + 2.0*symT);
	       }
	 }
   }
   N = N1/Num - N2/(Num*Num);

   // Solving generalized eigen value problem
   ES.compute(N, M);

   // let theta be the eigenvector corresponding
   // to the absolute largest eigenvalue.
   double a0, a8, amax;

   a0 = fabs(ES.eigenvalues()[0]);
   a8 = fabs(ES.eigenvalues()[8]);

   amax = (a8 > a0)? a8: a0;
   if (amax < 1.0)
   {
      std::cerr << "*** Warning ***\n"
		<< "The largest eigen value ("
		<< amax
		<< ") is too small!"
		<< std::endl;
      H = ZeroMat3;
      return false;
   }
   
   if (a8 > a0)
      theta = ES.eigenvectors().col(8);
   else
      theta = ES.eigenvectors().col(0);
	 
   theta /= theta.norm();

   // convert matrix
   H = convert_mat3(theta);
   
   return true;
}
