//
// Planar Triangulation
// * Optimal Correction of corresponding point pairs
//

#include "PlanarTriangulation.h"

// optimal correction of correspoing point pairs
bool
PlanarTriangulation::optimal_correction(
   Vector2d pos0[],
   Vector2d pos1[],
   int Num,
   const Vector9d& theta,
   int      Max_Iter, // Max. Iteration:            INPUT w. default
   double   Conv_EPS, // Threshold for convergence: INPUT w. default
   double   f0        // Default focal length:      INPUT w. default
)
{
   SelfAdjointEigenSolver<Matrix9d> ES;
   Vector4d     pa[Num];       // x and y
   Vector4d     pah[Num];       // hat x and y
   Vector4d     pat[Num];       // tilde x and y
   Vector9d     xast[Num][3];   // Xi*
   Matrix94d    Ta[Num][3];
   Matrix3d     Wa[Num], T;
   Matrix9d     M, V0kl;
   double       J0, Jst;
   int          count;

   for (int i = 0; i < Num; i++)
   {
      pa[i] << pos0[i](0), pos0[i](1), pos1[i](0), pos1[i](1);
      pah[i] = pa[i];
      pat[i] << 0.0, 0.0, 0.0, 0.0;
   }

   // set large number to J0 
   J0 = Large_Number;
   
   // Update by iteration
   count = 0;
   do
   {
      // set Ta
      for (int i = 0; i < Num; i++)
      {
         Ta[i][0] = Ta[i][1] = Ta[i][2] = ZeroMat94;
         set_Ta1(Ta[i][0], pah[i], f0);
         set_Ta2(Ta[i][1], pah[i], f0);
         set_Ta3(Ta[i][2], pah[i], f0);
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
         
         xast[i][0] = set_xi1(pah[i], f0) + Ta[i][0] * pat[i];
         xast[i][1] = set_xi2(pah[i], f0) + Ta[i][1] * pat[i];
         xast[i][2] = set_xi3(pah[i], f0) + Ta[i][2] * pat[i];
      }

      // update \hat{p} and \tilde{p} and compute residual
      Jst = 0.0;
      for (int i = 0; i < Num; i++)
      {
         pat[i] = ZeroVec4;
         for (int k = 0; k < 3; k++)
            for (int l = 0; l < 3; l++)
            {
               pat[i] += Wa[i](k,l) * theta.dot(xast[i][k])
                  * Ta[i][l].transpose() * theta;
            }

         pah[i] = pa[i] - pat[i];
         Jst += pat[i].squaredNorm();
      }
      Jst /= (double)Num;

      // check convergence 
      if (fabs(Jst - J0) < Conv_EPS) break;
      
      J0 = Jst;
   }
   while (++count < Max_Iter);

   // Maximum Iteration
   if (count >= Max_Iter)  return false;

   // update positions
   for (int i = 0; i < Num; i++)
   {
      pos0[i] << pah[i](0), pah[i](1);
      pos1[i] << pah[i](2), pah[i](3);
   }

   return true;
}
   
// optimal correction of correspoing point pairs
bool
PlanarTriangulation::optimal_correction(
   Vector2d pos0[],
   Vector2d pos1[],
   int Num,
   const Matrix3d& H,
   int      Max_Iter, // Max. Iteration:            INPUT w. default
   double   Conv_EPS, // Threshold for convergence: INPUT w. default
   double   f0        // Default focal length:      INPUT w. default
)
{
   bool flag;
   Vector9d theta;
   
   // Initialization
   theta = convert_vec9(H);
   
   flag = optimal_correction(pos0, pos1, Num, theta,
                             Max_Iter, Conv_EPS, f0);

   return flag;
}
