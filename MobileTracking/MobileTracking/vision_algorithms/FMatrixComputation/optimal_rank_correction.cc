//
// Optimal Rank Correction
//

#include "FMatrixComputation.hpp"

using namespace Eigen;

// Optimal rank correction of fundamental matrix
bool
FMatrixComputation::optimal_rank_correction(
   const Vector9d &theta0,
   Vector9d &theta,
   std::vector<Vector2d> pos0,
   std::vector<Vector2d> pos1,
   int Num,
   double Conv_EPS,
   double f0
)
{
   Matrix9d     M, Pth;
   Vector9d     Xi, thdag, PthXi;
   Matrix9d     V0, V0th;
   double       d;

   theta = theta0;
   // Calc Projection Matrix
   Pth = I9 - theta * theta.transpose();
   
   // Calc Matrices
   M = ZeroMat9;
   for (int al = 0; al < Num; al++)
   {
      Xi = convert_vec9(pos0[al], pos1[al], f0);
      V0 = make_cov_mat(pos0[al], pos1[al], f0);
      PthXi = Pth * Xi;
      M += PthXi * PthXi.transpose() / theta.dot(V0 * theta);
   }
   M /= (double)Num;

   // Compute Eigen Values and Vectors
   SelfAdjointEigenSolver<Matrix9d> ES(M);
   if (ES.info() != Success) return false;

   // Compute V0[theta] (generalized inverse with rank 8)
   V0th = ZeroMat9;
   for (int i = 1; i < 9; i++) // except i=0, which means the smallest eigen value
   {
      V0th += ES.eigenvectors().col(i) * ES.eigenvectors().col(i).transpose()
         / ES.eigenvalues()(i);
   } 
   V0th /= (double)Num;

   // update loop
   do
   {
      // theta^\dag: equivalent to the cofactor matirx of F.
      thdag = theta_dag(theta);

      // update theta
      theta -= thdag.dot(theta) * V0th * thdag
         / (3.0 * thdag.dot(V0th * thdag));
      theta.normalize();

      // update Ptheta
      Pth = I9 - theta * theta.transpose();

      // Update V0
      V0th = Pth * V0th * Pth;

      // check det F = 0
      d = fabs(thdag.dot(theta));
   }
   while (d >= Conv_EPS);

   // success
   return true;
}
