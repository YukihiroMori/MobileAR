//
// RANSAC for homography computation
//

#include "HomographyComputation.h"

// trap MT functions in unnamed namespace
namespace {
#include "MT.h"
}

using namespace Eigen;

// RANSAC method
bool
HomographyComputation::ransac(
   Vector2d pos0[],    // Point Positions:             INPUT
   Vector2d pos1[],    // Point Positions:             INPUT
   int Num,            // Number of data:              INPUT
   Matrix3d &H,        // Ellipse Parameter:           OUTPUT
   bool inlier[],      // inlierflag:                  OUTPUT
   unsigned long Seed, // Seed for MT:                 INPUT w. default
   double dist,        // Threshold for distance:      INPUT w. default
   int Max_Count,      // Maximum Random Sample Count: INPUT w. default
   double f0           // Default focal length:        INPUT w. default
)
{
   SelfAdjointEigenSolver<Matrix9d> ES;
   Vector9d theta, theta0, dtheta, tmpth;
   Matrix3d Wa[Num];
   Matrix3d T;
   Matrix9d M, N, V0kl;
   Matrix94d Ta[Num][3];
   Vector9d xi[Num][3];
   double d;
   double sqrDist = sqr(dist);
   int idx[Homography_Sample_Number];
   int sampleNum = Homography_Sample_Number;
   bool flag;
   int count;
   int sN, sN0;

   // set seed for Mersenne twister
   init_genrand(Seed);

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

   // loop
   count = 0;
   sN0 = 0;
   do
   {
      // sample unique indices
      sample_unique_indices(idx, sampleNum, Num);
      
      // compute moment matrix using sampling data
      M = ZeroMat9;
      for (int i = 0; i < sampleNum; i++)
         for (int k = 0; k < 3; k++)
            M += xi[idx[i]][k] * xi[idx[i]][k].transpose();

      // Compute eigen vector corresponding to the smallest eigen value
      ES.compute(M);
      if (ES.info() != Success) return false;
      tmpth = ES.eigenvectors().col(0);

      // count support
      sN = 0;
      for (int i = 0; i < Num; i++)
      {

         // update weight using temporal theta
         for (int k = 0; k < 3; k++)
            for (int l = 0; l < 3; l++)
            {
               V0kl = Ta[i][k] * Ta[i][l].transpose();
               T(k,l) = tmpth.dot(V0kl*tmpth);
            }
         Wa[i] = trancated_generalized_inverse(T,2);
         
         inlier[i] = false;
         d = 0.0;
         for (int k = 0; k < 3; k++)
            for (int l = 0; l < 3; l++)
               d += Wa[i](k,l) * tmpth.dot(xi[i][k]) * tmpth.dot(xi[i][l]);
         
         if (d < 2.0*sqrDist)
         {
            sN++;
            inlier[i] = true;
         }
      }

      // update # of support and paramter
      if (sN > sN0)
      {
         sN0 = sN;
         theta = tmpth;
         count = 0; // reset count
      }
   }
   while (++count <= Max_Count);

   H = convert_mat3(theta);

   return true;
}
