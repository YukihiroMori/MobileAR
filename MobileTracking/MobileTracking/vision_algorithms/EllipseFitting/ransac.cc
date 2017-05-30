//
// RANSAC for ellipse fitting
//

#include "EllipseFitting.h"

// trap MT functions in unnamed namespace
namespace {
#include "MT.h"
}

using namespace Eigen;

// RANSAC method
bool
EllipseFitting::ransac(
   Vector2d pos[],     // Point Positions:             INPUT
   int Num,            // Number of data:              INPUT
   Vector6d &theta,    // Ellipse Parameter:            OUTPUT
   unsigned long Seed, // Seed for MT:                 INPUT w. default
   double dist,        // Threshold for distance:      INPUT w. default
   int Max_Count,      // Maximum Random Sample Count: INPUT w. default
   double f0           // Default focal length:        INPUT w. default
                          )
{
   Vector6d     Xi[Num];
   Matrix6d     V0[Num];
   Matrix6d     M;
   Vector6d     tmpth;
   int          idx[Ellipse_Sample_Number];
   int          sampleNum = Ellipse_Sample_Number;
   double       sqrDist = sqr(dist);
   double       d;
   int          count;
   int          sN, sN0;

   // construct self adjoint eigen solver
   SelfAdjointEigenSolver<Matrix6d> ES;
   
   // set seed for Mersenne twister
   init_genrand(Seed);

   // Initialization
   for (int al = 0; al < Num; al++)
   {
      Xi[al] = convert_vec6(pos[al], f0);
      V0[al] = make_cov_mat(pos[al], f0);
   }

   // loop
   count = 0;
   sN0 = 0;
   do
   {
      // sample unique indices
      sample_unique_indices(idx, sampleNum, Num);
      
      // compute moment matrix using sampling data
      M = ZeroMat6;
      for (int i = 0; i < sampleNum; i++)
         M += Xi[idx[i]] * Xi[idx[i]].transpose();

      // Compute eigen vector corresponding to the smallest eigen value
      ES.compute(M);
      if (ES.info() != Success) return false;
      tmpth = ES.eigenvectors().col(0);

      // count support
      sN = 0;
      for (int al = 0; al < Num; al++)
      {
         d = sqr(Xi[al].dot(tmpth)) / (tmpth.dot(V0[al]*tmpth));
         if (d < sqrDist)  sN++;
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

   return true;
}
