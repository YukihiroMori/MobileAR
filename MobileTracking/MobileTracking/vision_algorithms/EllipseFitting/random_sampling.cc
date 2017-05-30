//
// Random sampling method for ellipse fitting
//

#include "EllipseFitting.h"

// trap MT functions in unnamed namespace
namespace {
#include "MT.h"
}

using namespace Eigen;

// Random sampling method
bool
EllipseFitting::random_sampling(
   Vector2d pos[],     // Point Positions:             INPUT
   int Num,            // Number of data:              INPUT
   Vector6d &theta,    // Ellipse Parameter:            OUTPUT
   unsigned long Seed, // Seed for MT:                 INPUT w. default
   int Max_Iter,       // Max. Iteration:              INPUT w. default
   double Conv_EPS,    // Threshold for convergence:   INPUT w. default
   int Max_Count,      // Maximum Random Sample Count: INPUT w. default
   double f0           // Default focal length:        INPUT w. default
                               )
{
   Vector6d     Xi[Num];
   Matrix6d     V0[Num];
   double       W[Num];
   Matrix6d     M;
   int          idx[Ellipse_Sample_Number];
   int          sampleNum = Ellipse_Sample_Number;
   double       J, J0;
   int          count;

   // construct self adjoint eigen solver
   SelfAdjointEigenSolver<Matrix6d> ES;
   
   // set seed for Mersenne twister
   init_genrand(Seed);

   // Initialization
   for (int al = 0; al < Num; al++)
   {
      Xi[al] = convert_vec6(pos[al], f0);
      V0[al] = make_cov_mat(pos[al], f0);
      W[al] = 1.0;        
   }

   // fitting by hyper renormalization
   if (!hyper_renormalization_core(Xi, V0, W, Num, theta, Max_Iter, Conv_EPS))
      return false;
   
   // parameter check
   if (theta(0)*theta(2) - sqr(theta(1)) > 0.0)
      return true;      // ellipse was fitted

   // loop
   count = 0;
   J0 = Large_Number;
   do
   {
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
         theta = ES.eigenvectors().col(0);

         // parameter check
      }
      while (theta(0)*theta(2) - sqr(theta(1)) <= 0.0);

      // compute Sampson error
      J = 0.0;
      for (int al = 0; al < Num; al++)
         J += sqr(Xi[al].dot(theta)) / (theta.dot(V0[al]*theta));

      if (J < J0)  J0 = J;
   }
   while (++count < Max_Count);

   return true;
}
