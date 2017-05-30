//
// Common Functions for ellipse fitting
//

#include "MT.h"
#include "EllipseFitting.h"

using namespace Eigen;

// Make symmetric matrix
Matrix6d
EllipseFitting::Sym(Matrix6d &A)
{
   return (A + A.transpose()) / 2.0;
}

Matrix6d
EllipseFitting::Sym2(Matrix6d &A)
{
   return (A + A.transpose());
}

// Convert 2-vector to 6-vector
Vector6d
EllipseFitting::convert_vec6(Vector2d &p, double f0)
{
   double x = p(0);
   double y = p(1);
   Vector6d t;

   t << sqr(x), 2.0*x*y, sqr(y), 2.0*f0*x, 2.0*f0*y, sqr(f0);

   return t;
}

// Convert 2-vector to 6-vector
Vector6d
EllipseFitting::convert_vec6_st(Vector2d &ph, Vector2d &pt, double f0)
{
   double xh = ph(0);
   double yh = ph(1);
   double xt = pt(0);
   double yt = pt(1);
   Vector6d t;

   t <<
      sqr(xh) + 2.0*xh*xt,
      2.0*(xh*yh + yh*xt + xh*yt),
      sqr(yh) + 2.0*yh*yt,
      2.0*f0*(xh + xt),
      2.0*f0*(yh + yt),
      sqr(f0);

   return t;
}

// Make covariance matrix
Matrix6d
EllipseFitting::make_cov_mat(Vector2d &p, double f0)
{
   Matrix6d T = ZeroMat6;
   double x = p(0);
   double y = p(1);
   
   T(0,0) = 4.0*sqr(x);
   T(1,1) = 4.0*(sqr(x) + sqr(y));
   T(2,2) = 4.0*sqr(y);
   T(0,1) = T(1,0) = T(2,1) = T(1,2) = 4.0*x*y;
   T(0,3) = T(3,0) = T(1,4) = T(4,1) = 4.0*f0*x;
   T(1,3) = T(3,1) = T(2,4) = T(4,2) = 4.0*f0*y;
   T(3,3) = T(4,4) = 4.0*sqr(f0);

   return T;
}

// Rank trancated genelralized inverse
Matrix6d
EllipseFitting::trancated_generalized_inverse(
   Matrix6d& M,         // Matrix:   Input
   int rank             // rank:     Input
                                             )
{
   // Eigen Solver for Self Adjoint Matrix
   SelfAdjointEigenSolver<Matrix6d> GES(M);

   Matrix6d Mi = ZeroMat6;
   for (int i = 6 - rank; i < 6; i++)
      Mi += GES.eigenvectors().col(i) * GES.eigenvectors().col(i).transpose()
         / GES.eigenvalues()(i);

   return Mi;
}

// Randomly sample unique indices
// The seed does not set in this function.
namespace {     // unnamed namespace
   bool
   check_unique(int idx[], int Num, int x)
   {
      for (int i = 0; i < Num; i++)
      {
         if (idx[i] == x)
            return false;
      }

      return true;
   }
}

void
EllipseFitting::sample_unique_indices(
   int idx[],           // index array:                 OUTPUT
   int SampleNum,       // Number of sample indices:    INPUT
   int DataNum          // Number of Data:              INPUT
                                     )
{
   int  x;

   for (int i = 0; i < SampleNum; i++)
   {
      // check
      do
      {
         x =  (int)(DataNum * genrand_real2());
      }
      while (!check_unique(idx, i, x));

      idx[i] = x;
   }
}
